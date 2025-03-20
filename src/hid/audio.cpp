#include "hid/audio.h"

namespace daisy
{
// ================================================================
// Static Globals
// ================================================================

// Ordinarily the user would be required to provide their own buffer, but
// in the interest in encourage newcomers, and this also being an audio-centric platform
// these buffers will always be present, and usable.
//
static const size_t kAudioMaxBufferSize = 1024;
static const size_t kAudioMaxChannels   = 6; // leaves more space ... but TDM data will be written in buff_rx[1][blocksize*2 + i] instead of buff_rx[2][i] ... 

// Static Global Buffers
// 8kB in SRAM1, non-cached memory
// 1k samples in, 1k samples out, 4 bytes per sample.
// One buffer per 2 channels (Interleaved on hardware)
static int32_t DMA_BUFFER_MEM_SECTION
    dsy_audio_rx_buffer[kAudioMaxChannels / 2][kAudioMaxBufferSize];
static int32_t DMA_BUFFER_MEM_SECTION
    dsy_audio_tx_buffer[kAudioMaxChannels / 2][kAudioMaxBufferSize];

// ================================================================
// Private Implementation Definition
// ================================================================
class AudioHandle::Impl
{
  public:
    // Interface
    AudioHandle::Result Init(const AudioHandle::Config config, SaiHandle sai);
    AudioHandle::Result
                        Init(const AudioHandle::Config config, SaiHandle sai1, SaiHandle sai2);
    AudioHandle::Result DeInit();
    AudioHandle::Result Start(AudioHandle::AudioCallback callback);
    AudioHandle::Result Start(AudioHandle::InterleavingAudioCallback callback);
    AudioHandle::Result Stop();
    AudioHandle::Result ChangeCallback(AudioHandle::AudioCallback callback);
    AudioHandle::Result
    ChangeCallback(AudioHandle::InterleavingAudioCallback callback);

    inline size_t GetChannels() const
    {
        size_t ch = 0;
        
        if(sai1_.IsInitialized())
            ch += 2;

        if(sai2_.IsInitialized())
        {
            size_t tdm_slots = sai2_.GetConfig().tdm_slots;
            ch += tdm_slots > 0 ? tdm_slots : 2; // backwards compatible: if tdm_slots is 0 the act like sai2_ is 2 channels
        }
        return ch;
    }

    AudioHandle::Result SetBlockSize(size_t size)
    {
        size_t maxSize    = kAudioMaxBufferSize / 4;
        config_.blocksize = size <= maxSize ? size : maxSize;
        return size <= maxSize ? AudioHandle::Result::OK
                               : AudioHandle::Result::ERR;
    }

    float GetSampleRate() { return sai1_.GetSampleRate(); }

    AudioHandle::Result SetPostGain(float val)
    {
        if(val <= 0.f)
            return AudioHandle::Result::ERR;
        config_.postgain = val;
        // Precompute input adjust
        postgain_recip_ = 1.f / config_.postgain;
        // Precompute output adjust
        output_adjust_ = config_.postgain * config_.output_compensation;
        return AudioHandle::Result::OK;
    }

    AudioHandle::Result SetOutputCompensation(float val)
    {
        config_.output_compensation = val;
        // recompute output adjustment (no need to recompute input adjust here)
        output_adjust_ = config_.output_compensation * config_.postgain;
        return AudioHandle::Result::OK;
    }

    AudioHandle::Result SetSampleRate(SaiHandle::Config::SampleRate sampelrate);

    // Internal Callback
    static void InternalCallback(int32_t* in, int32_t* out, size_t size);

    void *callback_, *interleaved_callback_;

    // Data
    AudioHandle::Config config_;
    SaiHandle           sai1_, sai2_;
    int32_t*            buff_rx_[2];
    int32_t*            buff_tx_[2];
    float               postgain_recip_;
    float               output_adjust_;
};

// ================================================================
// Static Reference for Object
// ================================================================

static AudioHandle::Impl audio_handle;

// ================================================================
// Private Implementation
// ================================================================

AudioHandle::Result AudioHandle::Impl::Init(const AudioHandle::Config config,
                                            SaiHandle                 sai)
{
    config_ = config;

    /** Precompute input level adjustment */
    if(config_.postgain > 0.f)
        postgain_recip_ = 1.f / config_.postgain;
    else
        return Result::ERR;

    /** Precompute output level adjustment */
    output_adjust_ = config_.postgain * config_.output_compensation;

    if(sai.IsInitialized())
    {
        sai1_              = sai;
        config_.samplerate = sai1_.GetConfig().sr;
    }
    else
    {
        return Result::ERR;
    }
    buff_rx_[0] = dsy_audio_rx_buffer[0];
    buff_tx_[0] = dsy_audio_tx_buffer[0];
    return Result::OK;
}

AudioHandle::Result AudioHandle::Impl::Init(const AudioHandle::Config config,
                                            SaiHandle                 sai1,
                                            SaiHandle                 sai2)
{
    this->Init(config, sai1);
    sai2_       = sai2;
    buff_rx_[1] = dsy_audio_rx_buffer[1];
    buff_tx_[1] = dsy_audio_tx_buffer[1];
    // How do we want to handle the rx/tx buffs for the second peripheral of audio..?
    return Result::OK;
}

AudioHandle::Result AudioHandle::Impl::DeInit()
{
    Stop();
    if(sai1_.IsInitialized())
    {
        if(sai1_.DeInit() != SaiHandle::Result::OK)
        {
            return Result::ERR;
        }
    }
    if(sai2_.IsInitialized())
    {
        if(sai2_.DeInit() != SaiHandle::Result::OK)
        {
            return Result::ERR;
        }
    }
    return Result::OK;
}

AudioHandle::Result
AudioHandle::Impl::Start(AudioHandle::AudioCallback callback)
{
    // Get instance of object
    if(sai2_.IsInitialized())
    {
        // Start stream with no callback. Data will be filled externally - from InternalCallback.
        sai2_.StartDma(buff_rx_[1], buff_tx_[1], config_.blocksize, nullptr);
    }
    sai1_.StartDma(buff_rx_[0],
                   buff_tx_[0],
                   config_.blocksize,
                   audio_handle.InternalCallback);
    callback_             = (void*)callback;
    interleaved_callback_ = nullptr;
    return Result::OK;
}

AudioHandle::Result
AudioHandle::Impl::Start(AudioHandle::InterleavingAudioCallback callback)
{
    // Get instance of object
    sai1_.StartDma(buff_rx_[0],
                   buff_tx_[0],
                   config_.blocksize,
                   audio_handle.InternalCallback);
    interleaved_callback_ = (void*)callback;
    callback_             = nullptr;
    return Result::OK;
}

AudioHandle::Result AudioHandle::Impl::Stop()
{
    if(sai1_.IsInitialized())
        sai1_.StopDma();
    if(sai2_.IsInitialized())
        sai2_.StopDma();
    return Result::OK;
}

AudioHandle::Result
AudioHandle::Impl::ChangeCallback(AudioHandle::AudioCallback callback)
{
    if(callback != nullptr)
    {
        callback_             = (void*)callback;
        interleaved_callback_ = nullptr;
        return Result::OK;
    }
    else
    {
        return Result::ERR;
    }
}

AudioHandle::Result AudioHandle::Impl::ChangeCallback(
    AudioHandle::InterleavingAudioCallback callback)
{
    if(callback != nullptr)
    {
        interleaved_callback_ = (void*)callback;
        callback_             = nullptr;
        return Result::OK;
    }
    else
    {
        return Result::ERR;
    }
}

AudioHandle::Result
AudioHandle::Impl::SetSampleRate(SaiHandle::Config::SampleRate samplerate)
{
    config_.samplerate = samplerate;
    if(sai1_.IsInitialized())
    {
        // Set, and reinit
        SaiHandle::Config cfg;
        cfg    = sai1_.GetConfig();
        cfg.sr = config_.samplerate;
        if(sai1_.Init(cfg) != SaiHandle::Result::OK)
        {
            return Result::ERR;
        }
    }
    if(sai2_.IsInitialized())
    {
        // Set, and reinit
        SaiHandle::Config cfg;
        cfg    = sai2_.GetConfig();
        cfg.sr = config_.samplerate;
        if(sai2_.Init(cfg) != SaiHandle::Result::OK)
        {
            return Result::ERR;
        }
    }
    return Result::OK;
}

// This turned into a very large function due to the bit-depth conversions..
// I didn't want to do a conditional conversion in a single loop because it seemed
// far less efficient, so this is where I landed.
//
// Using function pointers for the x2f and f2x functions would have been ideal, but
// wasn't possible due to the different parameter/return types for each function.
void AudioHandle::Impl::InternalCallback(int32_t* in, int32_t* out, size_t size)
{
    // Convert from sai format to float, and call user callback
    size_t                      chns;
    SaiHandle::Config::BitDepth bd;
    bd   = audio_handle.sai1_.GetConfig().bit_depth;
    chns = audio_handle.GetChannels();
    if(chns == 0)
        return;
    // Handle Interleaved / Non Interleaved separate
    if(audio_handle.interleaved_callback_)
    {
        //TODO: Interleaved_callback does not work for more than 2 chns yet. (also not TDM)
        if(chns > 2)
            return;
            
        float fin[size];
        float fout[size];
        // There _must_ be a more elegant way to do this....
        float gain = audio_handle.postgain_recip_; // to fit multiplications in 1 line later
        // Convert from int to float
        switch(bd)
        {
            case SaiHandle::Config::BitDepth::SAI_16BIT:
                for(size_t i = 0; i < size; i += 2)
                {
                    fin[i + 0] = s162f(in[i + 0]) * gain;
                    fin[i + 1] = s162f(in[i + 1]) * gain;
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_24BIT:
                for(size_t i = 0; i < size; i += 2)
                {
                    fin[i + 0] = s242f(in[i + 0]) * gain;
                    fin[i + 1] = s242f(in[i + 1]) * gain;
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_32BIT:
                for(size_t i = 0; i < size; i += 2)
                {
                    fin[i + 0] = s322f(in[i + 0]) * gain;
                    fin[i + 1] = s322f(in[i + 1]) * gain;
                }
                break;
            default: break;
        }

        InterleavingAudioCallback cb
            = (InterleavingAudioCallback)audio_handle.interleaved_callback_;
        cb(fin, fout, size);

        float adjust = audio_handle.output_adjust_; // to fit multiplications in 1 line later
        switch(bd)
        {
            case SaiHandle::Config::BitDepth::SAI_16BIT:
                for(size_t i = 0; i < size; i += 2)
                {
                    out[i + 0]     = f2s16(fout[i + 0] * adjust);
                    out[i + 1] = f2s16(fout[i + 1] * adjust);
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_24BIT:
                for(size_t i = 0; i < size; i += 2)
                {
                    out[i + 0]     = f2s24(fout[i + 0] * adjust);
                    out[i + 1] = f2s24(fout[i + 1] * adjust);
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_32BIT:
                for(size_t i = 0; i < size; i += 2)
                {
                    out[i + 0]     = f2s32(fout[i + 0] * adjust);
                    out[i + 1] = f2s32(fout[i + 1] * adjust);
                }
                break;
            default: break;
        }
    }
    else if(audio_handle.callback_)
    {
        // offset needed for 2nd audio codec.
        size_t offset    = audio_handle.sai2_.GetOffset();
        size_t block_size = size / 2;
        size_t buff_size = chns * block_size;
        float  finbuff[buff_size], foutbuff[buff_size];
        float* fin[chns];
        float* fout[chns];
        fin[0]  = finbuff;
        fout[0] = foutbuff;

        for(size_t ch = 1; ch < chns; ch++)
        {
            fin[ch]  = fin[ch - 1] + block_size;
            fout[ch] = fout[ch - 1] + block_size;
        }

        int32_t *in2 = audio_handle.buff_rx_[1] + offset; // in2 = SAI2 buffer position
        float gain = audio_handle.postgain_recip_; // to fit multiplications in 1 line later
        
        // TODO: handle different bd for sai interfaces (eg. sai1.bd=24bit, sai2.bd=16bit)
        // Deinterleave and scale
        switch(bd)
        {
            case SaiHandle::Config::BitDepth::SAI_16BIT:
                for(size_t i = 0; i < block_size; i ++)
                {
                    fin[0][i] = s162f(in[i * 2 + 0]) * gain;
                    fin[1][i] = s162f(in[i * 2 + 1]) * gain;
                    if (chns > 4) // TDM
                    {
                        // TODO: works with codec_ak4619 - test if this generalizes
                        fin[2][i] = s322f(in2[i * 4 + 0]) * gain;
                        fin[3][i] = s322f(in2[i * 4 + 1]) * gain;
                        fin[4][i] = s322f(in2[i * 4 + 2]) * gain;
                        fin[5][i] = s322f(in2[i * 4 + 3]) * gain;
                    }
                    else if(chns > 2)
                    {
                        fin[2][i] = s162f(in2[i * 2 + 0]) * gain;
                        fin[3][i] = s162f(in2[i * 2 + 1]) * gain;
                    }
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_24BIT:
                for(size_t i = 0; i < block_size; i++)
                {
                    fin[0][i] = s242f(in[i * 2 + 0]) * gain;
                    fin[1][i] = s242f(in[i * 2 + 1]) * gain;
                    if(chns > 4) // TDM
                    {
                        // TODO: works with codec_ak4619 - test if this generalizes
                        fin[2][i] = s322f(in2[i * 4 + 0]) * gain;
                        fin[3][i] = s322f(in2[i * 4 + 1]) * gain;
                        fin[4][i] = s322f(in2[i * 4 + 2]) * gain;
                        fin[5][i] = s322f(in2[i * 4 + 3]) * gain;
                    }
                    else if(chns > 2)
                    {
                        fin[2][i] = s242f(in2[i * 2 + 0]) * gain;
                        fin[3][i] = s242f(in2[i * 2 + 1]) * gain;
                    }
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_32BIT:
                for(size_t i = 0; i < block_size; i++)
                {
                    fin[0][i] = s322f(in[i * 2 + 0]) * gain;
                    fin[1][i] = s322f(in[i * 2 + 1]) * gain;
                    if (chns > 4) // TDM
                    {
                        // TODO: works with codec_ak4619 - test if this generalizes
                        fin[2][i] = s322f(in2[i * 4 + 0]) * gain;
                        fin[3][i] = s322f(in2[i * 4 + 1]) * gain;
                        fin[4][i] = s322f(in2[i * 4 + 2]) * gain;
                        fin[5][i] = s322f(in2[i * 4 + 3]) * gain;
                    }
                    else if(chns > 2)
                    {
                        fin[2][i] = s322f(in2[i * 2 + 0]) * gain;
                        fin[3][i] = s322f(in2[i * 2 + 1]) * gain;
                    }
                    
                }
                break;
            default: break;
        }

        AudioCallback cb = (AudioCallback)audio_handle.callback_;
        cb(fin, fout, size / 2);

        int32_t *out2 = audio_handle.buff_tx_[1] + offset; // out2 = SAI2 buffer position
        float adjust = audio_handle.output_adjust_; // to fit multiplications in 1 line later

        // Reinterleave and scale
        switch(bd)
        {
            case SaiHandle::Config::BitDepth::SAI_16BIT:
                for(size_t i = 0; i < block_size; i++)
                {
                    out[i * 2 + 0] = f2s16(fout[0][i] * adjust);
                    out[i * 2 + 1] = f2s16(fout[1][i] * adjust);
                    if (chns > 4) // TDM
                    {
                        // TODO: works with codec_ak4619 - test if this generalizes
                        out2[i * 4 + 0] = f2s32(fout[2][i] * adjust);
                        out2[i * 4 + 1] = f2s32(fout[3][i] * adjust);
                        out2[i * 4 + 2] = f2s32(fout[4][i] * adjust);
                        out2[i * 4 + 3] = f2s32(fout[5][i] * adjust);
                    }
                    else if(chns > 2)
                    {
                        out2[i * 2 + 0] = f2s16(fout[2][i] * adjust);
                        out2[i * 2 + 1] = f2s16(fout[3][i] * adjust);
                    }
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_24BIT:
                for(size_t i = 0; i < block_size; i++)
                {
                    out[i * 2 + 0] = f2s24(fout[0][i] * adjust);
                    out[i * 2 + 1] = f2s24(fout[1][i] * adjust);
                    if (chns > 4) // TDM
                    {
                        // TODO: works with codec_ak4619 - test if this generalizes
                        out2[i * 4 + 0] = f2s32(fout[2][i] * adjust);
                        out2[i * 4 + 1] = f2s32(fout[3][i] * adjust);
                        out2[i * 4 + 2] = f2s32(fout[4][i] * adjust);
                        out2[i * 4 + 3] = f2s32(fout[5][i] * adjust);
                    }
                    else if(chns > 2)
                    {
                        out2[i * 2 + 0] = f2s24(fout[2][i] * adjust);
                        out2[i * 2 + 1] = f2s24(fout[3][i] * adjust);
                    }
                }
                break;
            case SaiHandle::Config::BitDepth::SAI_32BIT:
                for(size_t i = 0; i < block_size; i++)
                {
                    out[i * 2 + 0] = f2s32(fout[0][i] * adjust);
                    out[i * 2 + 1] = f2s32(fout[1][i] * adjust);
                    if (chns > 4) // TDM
                    {
                        // TODO: works with codec_ak4619 - test if this generalizes
                        out2[i * 4 + 0] = f2s32(fout[2][i] * adjust);
                        out2[i * 4 + 1] = f2s32(fout[3][i] * adjust);
                        out2[i * 4 + 2] = f2s32(fout[4][i] * adjust);
                        out2[i * 4 + 3] = f2s32(fout[5][i] * adjust);
                    }
                    else if(chns > 2)
                    {
                        out2[i * 2 + 0] = f2s32(fout[2][i] * adjust);
                        out2[i * 2 + 1] = f2s32(fout[3][i] * adjust);
                    }
                }
                break;
            default: break;
        }
    }
}

// ================================================================
// SaiHandle -> SaiHandle::Pimpl
// ================================================================

AudioHandle::Result AudioHandle::Init(const Config& config, SaiHandle sai)
{
    // Figure out proper pattern for singleton behavior here.
    pimpl_ = &audio_handle;
    return pimpl_->Init(config, sai);
}

AudioHandle::Result
AudioHandle::Init(const Config& config, SaiHandle sai1, SaiHandle sai2)
{
    // Figure out proper pattern for singleton behavior here.
    pimpl_ = &audio_handle;
    return pimpl_->Init(config, sai1, sai2);
}

AudioHandle::Result AudioHandle::DeInit()
{
    return pimpl_->DeInit();
}

const AudioHandle::Config& AudioHandle::GetConfig() const
{
    return pimpl_->config_;
}

size_t AudioHandle::GetChannels() const
{
    return pimpl_->GetChannels();
}

AudioHandle::Result AudioHandle::SetBlockSize(size_t size)
{
    return pimpl_->SetBlockSize(size);
}

float AudioHandle::GetSampleRate()
{
    return pimpl_->GetSampleRate();
}

AudioHandle::Result
AudioHandle::SetSampleRate(SaiHandle::Config::SampleRate samplerate)
{
    return pimpl_->SetSampleRate(samplerate);
}

AudioHandle::Result AudioHandle::Start(AudioCallback callback)
{
    return pimpl_->Start(callback);
}

AudioHandle::Result AudioHandle::Start(InterleavingAudioCallback callback)
{
    return pimpl_->Start(callback);
}

AudioHandle::Result AudioHandle::Stop()
{
    return pimpl_->Stop();
}

AudioHandle::Result AudioHandle::ChangeCallback(AudioCallback callback)
{
    return pimpl_->ChangeCallback(callback);
}

AudioHandle::Result
AudioHandle::ChangeCallback(InterleavingAudioCallback callback)
{
    return pimpl_->ChangeCallback(callback);
}

AudioHandle::Result AudioHandle::SetPostGain(float val)
{
    return pimpl_->SetPostGain(val);
}

AudioHandle::Result AudioHandle::SetOutputCompensation(float val)
{
    return pimpl_->SetOutputCompensation(val);
}

} // namespace daisy
