/* Generated by LinkerScriptGenerator [http://visualgdb.com/tools/LinkerScriptGenerator]
 * Target: STM32H750IB
 * The file is provided under the BSD license.
 */

ENTRY(Reset_Handler)

MEMORY
{
	FLASH       (RX)  : ORIGIN = 0x08000000, LENGTH = 128K
	DTCMRAM     (RWX) : ORIGIN = 0x20000000, LENGTH = 128K
	SRAM        (RWX) : ORIGIN = 0x24000000, LENGTH = 512K - 32K
	RAM_D2_DMA  (RWX) : ORIGIN = 0x30000000, LENGTH = 64K     /* Changed from 32K to 64K */
	RAM_D2      (RWX) : ORIGIN = 0x30010000, LENGTH = 224K    /* Adjusted start and size to account for larger DMA section */
	RAM_D3      (RWX) : ORIGIN = 0x38000000, LENGTH = 64K
	BACKUP_SRAM (RWX) : ORIGIN = 0x38800000, LENGTH = 4K
	ITCMRAM     (RWX) : ORIGIN = 0x00000000, LENGTH = 64K
	SDRAM       (RWX) : ORIGIN = 0xc0000000, LENGTH = 64M
	QSPIFLASH   (RX)  : ORIGIN = 0x90040000, LENGTH = 7936K
}

_estack = 0x20020000;

SECTIONS
{
	.isr_vector :
	{
		. = ALIGN(4);
		KEEP(*(.isr_vector))
		. = ALIGN(4);
	} > SRAM

	.text :
	{
		. = ALIGN(4);
		_stext = .;

		*(.text)
		*(.text*)
		*(.rodata)
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)
		KEEP(*(.init))
		KEEP(*(.fini))
		. = ALIGN(4);
		_etext = .;

	} > SRAM

    /* SDMMC DMA Buffer (в RAM_D1) */
    .sdmmc_buf (NOLOAD) :
    {
        . = ALIGN(32); /* выравнивание на 32 байта для DMA */
        _ssdmmc_buf = .;

        PROVIDE(__sdmmc_buf_start__ = _ssdmmc_buf);
        KEEP(*(.sdmmc_buf))
        KEEP(*(.sdmmc_buf*))

        . = ALIGN(32);
        _esdmmc_buf = .;

        PROVIDE(__sdmmc_buf_end__ = _esdmmc_buf);
    } > SRAM

	.ARM.extab :
	{
		. = ALIGN(4);
		*(.ARM.extab)
		*(.gnu.linkonce.armextab.*)
		. = ALIGN(4);
	} > SRAM

	.exidx :
	{
		. = ALIGN(4);
		PROVIDE(__exidx_start = .);
		*(.ARM.exidx*)
		. = ALIGN(4);
		PROVIDE(__exidx_end = .);
	} > SRAM

	.ARM.attributes 0 : { *(.ARM.attributes) }

	.preinit_array :
	{
		PROVIDE_HIDDEN(__preinit_array_start = .);
		KEEP(*(.preinit_array*))
		PROVIDE_HIDDEN(__preinit_array_end = .);
	} > SRAM

	.init_array :
	{
		PROVIDE_HIDDEN(__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array*))
		PROVIDE_HIDDEN(__init_array_end = .);
	} > SRAM

	.fini_array :
	{
		PROVIDE_HIDDEN(__fini_array_start = .);
		KEEP(*(.fini_array*))
		KEEP(*(SORT(.fini_array.*)))
		PROVIDE_HIDDEN(__fini_array_end = .);
	} > SRAM

	.sram1_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_ssram1_bss = .;

		PROVIDE(__sram1_bss_start__ = _sram1_bss);
		*(.sram1_bss)
		*(.sram1_bss*)
		. = ALIGN(4);
		_esram1_bss = .;

		PROVIDE(__sram1_bss_end__ = _esram1_bss);
	} > RAM_D2_DMA

	/* Add definition for .ram_d2_bss section */
	.ram_d2_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sram_d2_bss = .;

		PROVIDE(__ram_d2_bss_start__ = _sram_d2_bss);
		*(.ram_d2_bss)
		*(.ram_d2_bss*)
		. = ALIGN(4);
		_eram_d2_bss = .;

		PROVIDE(__ram_d2_bss_end__ = _eram_d2_bss);
	} > RAM_D2

	.data :
	{
		. = ALIGN(4);
		_sdata = .;

		PROVIDE(__data_start__ = _sdata);
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;

		PROVIDE(__data_end__ = _edata);
	} > DTCMRAM AT > SRAM

	_sidata = LOADADDR(.data);

	.bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sbss = .;

		PROVIDE(__bss_start__ = _sbss);
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;

		PROVIDE(__bss_end__ = _ebss);
	} > DTCMRAM

	.dtcmram_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sdtcmram_bss = .;

		PROVIDE(__dtcmram_bss_start__ = _sdtcmram_bss);
		*(.dtcmram_bss)
		*(.dtcmram_bss*)
		. = ALIGN(4);
		_edtcmram_bss = .;

		PROVIDE(__dtcmram_bss_end__ = _edtcmram_bss);
	} > DTCMRAM

	/*
	.sdram_text :
	{
		. = ALIGN(4);
		_ssdram_text = .;

		PROVIDE(__sdram_text_start = _ssdram_text);
		*(.sdram_text)
		*(.sdram_text*)
		. = ALIGN(4);
		_esdram_text = .;

		PROVIDE(__sdram_text_end = _esdram_text);
	} > SDRAM AT >FLASH
	_sisdram_text = LOADADDR(.sdram_text);
	*/

	.sdram_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_ssdram_bss = .;

		PROVIDE(__sdram_bss_start = _ssdram_bss);
		*(.sdram_bss)
		*(.sdram_bss*)
		. = ALIGN(4);
		_esdram_bss = .;

		PROVIDE(__sdram_bss_end = _esdram_bss);
	} > SDRAM

	.backup_sram (NOLOAD) :
	{
		. = ALIGN(4);
		_sbackup_sram = .;

		PROVIDE(__backup_sram_start = _sbackup_sram);
		*(.backup_sram)
		*(.backup_sram*)
		. = ALIGN(4);
		_ebackup_sram = .;

		PROVIDE(__backup_sram_end = _ebackup_sram);
	} > BACKUP_SRAM

	.qspiflash_text :
	{
		. = ALIGN(4);
		_sqspiflash_text = .;

		PROVIDE(__qspiflash_text_start = _sqspiflash_text);
		*(.qspiflash_text)
		*(.qspiflash_text*)
		. = ALIGN(4);
		_eqspiflash_text = .;

		PROVIDE(__qspiflash_text_end = _eqspiflash_text);
	} > QSPIFLASH

	.qspiflash_data :
	{
		. = ALIGN(4);
		_sqspiflash_data = .;

		PROVIDE(__qspiflash_data_start = _sqspiflash_data);
		*(.qspiflash_data)
		*(.qspiflash_data*)
		. = ALIGN(4);
		_eqspiflash_data = .;

		PROVIDE(__qspiflash_data_end = _eqspiflash_data);
	} > QSPIFLASH

	.qspiflash_bss (NOLOAD) :
	{
		. = ALIGN(4);
		_sqspiflash_bss = .;

		PROVIDE(__qspiflash_bss_start = _sqspiflash_bss);
		*(.qspiflash_bss)
		*(.qspiflash_bss*)
		. = ALIGN(4);
		_eqspiflash_bss = .;

		PROVIDE(__qspiflash_bss_end = _eqspiflash_bss);
	} > QSPIFLASH

	.heap (NOLOAD) :
	{
		. = ALIGN(4);
		PROVIDE(__heap_start__ = .);
		PROVIDE(__heap_start = .);
		KEEP(*(.heap))
		. = ALIGN(4);
		PROVIDE(__heap_end = .);
		PROVIDE(__heap_end__ = .);
	} > RAM_D2

	PROVIDE(end = .);

	.reserved_for_stack (NOLOAD) :
	{
		. = ALIGN(4);
		PROVIDE(__reserved_for_stack_start__ = .);
		KEEP(*(.reserved_for_stack))
		. = ALIGN(4);
		PROVIDE(__reserved_for_stack_end__ = .);
	} > DTCMRAM

    DISCARD :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }

}