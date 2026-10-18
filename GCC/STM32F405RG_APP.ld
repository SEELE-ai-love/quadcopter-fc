/* STM32F405RG: 1 MiB Flash, 128 KiB DMA-capable SRAM.
 * Keep the existing 64 KiB bootloader reservation and 0x08010000 vector address.
 * CCM RAM is intentionally not used for ordinary data or DMA buffers.
 */
ENTRY(Reset_Handler)
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08010000, LENGTH = 960K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}
_estack = __StackTop;
_Min_Heap_Size = 8K;
_Min_Stack_Size = 4K;
SECTIONS
{
    .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } > FLASH
    .text :
    {
        . = ALIGN(4);
        *(.text .text.*) *(.glue_7 .glue_7t) *(.eh_frame)
        KEEP(*(.init)) KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;
    } > FLASH
    .rodata : { . = ALIGN(4); *(.rodata .rodata.*) . = ALIGN(4); } > FLASH
    .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
    .ARM.exidx :
    {
        __exidx_start = .; *(.ARM.exidx* .gnu.linkonce.armexidx.*) __exidx_end = .;
    } > FLASH
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH
    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH
    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH
    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4); _sdata = .;
        *(.data .data.*)
        . = ALIGN(4); _edata = .;
    } > RAM AT> FLASH
    .bss (NOLOAD) :
    {
        . = ALIGN(4); _sbss = .; __bss_start__ = .;
        *(.bss .bss.*) *(COMMON)
        . = ALIGN(4); _ebss = .; __bss_end__ = .;
    } > RAM
    .heap (NOLOAD) :
    {
        . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .);
        __HeapBase = .; . += _Min_Heap_Size; __HeapLimit = .;
    } > RAM
    .msp_stack (NOLOAD) :
    {
        . = ALIGN(8); __StackLimit = .;
        . += _Min_Stack_Size; . = ALIGN(8); __StackTop = .;
    } > RAM
    ASSERT(__StackTop <= ORIGIN(RAM) + LENGTH(RAM), "SRAM overflow: data, C heap and MSP stack overlap")
    ASSERT(ADDR(.isr_vector) == 0x08010000, "Application vector address changed")
    /* Prebuilt GCC 14 runtime libraries carry DWARF 5. Keep project DWARF 3
     * for uVision; omit only the runtime libraries' non-loadable debug data. */
    /DISCARD/ :
    {
        *libc_nano.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
        *libg_nano.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
        *libg.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
        *libc.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
        *libm.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
        *libgcc.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
        *libnosys.a:*(.debug* .zdebug* .stab* .gnu.debuglto_*)
    }
    .ARM.attributes 0 : { *(.ARM.attributes) }
}
