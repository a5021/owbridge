/*
 * Linker script for STM32F030x6 (32 KB usable Flash, 4 KB RAM)
 *
 * The F030F4P6 is documented as 16 KB, but the die ships with 32 KB (the same
 * as the F6P6); the upper 16 KB is reachable and used here. The last 1 KB
 * page is reserved for the configuration block.
 *
 * Flash layout:
 *   0x08000000 .. 0x08007BFF  31 KB  firmware (.text/.rodata)
 *   0x08007C00 .. 0x08007FFF   1 KB  configuration page (.config, NOLOAD)
 *
 * The config page is reserved so the linker never places code there; the
 * firmware reads/writes it directly through the flash controller.
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x0;
_Min_Stack_Size = 0x400;

MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 31K
  CONFIG (rx)  : ORIGIN = 0x08007C00, LENGTH =  1K
  RAM    (xrw) : ORIGIN = 0x20000000, LENGTH =  4K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH

  .ARM.exidx :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT>FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
  } >RAM

  /* Configuration page: reserved, not loaded, not zeroed. */
  .config (NOLOAD) :
  {
    . = ALIGN(4);
    _config_start = .;
    KEEP(*(.config))
    . = ALIGN(4);
    _config_end = .;
  } >CONFIG

  __heap_start = ALIGN(8);
  __heap_end   = __heap_start + _Min_Heap_Size;

  __StackTop = _estack;
  __stack = _estack;

  PROVIDE(end = .);
  PROVIDE(_end = .);

  .ARM.attributes 0 : { *(.ARM.attributes) }

  /DISCARD/ :
  {
    *(.comment)
    *(.note.gnu.build-id)
  }
}

PROVIDE(_ram_start   = ORIGIN(RAM));
PROVIDE(_ram_end     = ORIGIN(RAM) + LENGTH(RAM));
PROVIDE(_flash_start = ORIGIN(FLASH));
PROVIDE(_flash_end   = ORIGIN(FLASH) + LENGTH(FLASH));
PROVIDE(_config_page = ORIGIN(CONFIG));
