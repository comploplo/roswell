/* RP2040 memory map, matching hilt's `Platform::rp2040()` repl.
   hilt sets SP/PC/VTOR from the ELF vector table, so no boot2 is needed. */
MEMORY
{
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}
