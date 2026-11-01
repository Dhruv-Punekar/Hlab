/* start.S - reset entry for the HLab PicoRV32 firmware.
 * RAM is preloaded with the whole image (text + data) by $readmemh, so the
 * only startup work is the stack pointer and zeroing .bss. */
    .section .text.start
    .global _start
_start:
    la   sp, __stack_top
    la   t0, __bss_start
    la   t1, __bss_end
1:  bgeu t0, t1, 2f
    sw   zero, 0(t0)
    addi t0, t0, 4
    j    1b
2:  call main
3:  j    3b              /* main never returns */
