/* hlab.h - register map of the HLab CPU subsystem (see cpu_subsystem.v,
 * gfx_ctrl.v, uart.v). All registers are 32 bits wide. */
#ifndef HLAB_H
#define HLAB_H

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

/* ---- graphics pipeline control (gfx_ctrl.v) ---- */
#define GFX_BASE          0x10000000u
#define GFX_CTRL          REG32(GFX_BASE + 0x00)
#define GFX_STATUS        REG32(GFX_BASE + 0x04)
#define GFX_FRAME_COUNT   REG32(GFX_BASE + 0x08)
#define GFX_FRAME_CYCLES  REG32(GFX_BASE + 0x10)
#define GFX_DRAW_CYCLES   REG32(GFX_BASE + 0x14)
#define GFX_MATRIX(i)     REG32(GFX_BASE + 0x40 + 4 * (i))  /* row-major, Q24.8 */

#define GFX_CTRL_GO       (1u << 0)
#define GFX_CTRL_SYNC     (1u << 1)   /* start in the next VGA window (tear-free) */
#define GFX_STATUS_BUSY   (1u << 0)

/* ---- UART 115200 8N1 (uart.v) ---- */
#define UART_BASE         0x20000000u
#define UART_DATA         REG32(UART_BASE + 0x00)
#define UART_STATUS       REG32(UART_BASE + 0x04)
#define UART_TX_BUSY      (1u << 0)
#define UART_RX_VALID     (1u << 1)

/* ---- GPIO ---- */
#define GPIO_BASE         0x30000000u
#define GPIO_LED          REG32(GPIO_BASE + 0x00)
#define GPIO_SW           REG32(GPIO_BASE + 0x04)
#define GPIO_BTN          REG32(GPIO_BASE + 0x08)
#define BTN_C             (1u << 0)
#define BTN_U             (1u << 1)
#define BTN_L             (1u << 2)
#define BTN_R             (1u << 3)
#define BTN_D             (1u << 4)

/* ---- cycle timer (100 MHz) ---- */
#define TIMER_LO          REG32(0x40000000u)
#define TIMER_HI          REG32(0x40000004u)
#define CLK_HZ            100000000u

#endif
