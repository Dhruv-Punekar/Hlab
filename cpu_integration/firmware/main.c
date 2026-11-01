/* ============================================================================
 * main.c - HLab firmware: animates the cube on the hardware graphics pipeline
 *
 * Every frame the CPU
 *   1. reads buttons / switches / UART commands,
 *   2. advances the yaw and pitch angles,
 *   3. builds the 4x4 clip-space matrix  M = P * T(0,0,-D) * Rx(pitch) * Ry(yaw)
 *      in fixed point and writes it to the pipeline (signed Q24.8, row-major),
 *   4. starts one frame (synchronised to VGA: drawn while the image is off
 *      screen, so there is no tearing) and waits for it to finish.
 *
 * The camera constants reproduce the original matrix.mem exactly:
 *   60 degree vertical FOV, aspect 300/400, near 4, far 10, eye distance 6.
 * After reset the pipeline draws matrix.mem by itself (frame 0). The CPU then
 * draws the same matrix once more through its own path (frame 1, must be
 * bit-identical) and starts the animation from that pose.
 *
 * Board controls
 *   BTNL / BTNR  spin left / right faster (yaw speed -/+)
 *   BTNU / BTND  tumble speed +/- (pitch speed)
 *   BTNC         reset: original pose, default speeds
 *   SW0          pause the animation
 *   SW1          free-running (no VGA sync): shows the raw GPU frame rate,
 *                the image may tear
 *   SW15         show the original static frame (matrix.mem)
 *   LED[7:0]     frame counter / 4 (visible activity)
 *   LED[8]       paused, LED[9] free-running, LED[14] GPU busy last frame > 50%
 *   LED[15]      CPU trap (hardware)
 * UART (115200 8N1): h = help, p = pause, r = reset, s = stats,
 *                    a/d yaw speed -/+, w/x pitch speed +/-
 * ==========================================================================*/
#include <stdint.h>
#include "hlab.h"
#include "sin_table.h"

/* ---------------------------------------------------------------- UART --- */
static void uart_putc(char c)
{
    while (UART_STATUS & UART_TX_BUSY) ;
    UART_DATA = (uint8_t)c;
}
static void uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') uart_putc('\r');
        uart_putc(*s++);
    }
}
static void uart_putu(uint32_t v)
{
    char buf[11];
    int n = 0;
    do { buf[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n) uart_putc(buf[--n]);
}
static void uart_puti(int32_t v)
{
    if (v < 0) { uart_putc('-'); uart_putu((uint32_t)(-v)); }
    else       uart_putu((uint32_t)v);
}
static void uart_puthex(uint32_t v)
{
    for (int i = 7; i >= 0; i--)
        uart_putc("0123456789abcdef"[(v >> (4 * i)) & 15u]);
}
static int uart_getc(void)            /* -1 when nothing received */
{
    uint32_t d = UART_DATA;
    return (d & 0x100u) ? (int)(d & 0xFFu) : -1;
}

/* ------------------------------------------------------- fixed point --- */
/* angle unit: 1024 per turn; sin / cos return Q16.16 */
static int32_t isin(uint32_t a)
{
    a &= 1023u;
    uint32_t q = a >> 8, i = a & 255u;
    switch (q) {
    case 0:  return  SIN_Q16[i];
    case 1:  return  SIN_Q16[256 - i];
    case 2:  return -SIN_Q16[i];
    default: return -SIN_Q16[256 - i];
    }
}
static int32_t icos(uint32_t a) { return isin(a + 256u); }

/* (a * b) >> 16 with rounding, Q16 x Q16 -> Q16 */
static int32_t mul16(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * b + 32768) >> 16);
}
/* Q16 value -> Q24.8 with rounding */
static int32_t q16_to_q8(int32_t v) { return (v + 128) >> 8; }
/* (p * r) with p, r in Q16 -> Q24.8, rounded once */
static int32_t mul_to_q8(int32_t p, int32_t r)
{
    return (int32_t)(((int64_t)p * r + (1LL << 23)) >> 24);
}

/* camera / projection constants (Q16.16), see header */
#define P00   151349      /* f / aspect = 1.7320508 / 0.75      */
#define P11   113512      /* f = 1 / tan(30 deg)                 */
#define P22  (-152917)    /* (far + near) / (near - far) = -14/6 */
#define T2     43691      /* P22 * (-D) + 2*far*near/(near-far) = 0.6667 */
#define DIST  393216      /* eye distance D = 6.0                */

/* original matrix.mem, row-major (matrix.mem itself lists M33 first) */
static const int32_t ORIG_MATRIX[16] = {
    0x1e4,       0x0,         0x153,       0x0,
    0x6b,        0x192,       (int32_t)0xffffff66, 0x0,
    0x137,       (int32_t)0xffffff04, (int32_t)0xfffffe45, 0xab,
    0x85,        (int32_t)0xffffff94, (int32_t)0xffffff42, 0x600,
};

/* M = P * T(0,0,-D) * Rx(pitch) * Ry(yaw); angles in 1/1024 turn */
static void build_matrix(uint32_t yaw, uint32_t pitch, int32_t m[16])
{
    int32_t sy = isin(yaw),   cy = icos(yaw);
    int32_t sp = isin(pitch), cp = icos(pitch);

    /* rotation rows (Q16) */
    int32_t r0[3] = { cy, 0, sy };
    int32_t r1[3] = { mul16(sp, sy), cp, -mul16(sp, cy) };
    int32_t r2[3] = { -mul16(cp, sy), sp, mul16(cp, cy) };

    for (int c = 0; c < 3; c++) {
        m[0 + c]  = mul_to_q8(P00, r0[c]);
        m[4 + c]  = mul_to_q8(P11, r1[c]);
        m[8 + c]  = mul_to_q8(P22, r2[c]);
        m[12 + c] = q16_to_q8(-r2[c]);
    }
    m[3]  = 0;
    m[7]  = 0;
    m[11] = q16_to_q8(T2);
    m[15] = q16_to_q8(DIST);
}

/* -------------------------------------------------------------- frames --- */
static void gfx_write_matrix(const int32_t m[16])
{
    for (int i = 0; i < 16; i++) GFX_MATRIX(i) = (uint32_t)m[i];
}
static void gfx_draw(int sync)
{
    GFX_CTRL = GFX_CTRL_GO | (sync ? GFX_CTRL_SYNC : 0u);
    while (GFX_STATUS & GFX_STATUS_BUSY) ;
}

/* --------------------------------------------------------------- state --- */
/* angles and speeds in 1/1024 turn with 8 fraction bits */
#define YAW0   (100 << 8)     /* 35.2 deg  } nearest angle steps to the pose of */
#define PITCH0  (71 << 8)     /* 25.0 deg  } the original matrix (35, 25 deg)   */
#define YAW_SPEED0    384     /* 1.5 units / frame -> one turn in ~11 s at 60 fps */
#define PITCH_SPEED0  128     /* 0.5 units / frame */
#define SPEED_STEP     64
#define SPEED_MAX    2048

static uint32_t yaw_acc, pitch_acc;
static int32_t  yaw_speed, pitch_speed;
static int      paused_uart;

static void reset_motion(void)
{
    yaw_acc = YAW0; pitch_acc = PITCH0;
    yaw_speed = YAW_SPEED0; pitch_speed = PITCH_SPEED0;
}
static int32_t clamp_speed(int32_t v)
{
    return v > SPEED_MAX ? SPEED_MAX : v < -SPEED_MAX ? -SPEED_MAX : v;
}

static void print_help(void)
{
    uart_puts("keys: h help, p pause, r reset, s stats, a/d yaw -/+, w/x pitch +/-\n"
              "board: BTNL/BTNR yaw, BTNU/BTND pitch, BTNC reset, "
              "SW0 pause, SW1 no-sync, SW15 original frame\n");
}

static void print_matrix(const int32_t m[16])
{
    uart_puts("M=");
    for (int i = 0; i < 16; i++) { uart_puthex((uint32_t)m[i]); uart_putc(i == 15 ? '\n' : ' '); }
}

static void print_stats(uint32_t frame, uint32_t fps_x10)
{
    uint32_t draw = GFX_DRAW_CYCLES;
    uart_puts("frame "); uart_putu(frame);
    uart_puts("  draw "); uart_putu(draw / 100u); uart_puts(" us");
    uart_puts("  fps "); uart_putu(fps_x10 / 10u); uart_putc('.'); uart_putu(fps_x10 % 10u);
    uart_puts("  yaw "); uart_putu(((yaw_acc >> 8) & 1023u) * 360u / 1024u);
    uart_puts("  pitch "); uart_putu(((pitch_acc >> 8) & 1023u) * 360u / 1024u);
    uart_puts("  speed "); uart_puti(yaw_speed); uart_putc('/'); uart_puti(pitch_speed);
    uart_putc('\n');
}

int main(void)
{
    int32_t m[16];
    uint32_t btn_prev = 0, frame = 0;
    uint32_t t_mark = TIMER_LO, frames_mark = 0, fps_x10 = 0;

    GPIO_LED = 0;
    uart_puts("\nHLab: PicoRV32 + hardware 3D pipeline\n");
    print_help();

    reset_motion();

    /* frame 0 is the pipeline's own power-up render of matrix.mem */
    while (GFX_STATUS & GFX_STATUS_BUSY) ;
    uart_puts("power-up frame drawn in ");
    uart_putu(GFX_DRAW_CYCLES / 100u); uart_puts(" us\n");

    /* frame 1: the same matrix through the CPU path (must look identical) */
    gfx_write_matrix(ORIG_MATRIX);
    gfx_draw(1);

    build_matrix(yaw_acc >> 8, pitch_acc >> 8, m);
    print_matrix(m);

    for (;;) {
        uint32_t sw  = GPIO_SW;
        uint32_t btn = GPIO_BTN;
        uint32_t pressed = btn & ~btn_prev;     /* rising edges, sampled once per frame */
        btn_prev = btn;

        if (pressed & BTN_C) reset_motion();
        if (pressed & BTN_R) yaw_speed   = clamp_speed(yaw_speed + SPEED_STEP);
        if (pressed & BTN_L) yaw_speed   = clamp_speed(yaw_speed - SPEED_STEP);
        if (pressed & BTN_U) pitch_speed = clamp_speed(pitch_speed + SPEED_STEP);
        if (pressed & BTN_D) pitch_speed = clamp_speed(pitch_speed - SPEED_STEP);

        int c = uart_getc();
        switch (c) {
        case 'h': print_help(); break;
        case 'p': paused_uart = !paused_uart;
                  uart_puts(paused_uart ? "paused\n" : "running\n"); break;
        case 'r': reset_motion(); uart_puts("reset\n"); break;
        case 's': print_stats(frame, fps_x10); print_matrix(m); break;
        case 'd': yaw_speed   = clamp_speed(yaw_speed + SPEED_STEP);   break;
        case 'a': yaw_speed   = clamp_speed(yaw_speed - SPEED_STEP);   break;
        case 'w': pitch_speed = clamp_speed(pitch_speed + SPEED_STEP); break;
        case 'x': pitch_speed = clamp_speed(pitch_speed - SPEED_STEP); break;
        default:  break;
        }

        int paused = (sw & 1u) || paused_uart;
        int sync   = !(sw & 2u);

        if (!paused) {
            yaw_acc   += (uint32_t)yaw_speed;
            pitch_acc += (uint32_t)pitch_speed;
        }

        if (sw & 0x8000u) {
            gfx_write_matrix(ORIG_MATRIX);
        } else {
            build_matrix(yaw_acc >> 8, pitch_acc >> 8, m);
            gfx_write_matrix(m);
        }
        gfx_draw(sync);
        frame++;

        /* frame rate over the last second */
        uint32_t now = TIMER_LO;
        if (now - t_mark >= CLK_HZ) {
            fps_x10 = (frame - frames_mark) * 10u * (CLK_HZ / 1000u) / ((now - t_mark) / 1000u);
            t_mark = now; frames_mark = frame;
            print_stats(frame, fps_x10);
        }

        uint32_t busy_pct = GFX_DRAW_CYCLES / ((CLK_HZ / 100u) / 60u);   /* % of a 60 Hz frame */
        GPIO_LED = ((frame >> 2) & 0xFFu)
                 | (paused ? 1u << 8 : 0u)
                 | (sync ? 0u : 1u << 9)
                 | (busy_pct > 50u ? 1u << 14 : 0u);
    }
}
