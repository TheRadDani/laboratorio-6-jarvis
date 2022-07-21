#include <stdint.h>
#define LOOP_WAIT_LIMIT 2000000

#define LEDS_7SEG 0x10000000
#define Y_ADDR 0x20000000
#define Z_ADDR 0x30000000

char active = 0;

// allocate in memory
static void putuint(uint32_t i, int addr)
{
    *((volatile uint32_t *)addr) = i;
}
// Retrieve from memory
static uint32_t getuint(int addr)
{
    return *((volatile uint32_t *)addr);
}

// Delay
static void delay()
{
    uint32_t counter = 0;
    while (counter < LOOP_WAIT_LIMIT)
    {
        counter++;
    }
}

// Interrupt handling
uint32_t *irq(uint32_t *regs, uint32_t irqs)
{
    if ((irqs & 0x00000004) == 0x00000004)
        active = 1;
    else
        active = 0;
    return regs;
}

void main()
{
    uint32_t y_val = 0;
    uint32_t z_val = 0;
    while (1)
    {
        if (active)
        {
            // Get Y and Z axis accel values
            y_val = getuint(Y_ADDR);
            z_val = getuint(Z_ADDR);
            // Combine and write to 7 segment register
            putuint(((z_val << 16) | y_val), LEDS_7SEG);
            delay();
        }
        else
            putuint(0, LEDS_7SEG);
    }
}