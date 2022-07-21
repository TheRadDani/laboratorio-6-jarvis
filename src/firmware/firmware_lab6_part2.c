#include <stdint.h>

#define LED_REGISTERS_MEMORY_ADD 0x10000010
#define IRQ_REGISTERS_MEMORY_ADD 0x10000011
#define LOOP_WAIT_LIMIT 1000 // time requiered to wait

uint32_t selector = 0;

static void putuint(uint32_t i) {
	*((volatile uint32_t *)LED_REGISTERS_MEMORY_ADD) = i;
}

static void putuint2(uint32_t i) {
	*((volatile uint32_t *)IRQ_REGISTERS_MEMORY_ADD) = i;
}


uint32_t *irq(uint32_t *regs, uint32_t irqs) {
    putuint2(0);
    return regs;
}


void main() {
	uint32_t number_to_display = 0;
	uint32_t counter = 0;

	while (1) {
		counter = 0;
		putuint(0);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
       putuint(1);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
       putuint(2);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
		putuint(3);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
      
       counter = 0;
		 putuint(4);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
		putuint(5);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
		putuint(6);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
       putuint(7);
		number_to_display++;
		while (counter < LOOP_WAIT_LIMIT) {
			counter++;
		}
       
       counter = 0;
       putuint(8);
        number_to_display++;
        while (counter < LOOP_WAIT_LIMIT) {
            counter++;
        }
	}
}
