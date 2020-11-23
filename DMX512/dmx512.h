#ifndef __DMX512_H_
#define __DMX512_H_

#include "stm32f0xx_hal.h"

typedef struct{
	uint8_t recbuf[513];
	uint16_t bufcount;
	uint16_t RXB8;
	uint8_t fist_right;
	uint8_t tim_run;
	uint8_t time_count;
	uint8_t rx_end;
}__dmx512;

extern __dmx512 dmx512_model;
extern void dmx512_service(void);
#endif
