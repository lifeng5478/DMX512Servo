#include "dmx512.h"

__dmx512 dmx512_model={{0},0,0,0,0,0,0};

void dmx512_service(void)
{
	if(dmx512_model.rx_end == 1)
	{
		TIM14->CCR1 = dmx512_model.recbuf[1]*100/256+25;	//20k pwm 100 起始0.5ms 结束2.5ms
		dmx512_model.bufcount = 0;
		dmx512_model.rx_end = 0;
	}
}
