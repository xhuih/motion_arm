#ifndef ZDT_H
#define ZDT_H

#include "usart.h"
#include "stdbool.h"

void position_control_zdt(uint8_t add,uint8_t dir,uint16_t vel,uint32_t pos,bool pos_flag,bool bot_flag);
void Emm_test(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);


#endif
