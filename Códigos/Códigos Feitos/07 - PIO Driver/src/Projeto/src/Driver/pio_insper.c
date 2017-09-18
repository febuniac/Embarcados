#include "pio_insper.h"


void _pio_set(Pio *p_pio, const uint32_t ul_mask){
	p_pio->PIO_SODR = ul_mask;
}

void _pio_clear( Pio *p_pio,const uint32_t ul_mask){
	p_pio->PIO_CODR = ul_mask;
}
//pull-up,pio_set_output, pio_set_input,_pio_get_output_data_status