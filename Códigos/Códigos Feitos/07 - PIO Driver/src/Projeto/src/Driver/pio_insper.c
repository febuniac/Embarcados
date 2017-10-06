#include "pio_insper.h"

/*Set a high output level on all the PIOs defined in ul_mask.
This has no immediate effects on PIOs that are not output, but the PIO controller will save the value if they are changed to outputs.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask of one or more pin(s) to configure.
*/
void _pio_set(Pio *p_pio, const uint32_t ul_mask){
	p_pio->PIO_SODR = ul_mask;
}

/*Set a low output level on all the PIOs defined in ul_mask.
This has no immediate effects on PIOs that are not output, but the PIO controller will save the value if they are changed to outputs.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask of one or more pin(s) to configure.
*/
void _pio_clear( Pio *p_pio,const uint32_t ul_mask){
	p_pio->PIO_CODR = ul_mask;
}

/*
Configure PIO internal pull-up.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask of one or more pin(s) to configure.
ul_pull_up_enable	Indicates if the pin(s) internal pull-up shall be configured. 
*/
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable) {
	if (ul_pull_up_enable)
	p_pio->PIO_PUER = ul_mask; // Pull Up Enable Register
	else
	p_pio->PIO_PUDR = ul_mask; // Pull Up Disable Register
}

/*Configure PIO pin internal pull-down.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask of one or more pin(s) to configure.
ul_pull_down_enable	Indicates if the pin(s) internal pull-down shall be configured.
*/
void _pio_pull_down(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_down_enable) {
	if (ul_pull_down_enable)
	p_pio->PIO_PPDER = ul_mask; // Pull Down Enable Register
	else
	p_pio->PIO_PPDDR = ul_mask; // // Pull Down Enable Register
}

/*Configure one or more pin(s) of a PIO controller as outputs, with the given default value.
Optionally, the multi-drive feature can be enabled on the pin(s).
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask indicating which pin(s) to configure.
ul_default_level	Default level on the pin(s).
ul_multidrive_enable	Indicates if the pin(s) shall be configured as open-drain.
ul_pull_up_enable	Indicates if the pin shall have its pull-up activated.*/
void pio_set_output	(Pio * 	p_pio,const uint32_t ul_mask,const uint32_t ul_default_level,const uint32_t ul_multidrive_enable,const uint32_t ul_pull_up_enable){
	
}

/*Configure one or more pin(s) or a PIO controller as inputs.
Optionally, the corresponding internal pull-up(s) and glitch filter(s) can be enabled.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask indicating which pin(s) to configure as input(s).
ul_attribute	PIO attribute(s).*/

void pio_set_input	(Pio * 	p_pio,const uint32_t ul_mask,const uint32_t ul_attribute){
	
}

/*Return 1 if one or more PIOs of the given Pin are configured to output a high level (even if they are not output).
To get the actual value of the pin, use PIO_Get() instead.*/

uint32_t pio_get_output_data_status	(const Pio * p_pio, const uint32_t 	ul_mask){
	if ((p_pio->PIO_ODSR & ul_mask) == 0)
		return 0;
	else
		return 1
}


//pull-up,pio_set_output, pio_set_input,_pio_get_output_data_status
http://asf.atmel.com/docs/latest/common.components.wifi.winc1500.ap_scan_example.sam4s_xplained_pro/html/group__sam__drivers__pio__group.html#ga462f21899dd66756a34de15beb15474d

