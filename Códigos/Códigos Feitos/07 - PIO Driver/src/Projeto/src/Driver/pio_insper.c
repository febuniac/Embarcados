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
	if (ul_pull_up_enable){
		p_pio->PIO_PUER = ul_mask; // Pull Up Enable Register
	}
	
	else{
		p_pio->PIO_PUDR = ul_mask; // Pull Up Disable Register
		
	}
}

/*Configure PIO pin internal pull-down.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask of one or more pin(s) to configure.
ul_pull_down_enable	Indicates if the pin(s) internal pull-down shall be configured.
*/
void _pio_pull_down(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_down_enable) {
	if (ul_pull_down_enable){
		p_pio->PIO_PPDER = ul_mask; // Pull Down Enable Register
	}
	else{
		p_pio->PIO_PPDDR = ul_mask; // // Pull Down Enable Register
	}
	
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
	p_pio->PIO_PER = ul_mask; //  PIO Enable Register (Ativa controle do pino no PIO)
	p_pio->PIO_OER = ul_mask;//	 Output Enable register (Ativa saída)
	
	if(!ul_pull_up_enable)// Checa se pullup nao esta enabled
	{
		p_pio->PIO_PUER	 = ul_mask;// Pull Up Enable Register
	}
	if(!ul_default_level)// Checa pela inicialização desejada
	{
		p_pio->PIO_CODR = ul_mask;//Clear Output Data Register (0 --> saída)
	}
	
	else
	{
	p_pio->PIO_SODR = ul_mask;//Set Output Data Register (1 --> saída)
	}
}
	


/*Configure one or more pin(s) or a PIO controller as inputs.
Optionally, the corresponding internal pull-up(s) and glitch filter(s) can be enabled.
Parameters
p_pio	Pointer to a PIO instance.
ul_mask	Bitmask indicating which pin(s) to configure as input(s).
ul_attribute	PIO attribute(s).*/

void pio_set_input	(Pio * 	p_pio,const uint32_t ul_mask,const uint32_t ul_attribute){
	p_pio->PIO_ODR	 = ul_mask;//Output Disable Register
	p_pio->PIO_PER	 = ul_mask;////Output Enable Register
	_pio_pull_up(p_pio,ul_mask,PIO_PULLUP & ul_attribute);
	if(PIO_DEBOUNCE & ul_attribute){
		p_pio->PIO_IFER	 = ul_mask; // Debouncing ativado
	}
}

/*Return 1 if one or more PIOs of the given Pin are configured to output a high level (even if they are not output).
To get the actual value of the pin, use PIO_Get() instead.*/

uint32_t pio_get_output_data_status	(const Pio * p_pio, const uint32_t 	ul_mask){
	if ((p_pio->PIO_ODSR & ul_mask) == 0){
		return 0;
		}
	else{
		return 1;
	}
}


//pull-up,pio_set_output, pio_set_input,_pio_get_output_data_status

