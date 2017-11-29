#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

/**
 *  Informacoes para o RTC
 *  poderia ser extraida do __DATE__ e __TIME__
 *  ou ser atualizado pelo PC.
 */
#define YEAR        2017
#define MOUNTH      9
#define DAY         20
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)


#define LED1_PIO_ID    ID_PIOA
#define LED1_PIO       PIOA
#define LED1_PIN       0
#define LED1_PIN_MASK  (1 << LED1_PIN)



#define LED2_PIO_ID    ID_PIOC
#define LED2_PIO       PIOC
#define LED2_PIN       30
#define LED2_PIN_MASK  (1 << LED2_PIN)



#define LED3_PIO_ID    ID_PIOB
#define LED3_PIO       PIOB
#define LED3_PIN       2
#define LED3_PIN_MASK  (1 << LED3_PIN)



/**
 * Botão
 */

#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79



#define BUT1_PIO_ID		ID_PIOD
#define BUT1_PIO		PIOD
#define BUT1_PIN		28
#define BUT1_PIN_MASK	(1 << BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE  79



#define BUT2_PIO_ID		ID_PIOC
#define BUT2_PIO		PIOC
#define BUT2_PIN		31
#define BUT2_PIN_MASK	(1 << BUT2_PIN)
#define BUT2_DEBOUNCING_VALUE  79



#define BUT3_PIO_ID		ID_PIOA
#define BUT3_PIO		PIOA
#define BUT3_PIN		19
#define BUT3_PIN_MASK	(1 << BUT3_PIN)
#define BUT3_DEBOUNCING_VALUE  79



/************************************************************************/

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile uint8_t flag_led0 = 1;//variável global que faz a  decisão se o LED está em modo “pisca pisca"


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void BUT_init(void);
void LED_init(int estado);
void TC1_init(void);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/


/**
 *  Handle Interrupcao botao 1
 */

void but1_Handler(){
	/*
	*  limpa interrupcao do PIO
	*/
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT1_PIO);
	/**
	*  Toggle status led
	*/
	if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK)){
		pio_clear(LED1_PIO, LED1_PIN_MASK);
		tc_start(TC0, 1);
	}
	else{
		pio_set(LED1_PIO,LED1_PIN_MASK);
		tc_stop(TC0, 1);
	}
}

/**
 *  Handle Interrupcao botao 2
 */

void but2_Handler(){
	/*
	*  limpa interrupcao do PIO
	*/

	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT2_PIO);

	/**
	*  Toggle status led
	*/

	if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK)){
		pio_clear(LED2_PIO, LED2_PIN_MASK);
		tc_start(TC0, 2);
	}
	else{
		pio_set(LED2_PIO,LED2_PIN_MASK);
		tc_stop(TC0, 2);
	}
}

/**
 *  Handle Interrupcao botao 1
 */

void but3_Handler(){
	/*
	*  limpa interrupcao do PIO
	*/

	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT3_PIO);
    
	/**
	*  Toggle status led
	*/

	if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK)){
		pio_clear(LED3_PIO, LED3_PIN_MASK);
		tc_start(TC1, 0);
	}

   
	else{
		pio_set(LED3_PIO,LED3_PIN_MASK);
		tc_stop(TC1, 0);
	}
}

/**
 *  Interrupt handler for TC0 interrupt. 
 */

void TC0_Handler(void){
	volatile uint32_t ul_dummy;
	/****************************************************************

	* Devemos indicar ao TC que a interrupção foi satisfeita.

	******************************************************************/

	ul_dummy = tc_get_status(TC0, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	/** Muda o estado do LED */
	pin_toggle(LED_PIO, LED_PIN_MASK);

}

/**

 *  Interrupt handler for TC1 interrupt. 

 */
/**
 *  Interrupt handler for TC1 interrupt. 
 */
void TC1_Handler(void){
	volatile uint32_t ul_dummy;
    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);
	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	/** Muda o estado do LED */
    if(flag_led0)
        pin_toggle(LED_PIO, LED_PIN_MASK);
		
}

/**

 *  Interrupt handler for TC2 interrupt. 

 */

void TC2_Handler(void){

	volatile uint32_t ul_dummy;

	/****************************************************************

	* Devemos indicar ao TC que a interrupção foi satisfeita.

	******************************************************************/
	ul_dummy = tc_get_status(TC0, 2);
	/* Avoid compiler warning */

	UNUSED(ul_dummy);
	/** Muda o estado do LED */

	pin_toggle(LED2_PIO, LED2_PIN_MASK);

}

/**

 *  Interrupt handler for TC4 interrupt. 

 */

void TC3_Handler(void){

	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC1, 0);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	/** Muda o estado do LED */
	pin_toggle(LED3_PIO, LED3_PIN_MASK);
}



/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */


/**
 * \brief Interrupt handler for the RTC. Refresh the display.
 */
void RTC_Handler(void)
{
	uint32_t ul_status = rtc_get_status(RTC);

	/* Second increment interrupt */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
	
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);

	} else {
		/* Time or date alarm */
		if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
      flag_led0 = 0;    
       
			rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
		}
	}
}


/************************************************************************/
/* Funcoes                                                              */
/************************************************************************/

/** 
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
}

/**
 * @Brief Inicializa o pino do BUT
 */
void BUT_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, Button1_Handler);
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};

/**
 * @Brief Inicializa o pino do LED
 */
void LED_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID);
    pio_set_output(LED_PIO, LED_PIN_MASK, estado, 0, 0 );
};

/**
 * Configura TimerCounter (TC0) para gerar uma interrupcao no canal 0-(ID_TC1) 
 * a cada 250 ms (4Hz)
 */
void TC1_init(void){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
    
    uint32_t channel = 1;
    
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC1);    
	
	int freq_TC=4;//4Hz (4 vezes pos segundo led pisca)
    
	/** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
    tc_find_mck_divisor(freq_TC, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, channel, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) / freq_TC);

    /* Configura e ativa interrupçcão no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
    tc_enable_interrupt(TC0, channel, TC_IER_CPCS);

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}

/**
 * Configura o RTC para funcionar com interrupcao de alarme
 */
void RTC_init(){
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_RTC);
        
    /* Default RTC configuration, 24-hour mode */
    rtc_set_hour_mode(RTC, 0);

    /* Configura data e hora manualmente */
    rtc_set_date(RTC, YEAR, MOUNTH, DAY, WEEK);
    rtc_set_time(RTC, HOUR, MINUTE, SECOND);

    /* Configure RTC interrupts */
	//NVIC chama os handlers
    NVIC_DisableIRQ(RTC_IRQn);
    NVIC_ClearPendingIRQ(RTC_IRQn);
    NVIC_SetPriority(RTC_IRQn, 0);
    NVIC_EnableIRQ(RTC_IRQn);
    
    /* Ativa interrupcao via alarme */
    rtc_enable_interrupt(RTC,  RTC_IER_ALREN); 
    
}

/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

  /* Configura Leds */
  LED_init(0);
	
	/* Configura os botões */
	BUT_init();    
    
  /** Configura timer 0 */
  TC1_init();//faz o led piscar em detrminada frequência
    
  /** Configura RTC */
  RTC_init();

  /* configura alarme do RTC */    
  rtc_set_date_alarm(RTC, 1, MOUNTH, 1, DAY);
  rtc_set_time_alarm(RTC, 1, HOUR, 1, MINUTE, 1, SECOND+5);
  
  	while (1) {
    
	}

}
