/************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 10-PIO-INTERRUPCAO
*
* [ref] http://www.atmel.com/Images/Atmel-42142-SAM-AT03258-Using-Low-Power-Mode-in-SAM4E-Microcontroller_Application-Note.pdf
* [ref] https://www.eecs.umich.edu/courses/eecs373/labs/refs/M3%20Guide.pdf
************************************************************************/


#include "asf.h"
#include "conf_clock.h"
#include "Driver/pio_insper.h"
#include "Driver/pmc_insper.h"


/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs do SAME70
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN		    8
#define LED_PIN_MASK    (1<<LED_PIN)

//definindo os leds OLED1
//LED1
#define LED1_PIO_ID		ID_PIOA
#define LED1_PIO         PIOA
#define LED1_PIN		    0
#define LED1_PIN_MASK    (1<<LED1_PIN)
//LED2 
#define LED2_PIO_ID		ID_PIOC
#define LED2_PIO         PIOC
#define LED2_PIN		    30
#define LED2_PIN_MASK    (1<<LED2_PIN)
//LED3
#define LED3_PIO_ID		ID_PIOB
#define LED3_PIO         PIOB
#define LED3_PIN		    2
#define LED3_PIN_MASK    (1<<LED3_PIN)


/**
 * Botão do SAME70
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

//definindo os botões OLED1
//BOTÃO 1
#define BOTAO1_PIO_ID	ID_PIOD
#define BUT1_PIO		PIOD
#define BUT1_PIN		28
#define BUT1_PIN_MASK	(1<<BUT1_PIN)
//BOTÃO 2
#define BOTAO2_PIO_ID	ID_PIOC
#define BUT2_PIO		PIOC
#define BUT2_PIN		31
#define BUT2_PIN_MASK	(1<<BUT2_PIN)
//BOTÃO 3
#define BOTAO3_PIO_ID	ID_PIOA
#define BUT3_PIO		PIOA
#define BUT3_PIN		19
#define BUT3_PIN_MASK	(1<<BUT3_PIN)

/************************************************************************/
/* prototype   //existe essa função compila q vai achar                                                          */
/************************************************************************/
void led_init(int estado);
void led1_init(int estado);
void led2_init(int estado);
void but_init(void);
void but1_init(void);
void but2_init(void);
void but3_init(void);
void but_Handler();
void but1_Handler();
void but2_Handler();
void but3_Handler();

void piscaLED_placa(){
	pio_toggle_pin_group(LED_PIO,LED_PIN_MASK);
}
void piscaLED1(){
	pio_toggle_pin_group(LED1_PIO, LED1_PIN_MASK);
}

void piscaLED2(){
	pio_toggle_pin_group(LED2_PIO, LED2_PIN_MASK);
}
void piscaLED3(){
	pio_toggle_pin_group(LED3_PIO, LED3_PIN_MASK);
}
/************************************************************************/
/* Interrupçcões                                                        */
/************************************************************************/

void but_Handler(){
    /*
     *  limpa interrupcao do PIO
     */
    uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT_PIO);
    

   /**
    *  Toggle status led
    
   if(pio_get_output_data_status(LED_PIO, LED_PIN_MASK))
    pio_clear(LED_PIO, LED_PIN_MASK);
   else
    pio_set(LED_PIO,LED_PIN_MASK);
    */
   piscaLED_placa();
}

// Handlers para botões OLED1
void but1_Handler(){
	uint32_t pioIntStatus;
    pioIntStatus =  pio_get_interrupt_status(BUT_PIO);//aknowledge- para saber que a interrupção foi feita há uma leitura no registrador 
 
 /* Toggle na mão
   if(pio_get_output_data_status(LED1_PIO, LED1_PIN_MASK))
    pio_clear(LED1_PIO, LED1_PIN_MASK);
   else
    pio_set(LED1_PIO,LED1_PIN_MASK);
	*/
	piscaLED1();
}

void but2_Handler(){
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT_PIO);//aknowledge- para saber que a interrupção foi feita há uma leitura no registrador
	
	/*
	if(pio_get_output_data_status(LED2_PIO, LED2_PIN_MASK))
	pio_clear(LED2_PIO, LED2_PIN_MASK);
	else
	pio_set(LED2_PIO,LED2_PIN_MASK);	
	*/
	piscaLED2();
}

void but3_Handler(){
	uint32_t pioIntStatus;
	pioIntStatus =  pio_get_interrupt_status(BUT_PIO);//aknowledge- para saber que a interrupção foi feita há uma leitura no registrador
	
	/*
	if(pio_get_output_data_status(LED3_PIO, LED3_PIN_MASK))
	pio_clear(LED3_PIO, LED3_PIN_MASK);
	else
	pio_set(LED3_PIO,LED3_PIN_MASK);	
	*/
	piscaLED3();
}


/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void led_init(int estado){
    pmc_enable_periph_clk(LED_PIO_ID); //
    pio_set_output(LED_PIO, LED_PIN_MASK, 1, 0, 0 );//const uint32_t ul_default_levelconst
};

//Inicializando os LEDs do OLED1

void led1_init(int estado){
	pmc_enable_periph_clk(LED1_PIO_ID); //setando clock
	pio_set_output(LED1_PIO, LED1_PIN_MASK, 1, 0, 0 );//const uint32_t ul_default_levelconst (se a saída é em um ou 0)quando ligar vai apagar pois esta em 1
};
void led2_init(int estado){
	pmc_enable_periph_clk(LED2_PIO_ID); //setando clock
	pio_set_output(LED2_PIO, LED2_PIN_MASK, 1, 0, 0 );//const uint32_t ul_default_levelconst
};
void led3_init(int estado){
	pmc_enable_periph_clk(LED3_PIO_ID); //setando clock
	pio_set_output(LED3_PIO, LED3_PIN_MASK, 1, 0, 0 );//const uint32_t ul_default_levelconst
};
/**
 * @Brief Inicializa o pino do BUT
 *  config. botao em modo entrada enquanto 
 *  ativa e configura sua interrupcao.
 */
void but_init(void){
    /* config. pino botao em modo de entrada */
    pmc_enable_periph_clk(BUT_PIO_ID);
    pio_set_input(BUT_PIO, BUT_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
    
    /* config. interrupcao em borda de descida no botao do kit */
    /* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
    pio_enable_interrupt(BUT_PIO, BUT_PIN_MASK);//este pio neste pino (entradas) está com interrupção ativa
    pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIN_MASK, PIO_IT_FALL_EDGE, but_Handler);//este pio com este ID e este pino ativa a função but_Handler em borda de descida(quando apertar ele ativa- passa de 1 para 0)
    
    /* habilita interrupçcão do PIO que controla o botao */
    /* e configura sua prioridade                        */
    NVIC_EnableIRQ(BUT_PIO_ID);
    NVIC_SetPriority(BUT_PIO_ID, 1);
};


//Inicializando botões OLED1
//BOTAO1 inicializado
void but1_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BOTAO1_PIO_ID);
	pio_set_input(BUT1_PIO, BUT1_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT1_PIO, BUT1_PIN_MASK);
	pio_handler_set(BUT1_PIO, BOTAO1_PIO_ID, BUT1_PIN_MASK, PIO_IT_FALL_EDGE, but1_Handler); ;//aciona na borda de descida   PIO_IT_FALL_EDGE
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BOTAO1_PIO_ID);
	NVIC_SetPriority(BOTAO1_PIO_ID, 1);
};
//BOTAO2 inicializado
void but2_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BOTAO2_PIO_ID);
	pio_set_input(BUT2_PIO, BUT2_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT2_PIO, BUT2_PIN_MASK);
	pio_handler_set(BUT2_PIO, BOTAO2_PIO_ID, BUT2_PIN_MASK, PIO_IT_RISE_EDGE, but2_Handler);;//aciona na borda de subida &PIO_IT_RISE_EDGE
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BOTAO2_PIO_ID);
	NVIC_SetPriority(BOTAO2_PIO_ID, 1);
};
//BOTAO3 inicializado
void but3_init(void){
	/* config. pino botao em modo de entrada */
	pmc_enable_periph_clk(BOTAO3_PIO_ID);
	pio_set_input(BUT3_PIO, BUT3_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	
	/* config. interrupcao em borda de descida no botao do kit */
	/* indica funcao (but_Handler) a ser chamada quando houver uma interrupção */
	pio_enable_interrupt(BUT3_PIO, BUT3_PIN_MASK);
	pio_handler_set(BUT3_PIO, BOTAO3_PIO_ID, BUT3_PIN_MASK, PIO_IT_FALL_EDGE&&PIO_IT_RISE_EDGE, but3_Handler);//aciona na borda de subida e descida   PIO_IT_FALL_EDGE&&PIO_IT_RISE_EDGE
	
	/* habilita interrupçcão do PIO que controla o botao */
	/* e configura sua prioridade                        */
	NVIC_EnableIRQ(BOTAO3_PIO_ID);
	NVIC_SetPriority(BOTAO3_PIO_ID, 1);
};



/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{
	/************************************************************************/
	/* Inicialização básica do uC                                           */
	/************************************************************************/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializao I/OS                                                     */
	/************************************************************************/
	//Inicializa LEDs
	led_init(1);//placa
	led1_init(1);//OLED1
	led2_init(1);//OLED1
	led3_init(1);//OLED1
    //Inicializa Botões
	but_init();//placa
	but1_init();//OLED1
	but2_init();//OLED1
	but3_init();//OLED1

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
       /* entra em modo sleep */
       pmc_sleep(SLEEPMGR_SLEEP_WFI);//sleep no clock
	   
	   	for (int i=0; i<10;i++ ){//o led da placa pisca 10 vezes toda vez que um botão externo é clicado
		   	delay_ms(50);//atrasa 50 miliseconds
			piscaLED_placa();
	   	}
   	};
}


