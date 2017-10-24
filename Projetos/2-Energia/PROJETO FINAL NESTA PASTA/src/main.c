#include "asf.h"

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/



/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN		    13
#define LED_PIN_MASK  (1<<LED_PIN)

/** 
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

/************************************************************************/
/* VAR globais                                                          */
/************************************************************************/
volatile bool g_ledBlinkOn = false;
uint8_t *pstring;
 uint32_t character =0;
 /* buffer para recebimento de dados */
 uint8_t bufferRX[100];
  
 /* buffer para transmissão de dados */
 uint8_t bufferTX[100];
 
 volatile uint8_t flag_led0 = 0;//variável global que faz a  decisão se o LED está em modo “pisca pisca"
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void LED_init(int estado);
void TC1_init(void);
//void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);
//USART
uint32_t usart_puts(uint8_t *pstring);
uint32_t usart_gets(uint8_t *pstring);

/************************************************************************/
/* Handlers                                                             */
/************************************************************************/

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

    /** Configura o TC para operar em  4Mhz e interrupçcão no RC compare */
    tc_find_mck_divisor(2, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);// pisca a cada meio segundo T =1/f f = 1/t t=1/2
    tc_init(TC0, channel, ul_tcclks | TC_CMR_WAVE);//mudando modo de operação
    tc_write_rc(TC0, channel, (ul_sysclk / ul_div) /93);//faz piscar mais rápido ou mais lento (divisor decide)
	tc_write_ra(TC0,channel,(ul_sysclk/ul_div)/94 );//mudanca de interrupcao do RA  (pg1386)
    /* Configura e ativa interrupçcão no TC canal 0 */
    NVIC_EnableIRQ((IRQn_Type) ID_TC1);
	tc_enable_interrupt(TC0, channel, (TC_IER_CPCS)|(TC_IER_CPAS));//concatenando as interrupcoes de RA e RC 

    /* Inicializa o canal 0 do TC */
    tc_start(TC0, channel);
}


/**
 * \brief Configure UART console.
 */
static void USART1_init(void){
  
  /* Configura USART1 Pinos */
 sysclk_enable_peripheral_clock(ID_PIOB);
 sysclk_enable_peripheral_clock(ID_PIOA);
 pio_set_peripheral(PIOB, PIO_PERIPH_D, PIO_PB4);  // RX
 pio_set_peripheral(PIOA, PIO_PERIPH_A, PIO_PA21); // TX
 MATRIX->CCFG_SYSIO |= CCFG_SYSIO_SYSIO4;
  
  /* Configura opcoes USART */
  const sam_usart_opt_t usart_settings = {
    .baudrate     = 115200,
    .char_length  = US_MR_CHRL_8_BIT,
    .parity_type  = US_MR_PAR_NO,
    .stop_bits    = US_MR_NBSTOP_1_BIT    ,
    .channel_mode = US_MR_CHMODE_NORMAL
  };
 /* Ativa Clock periferico USART0 */
  sysclk_enable_peripheral_clock(USART_COM_ID);

  /* Configura USART para operar em modo RS232 */
  usart_init_rs232(USART_COM, &usart_settings, sysclk_get_peripheral_hz());

  /* Enable the receiver and transmitter. */
  usart_enable_tx(USART_COM);
  usart_enable_rx(USART_COM);

 }
/**
 *  Envia para o UART uma string
 */
uint32_t usart_puts(uint8_t *pstring){
  uint32_t i = 0 ;

  while(*(pstring + i)){
    usart_serial_putchar(USART_COM, *(pstring+i++));
    while(!uart_is_tx_empty(USART_COM)){};
  }
  return(i);
}

/**
 * Busca do UART uma mensagem enviada pelo computador terminada em \n
 */
uint32_t usart_gets(uint8_t *pstring){
  uint32_t i = 0 ;
  usart_serial_getchar(USART_COM, (pstring+i));
  while(*(pstring+i) != '\n'){
    usart_serial_getchar(USART_COM, (pstring+(++i)));
  }
  *(pstring+i+1)= 0x00;
  return(i);

}
/************************************************************************/
/* Main Code	                                                        */
/************************************************************************/
int main(void){
	/* Initialize the SAM system */
	sysclk_init();
	char str[10];
	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;

  /* Configura Leds */
  LED_init(0);


  /** Configura timer 0 */
  TC1_init();
    
  
  /** Inicializa USART como printf */
  USART1_init();
  
	while (1) {
		 pmc_enable_sleepmode(0);//dorme enquanto o TC Handler não é chamado
	   usart_gets(str);
    
    if(str[0] == 'm'){
      usart_puts("Menu: \n m- aparece menu \n l-pisca led \n o- desliga led \n");
      usart_puts(bufferRX);   
    }
    
    else if (str[0] == 'l') {
      usart_puts("Olhe! O LED pisca!!! \n");
      flag_led0 = 1;
    }
    
    else if (str[0] == 'o') {
      usart_puts("Parou de Piscar \n!");
      flag_led0 = 0;
  }
	}
}