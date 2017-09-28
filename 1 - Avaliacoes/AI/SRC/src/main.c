/**
 *    Computacao Embarcada - Computacao - Insper
 *
 *            Avaliacao Intermediaria
 *
 * Faça um firmware que permita a um usuário no computador acessar e configurar algumas
 * informações/ modos  de operação do microcontrolador. Essas funcionalidades devem ser
 * acessadas via comunicação serial (COM). Um menu deve informar ao usuário as possibilidades
 * e os comandos que devem ser digitados para operar o embarcado.
 *
 * Funcionalidades que o firmware deve possuir :
 *
 * 1. Exibir menu
 * 1. O usuário deve ser capaz de ligar/desligar o piscar led (led da placa)
 * 1. O usuário deve ser capaz de aumentar(+2 Hz) e diminuir (-2 Hz) a frequência do led
 *
 * Utilize o programa disponível no repositório (github.com/insper/Computacao-Embarcada/Avaliacoes/A1/)
 * como ponto de parida. O código deve fazer uso de interrupções e periféricos para gerenciar a
 * comunicação com o PC e o LED.
 *
 *  ## Extra (A)
 *
 *  1. O usuário deve ser capaz de ler o relógio do microcontrolador.
 *  1. O usuário deve ser capaz de entrar com um valor de frequência para o 
  de forma numérica no termina.
 *
 */

/************************************************************************/
/* Includes                                                              */
/************************************************************************/

#include "asf.h"
#include <string.h>

/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/
/**
 *  USART
 */
#define USART_COM     USART1
#define USART_COM_ID  ID_USART1

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


/************************************************************************/
/* Variaveis globais                                                          */
/************************************************************************/

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

uint32_t usart_puts(uint8_t *pstring);
uint32_t usart_gets(uint8_t *pstring);

void LED_init(int estado);
void TC1_init(int freq_TC);
void RTC_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

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
void TC1_init(int freq_TC){   
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();
    
    uint32_t channel = 1;
    
    /* Configura o PMC */
    pmc_enable_periph_clk(ID_TC1);    
	
	//int freq_TC=4;//4Hz (4 vezes pos segundo led pisca)
    
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
/**
 * \brief Configure USART peripheral
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
 *  Toggle pin controlado pelo PIO (out)
 */
void pin_toggle(Pio *pio, uint32_t mask){
   if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
   else
    pio_set(pio,mask);
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
	char str[10];
	int freq_TC = 4;
	uint32_t rtc;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	
	/* Initialize the SAM system */
	sysclk_init();

	/* Disable the watchdog */
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	/** Inicializa USART */
	USART1_init();
  
	/* Configura Leds */
    LED_init(1);
    
	/** Configura timer 0 */
    TC1_init(4);//faz o led piscar em detrminada frequência
	
    /** Configura RTC */
    RTC_init();
	
	
  /** Super loop */
	while (1) {
		usart_gets(str);
		//usart_puts(str);
		
		if(str[0] == 'm'){
			usart_puts("Menu: m- aparece menu \n l-pisca led \n  o- desliga led \n a - aumenta frequência \n d - Diminui Frequência \n h - mostra a hora	  \n");
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
		
		else if (str[0] == 'a') {
			usart_puts("Aumenta Freq \n!");
			TC1_init(freq_TC+=2);
		}
		
		else if (str[0] == 'd') {
			usart_puts("Diminui Freq \n!");
			TC1_init(freq_TC-=2);
		}
		
		else if (str[0] == 'h') {
			//usart_puts("Horário \n!");
			rtc_get_time(RTC,&hour,&minute,&second);		
			sprintf(bufferTX, "%d : %d : %d \n",hour,minute,second);
			usart_puts(bufferTX);
		}	
	}
}
