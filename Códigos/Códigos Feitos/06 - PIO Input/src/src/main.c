/**************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 08-PIO-ENTRADA
************************************************************************/


#include "asf.h"
#include "conf_clock.h"

/************************************************************************/
/* Defines                                                              */
/************************************************************************/

/**
 * LEDs
 */
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN			  8
#define LED_PIN_MASK	(1<<LED_PIN)

//Criando defines dos LEDs da placa OLED
#define OLED_LED_1  ID_PIOD
#define LED_PIO		PIOD
#define LED1_BIT		11
#define LED1_BIT_MASK (1<<LED1_BIT)


#define OLED_LED_2  ID_PIOC
#define LED_PIO	    PIOC
#define LED2_BIT		19
#define LED2_BIT_MASK (1<<LED2_BIT)


#define OLED_LED_3  ID_PIOD
#define LED_PIO	    PIOD
#define LED3_BIT		26
#define LED3_BIT_MASK (1<<LED3_BIT)

/**
 * Botão
 */
/**
 * Botão
 */
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

//Criando defines dos Botões da placa OLED
#define BUT_PIO_ID_1  ID_PIOD
#define BUT_PIO		PIOD
#define BUT1_BIT		11
#define BUT1_BIT_MASK (1<<BUT_PIN)


#define BUT_PIO_ID_2  ID_PIOD
#define BUT_PIO		PIOD
#define BUT2_BIT		19
#define BUT2_BIT_MASK (1<<BUT_BIT)


#define OLED_LED_3  ID_PIOD
#define LED_PIO	    PIOD
#define LED3_BIT		26
#define LED3_BIT_MASK (1<<LED_PIN)


/************************************************************************/
/* Prototipação                                                        */
/************************************************************************/
void ledConfig();

/************************************************************************/
/* Funções	                                                            */
/************************************************************************/

/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(int estado){
	PMC->PMC_PCER0    = (1<<LED_PIO_ID);	    // Ativa clock do periférico no PMC
	LED_PIO->PIO_PER  = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER  = LED_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicialização desejada
    LED_PIO->PIO_CODR = LED_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
  else
    LED_PIO->PIO_SODR = LED_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
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
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	ledConfig(0);//iniciar led no estado 0, desligado.

	// Configura botao
	PMC->PMC_PCER0        = (1<<BUT_PIO_ID);     // Ativa clock do periférico no PMC
	BUT_PIO->PIO_ODR	  = BUT_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT_PIO->PIO_PER	  = BUT_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT_PIO->PIO_PUER	  = BUT_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT_PIO->PIO_IFER	  = BUT_PIN_MASK;        // Ativa debouncing
	BUT_PIO->PIO_IFSCER	  = BUT_PIN_MASK;        // Ativa clock periferico
	BUT_PIO->PIO_SCDR	  = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		/**
     * @Brief Verifica constantemente o status do botão
     * 1 : não apertado
     * 0 : apertado
     */
    if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){
			LED_PIO->PIO_CODR = LED_PIN_MASK;//clear do pino ligado no led(por acaso é um led)
    }
		else{
			LED_PIO->PIO_SODR = LED_PIN_MASK;//set do pino  que esta ligado no led(por acaso é um led)
    }
	};
}
