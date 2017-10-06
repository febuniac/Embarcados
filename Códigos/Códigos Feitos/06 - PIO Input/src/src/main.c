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
#define BUT1_PIO_ID	ID_PIOD
#define BUT1_PIO		PIOD
#define BUT1_PIN		28
#define BUT1_PIN_MASK	(1<<BUT1_PIN)
#define BUT1_DEBOUNCING_VALUE  79

//BOTÃO 2
#define BUT2_PIO_ID	ID_PIOC
#define BUT2_PIO		PIOC
#define BUT2_PIN		31
#define BUT2_PIN_MASK	(1<<BUT2_PIN)
#define BUT2_DEBOUNCING_VALUE  79

//BOTÃO 3
#define BUT3_PIO_ID	ID_PIOA
#define BUT3_PIO		PIOA
#define BUT3_PIN		19
#define BUT3_PIN_MASK	(1<<BUT3_PIN)
#define BUT3_DEBOUNCING_VALUE  79





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

void led1Config(int estado){
	PMC->PMC_PCER0    = (1<<LED1_PIO_ID);	    // Ativa clock do periférico no PMC
	LED1_PIO->PIO_PER  = LED1_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED1_PIO->PIO_OER  = LED1_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicialização desejada
	LED1_PIO->PIO_CODR = LED1_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
	else
	LED1_PIO->PIO_SODR = LED1_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};

void led2Config(int estado){
	PMC->PMC_PCER0    = (1<<LED2_PIO_ID);	    // Ativa clock do periférico no PMC
	LED2_PIO->PIO_PER  = LED2_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED2_PIO->PIO_OER  = LED2_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicialização desejada
	LED2_PIO->PIO_CODR = LED2_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
	else
	LED2_PIO->PIO_SODR = LED2_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};

void led3Config(int estado){
	PMC->PMC_PCER0    = (1<<LED3_PIO_ID);	    // Ativa clock do periférico no PMC
	LED3_PIO->PIO_PER  = LED3_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED3_PIO->PIO_OER  = LED3_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicialização desejada
	LED3_PIO->PIO_CODR = LED3_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
	else
	LED3_PIO->PIO_SODR = LED3_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};

void butconfig (){
	// Configura botao
	PMC->PMC_PCER0        = (1<<BUT_PIO_ID);     // Ativa clock do periférico no PMC
	BUT_PIO->PIO_ODR	  = BUT_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT_PIO->PIO_PER	  = BUT_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT_PIO->PIO_PUER	  = BUT_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT_PIO->PIO_IFER	  = BUT_PIN_MASK;        // Ativa debouncing
	BUT_PIO->PIO_IFSCER	  = BUT_PIN_MASK;        // Ativa clock periferico
	BUT_PIO->PIO_SCDR	  = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
}
void but1config (void){
	// Configura botao1
	PMC->PMC_PCER0        = (1<<BUT1_PIO_ID);     // Ativa clock do periférico no PMC
	BUT1_PIO->PIO_ODR	  = BUT1_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT1_PIO->PIO_PER	  = BUT1_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT1_PIO->PIO_PUER	  = BUT1_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT1_PIO->PIO_IFER	  = BUT1_PIN_MASK;        // Ativa debouncing
	BUT1_PIO->PIO_IFSCER  = BUT1_PIN_MASK;        // Ativa clock periferico
	BUT1_PIO->PIO_SCDR	  = BUT1_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
}
void but2config (void){
	// Configura botao2
	PMC->PMC_PCER0        = (1<<BUT2_PIO_ID);     // Ativa clock do periférico no PMC
	BUT2_PIO->PIO_ODR	  = BUT2_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT2_PIO->PIO_PER	  = BUT2_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT2_PIO->PIO_PUER	  = BUT2_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT2_PIO->PIO_IFER	  = BUT2_PIN_MASK;        // Ativa debouncing
	BUT2_PIO->PIO_IFSCER  = BUT2_PIN_MASK;        // Ativa clock periferico
	BUT2_PIO->PIO_SCDR	  = BUT2_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
}
void but3config (void){
	// Configura botao3
	PMC->PMC_PCER0        = (1<<BUT3_PIO_ID);     // Ativa clock do periférico no PMC
	BUT3_PIO->PIO_ODR	  = BUT3_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT3_PIO->PIO_PER	  = BUT3_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT3_PIO->PIO_PUER	  = BUT3_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT3_PIO->PIO_IFER	  = BUT3_PIN_MASK;        // Ativa debouncing
	BUT3_PIO->PIO_IFSCER  = BUT3_PIN_MASK;        // Ativa clock periferico
	BUT3_PIO->PIO_SCDR	  = BUT3_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
}
	uint32_t atraso = 100;
	uint32_t atraso1 = 150;
	uint32_t atraso2 = 50;
	uint32_t atraso3 = 200;
void piscaLED_placa(){
		pio_set(LED_PIO, LED_PIN_MASK);
		delay_ms(atraso);
		pio_clear(LED_PIO, LED_PIN_MASK);
}
void piscaLED1(){
		pio_set(LED1_PIO, LED1_PIN_MASK);
		delay_ms(atraso1);
		pio_clear(LED1_PIO, LED1_PIN_MASK);
}

void piscaLED2(){
		pio_set(LED2_PIO, LED2_PIN_MASK);
		delay_ms(atraso2);
		pio_clear(LED2_PIO, LED2_PIN_MASK);
}
void piscaLED3(){
		pio_set(LED3_PIO, LED3_PIN_MASK);
		delay_ms(atraso3);
		pio_clear(LED3_PIO, LED3_PIN_MASK);

}


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
	ledConfig(1);//iniciar led no estado 0, desligado.
	led1Config(1);
	led2Config(1);
	led3Config(1);
   //Inicializa Botões
	butconfig();//placa
	but1config();//OLED1
	but2config();//OLED1
	but3config();//OLED1
	
	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/
	while(1){
		/**
		 * @Brief Verifica constantemente o status do botão
		 * 1 : não apertado
		 * 0 : apertado
		 */
		uint32_t botaoapertado =  !pio_get(BUT_PIO, PIO_INPUT, BUT_PIN_MASK);//BUT_PIO->PIO_PDSR & (BUT_PIN_MASK);
		uint32_t botao1apertado = !pio_get(BUT1_PIO, PIO_INPUT, BUT1_PIN_MASK);
		uint32_t botao2apertado = !pio_get(BUT2_PIO, PIO_INPUT, BUT2_PIN_MASK);
		uint32_t botao3apertado = !pio_get(BUT3_PIO, PIO_INPUT, BUT3_PIN_MASK);

    
		if(botaoapertado){
			piscaLED_placa();
				//LED_PIO->PIO_CODR = LED_PIN_MASK;//clear do pino ligado no led(por acaso é um led)
				}
		else if(botao1apertado){
			piscaLED1();
			}
		else if(botao2apertado){
			piscaLED2();
			}
		else if(botao3apertado){
			piscaLED3();
			}
	};
};
	
