/**************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* PROJETO

/**
 * @Brief Defines
 * Aqui incluímos as bibliotecas (head files) necessárias
 * para o complementar o nosso arquivo principal main.c
 *
 * O arquivo asf.h é um head file que contém includes para outros arquivos,
 * iremos ver como isso funciona futuramente.
 */
#include "asf.h"
#include "conf_clock.h"

/**
 * @Brief LED
 * Declaramos alguns #defines que irã facilitar o desenvolvimento desse projeto.
 *	LED_PIO_ID	 - 12
 *	LED_PIO		   - Ponteiro para estrutura que contém os registradores do PIOC
 *  LED_PIN		   - PIO C 8, é o pino que o LED está conectado
 *  LED_PIN_MASK - Máscara para configurarmos o LED
 *
 *
 * LED_PIO é uma estrutura que contém os registradores (endereço de memória) do PIO em questão,
 * podemos acessar cada registrador desse periférico da seguinte maneira :
 *	PIOC->PIO_OER = (1 << 8);
 * nesse caso, o registrador Output Enable register do PIOC irá configurar o bit
 * 8 como 1, ou seja, irá configurar o pino PIOC8 como sendo saída.
 *
 */

/* LED DA PLACA*/
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN			8
#define LED_PIN_MASK	(1<<LED_PIN)

/*LEDs EXTERNO VERDE E VERMELHO*/
//LED Verde ( EXT1 PIO B pino 3 (PB3) (pino 5 da placa)
#define LEDVerde_PIO_ID		ID_PIOB
#define LEDVerde_PIO        PIOB
#define LEDVerde_PIN	    3 
#define LEDVerde_PIN_MASK	(1<<LEDVerde_PIN)

//LED Vermelho ( EXT2 PIO A pino 6 (PA6) (pino 5 da placa)
#define LEDVermelho_PIO_ID		ID_PIOA
#define LEDVermelho_PIO        PIOA
#define LEDVermelho_PIN	    6
#define LEDVermelho_PIN_MASK	(1<<LEDVermelho_PIN)

/*ENTRADAS DO MOTOR 1 e 2*/

//IN 1 do motor no PD11 ( EXT2 PIO D pino 11 (PD11) (pino 6 da placa) - cabo preto
#define IN1             ID_PIOD
#define IN1_PIO			PIOD
#define IN1_BIT			11
#define IN1_BIT_MASK 	(1<<IN1_BIT)

//IN 2 do motor no ( EXT2 PIO C pino 19 (PC19) (pino 7 da placa) - cabo vermelho
#define IN2  			ID_PIOC
#define IN2_PIO	    	PIOC
#define IN2_BIT			19
#define IN2_BIT_MASK 	(1<<IN2_BIT)

/* PARA BOTÃO DA PLACA*/
//Botão
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79




/* SENSOR MAGNÉTICO */
//Sensor de IMÃ  (EXT1 PIO B pino 2 (PB2) (pino 6 da placa) - cabo verde
#define IM_PIO_ID      ID_PIOB
#define IM_PIO         PIOB
#define IM_PIN		   2
#define IM_PIN_MASK    (1 << IM_PIN)
#define IM_DEBOUNCING_VALUE  79
/**
 * @Brief Inicializa o pino do LED
 */
void IN1Config(int estado){
	PMC->PMC_PCER0    = (1<<IN1);	    // Ativa clock do periférico no PMC
	IN1_PIO->PIO_PER  = IN1_BIT_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	IN1_PIO->PIO_OER  = IN1_BIT_MASK;           // Ativa saída                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicialização desejada
    IN1_PIO->PIO_CODR = IN1_BIT_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
  else
    IN1_PIO->PIO_SODR = IN1_BIT_MASK;       // Coloca 1 na saída                (SET Output Data register)
};
void IN2Config(int estado){
	PMC->PMC_PCER0    = (1<<IN2);	    // Ativa clock do periférico no PMC
	IN2_PIO->PIO_PER  = IN2_BIT_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	IN2_PIO->PIO_OER  = IN2_BIT_MASK;           // Ativa saída                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicialização desejada
    IN2_PIO->PIO_CODR = IN2_BIT_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
  else
    IN2_PIO->PIO_SODR = IN2_BIT_MASK;       // Coloca 1 na saída                (SET Output Data register)
};
/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(int estado){
	PMC->PMC_PCER0      = (1<<LED_PIO_ID);	    // Ativa clock do periférico no PMC
	LED_PIO->PIO_PER    = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER    = LED_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicialização desejada
    LED_PIO->PIO_CODR   = LED_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
  else
    LED_PIO->PIO_SODR   = LED_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};
void ledConfig1(int estado){
	PMC->PMC_PCER0      = (1<<LEDVerde_PIO_ID);	    // Ativa clock do periférico no PMC
	LEDVerde_PIO->PIO_PER   = LEDVerde_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LEDVerde_PIO->PIO_OER   = LEDVerde_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicialização desejada
	LEDVerde_PIO->PIO_CODR  = LEDVerde_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
	else
	LEDVerde_PIO->PIO_SODR  = LEDVerde_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};
void ledConfig2(int estado){
	PMC->PMC_PCER0      = (1<<LEDVermelho_PIO_ID);	    // Ativa clock do periférico no PMC
	LEDVermelho_PIO->PIO_PER   = LEDVermelho_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LEDVermelho_PIO->PIO_OER   = LEDVermelho_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicialização desejada
	LEDVermelho_PIO->PIO_CODR  = LEDVermelho_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
	else
	LEDVermelho_PIO->PIO_SODR  = LEDVermelho_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};
void butConfig(){
	// Configura botao
	PMC->PMC_PCER0        = (1<<BUT_PIO_ID);     // Ativa clock do periférico no PMC
	BUT_PIO->PIO_ODR	  = BUT_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	BUT_PIO->PIO_PER	  = BUT_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT_PIO->PIO_PUER	  = BUT_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT_PIO->PIO_IFER	  = BUT_PIN_MASK;        // Ativa debouncing
	BUT_PIO->PIO_IFSCER   = BUT_PIN_MASK;        // Ativa clock periferico
	BUT_PIO->PIO_SCDR	  = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
}
void imConfig(){
	// Configura imã
	PMC->PMC_PCER0       = (1<<IM_PIO_ID);     // Ativa clock do periférico no PMC
	IM_PIO->PIO_ODR	     = IM_PIN_MASK;        // Desativa saída                   (Output DISABLE register)
	IM_PIO->PIO_PER	     = IM_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	IM_PIO->PIO_PUER	 = IM_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	IM_PIO->PIO_IFER	 = IM_PIN_MASK;        // Ativa debouncing
	IM_PIO->PIO_IFSCER   = IM_PIN_MASK;        // Ativa clock periferico
	IM_PIO->PIO_SCDR	 = IM_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
}

void ligaMotor(){
/*PIO_SODR : coloca 1 nesse pino
PIO_CODR : coloca 0 nesse pino*/
	/* Valor 0 e 0 deixam o motor no ponto morto*/
	IN2_PIO->PIO_SODR = IN2_BIT_MASK;//lear do pino ligado na entrada 1 do motor (valor 1)
	IN1_PIO->PIO_CODR = IN1_BIT_MASK;//lear do pino ligado na entrada 2 do motor (valor 0)

}

void desligaMotor(){
/*PIO_SODR : coloca 1 nesse pino
PIO_CODR : coloca 0 nesse pino*/	
	/* Valor 0 e 1 deixam o motor girando no sentido horário*/
	IN1_PIO->PIO_CODR = IN1_BIT_MASK;//clear do pino ligado na entrada 1 do motor (valor 0)
	IN2_PIO->PIO_CODR = IN2_BIT_MASK;//clear do pino ligado na entrada 2 do motor (valor 0)

}

void desligaLEDverde(){
	LEDVerde_PIO->PIO_CODR = (LEDVerde_PIN_MASK);//1<<LEDVerde_PIN
}

void ligaLEDverde(){
	LEDVerde_PIO->PIO_SODR = (LEDVerde_PIN_MASK);//1<<LEDVerde_PIN
}

void desligaLEDvermelho(){
	LEDVermelho_PIO->PIO_CODR = (LEDVermelho_PIN_MASK);//1<<LEDVermelho_PIN
}

void ligaLEDvermelho(){
	LEDVermelho_PIO->PIO_SODR = (LEDVermelho_PIN_MASK);//1<<LEDVermelho_PIN
}

void piscaLEDvermelho(){
	pio_toggle_pin_group(LEDVermelho_PIO, LEDVermelho_PIN_MASK);
}
void ligaIma(){
	IM_PIO->PIO_SODR = IM_PIN_MASK;

}
void desligaIma(){
	IM_PIO->PIO_CODR = IM_PIN_MASK;
}

/**
* Periférico : PMC
* @Brief Peripheral Clock Enable Register
* O PMC é o periférico responsável pelo controle de energia dos
* demais periféricos.
* Inicializamos aqui o clock do periférico PIO C e D.
*/
void motorConfig(int estado){
//PMC->PMC_PCER0 = (1<<IN1_PIO_ID);

	PMC->PMC_PCER0 = (1<< IN1);//PMC ->PCER0 (PMC do pio D)
	PMC->PMC_PCER0 = (1<< IN2);//PMC ->PCER0 (PMC do pio C)
	IN1_PIO->PIO_OER = IN1_BIT_MASK;// Output Enable Register PIOD
	IN2_PIO->PIO_OER = IN2_BIT_MASK;// Output Enable Register PIOC
	IN1_PIO->PIO_PER = IN1_BIT_MASK;//  Enable Register PIOD
	IN2_PIO->PIO_PER = IN2_BIT_MASK;//  Enable Register PIOC
	IN1_PIO->PIO_SODR = IN1_BIT_MASK;
	IN2_PIO->PIO_CODR = IN2_BIT_MASK;// Clear Output Data Register PIOC
	if (estado == 0){
		desligaMotor();
	}
	else{
		ligaMotor();
	}
}
/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* 1. Inicialização do clock interno                                    */
	/************************************************************************/

	/**
	* Periférico : PMC
	* @Brief Inicializa clock do microcontrolador em 300Mhz
	* A configuração do clock dessa placa está no arquivo:
	* conf_clock.h
    *
    * O clock aqui está configurado apra 100Mhz !
	*/
	
    /**
	* Periférico : Watchdog
	* @Brief Desabilita o watchdog
	* Watchdog é uma função do microcontrolador responsável
	* por verificar possíveis travamentos.
	*/
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo saída
	//saídas
	ledConfig(1);
	ledConfig1(0);
	ledConfig2(0);
	motorConfig(0);
	//entradas
	butConfig();
	imConfig();




	/************************************************************************/
	
	/* Super loop                                                           */
	/************************************************************************/
	/**
	* @Brief Diferente de um código comum que executa em um pc, no uC deve estar
	* sempre executando um código, por isso utilizamos esse loop infingir.
	*/
	
	while(1){
		uint32_t botao_ligado = !pio_get(BUT_PIO, PIO_INPUT, BUT_PIN_MASK);
		uint32_t ima_ligado = !pio_get(IM_PIO, PIO_INPUT, IM_PIN_MASK);
		uint32_t botao_desligado = pio_get(BUT_PIO, PIO_INPUT, BUT_PIN_MASK);// se retorna 1 é desligado pois pull up esta ativado (botão esta sempre em 1 e vai para zero ao apertar o botão)
		uint32_t ima_desligado = pio_get(IM_PIO, PIO_INPUT, IM_PIN_MASK);

		if (botao_desligado && ima_desligado){
			desligaIma();
			desligaLEDverde();
			desligaLEDvermelho();
			desligaMotor();
			
		}
		else if (botao_desligado && ima_ligado){
			ligaIma();
			desligaLEDverde();
			ligaLEDvermelho();
			desligaMotor();
			
		}
		else if (botao_ligado && ima_desligado){
			desligaIma();
			desligaLEDverde();
			piscaLEDvermelho();
			
			desligaMotor();
			
		}
		else if (botao_ligado && ima_ligado){
			ligaLEDverde();
			ligaMotor();
			ligaIma();
			desligaLEDvermelho();
			
		}
		delay_ms(100);
	};
};
		/*
		
		//PARA BOTÃO DA PLACA E LED
		if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){ // Pull Down Status Register para Botão
				desligaMotor();
				desligaLEDverde();
				desligaLEDvermelho();
				
		}
			else{
				ligaMotor();
				ligaLEDverde();
				desligaLEDvermelho();
				
				
				
		}
		
		//PARA IMÃ
		if(IM_PIO->PIO_PDSR & (IM_PIN_MASK)){// Pull Down Status Register para Imã
			desligaIma();
			desligaMotor();
			desligaLEDverde();
			desligaLEDvermelho();
		}
		else{
			ligaIma();
			ligaMotor();
			ligaLEDverde();
			desligaLEDvermelho();
		}
		
	};

		};

*/
		
				/*
		 * @Brief Verifica constantemente o status do botão
		 * 1 : não apertado
		 * 0 : apertado
		 */
	/*if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){
		desligaLEDverde();
		desligaLEDvermelho();
		desligaMotor();
		if (IM_PIO->PIO_PDSR & (IM_PIN_MASK))
		{
			ligaLEDvermelho();
		}
		else{
			ligaMotor();
			ligaLEDverde();
		}
	}
	else{
		ligaLEDvermelho();
		//mdelay(500);
	}*/