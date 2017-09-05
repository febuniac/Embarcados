/**************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computa��o Embarcada
*
* 07-PIO-SAIDA
*
* Objetivo : Acionar um LED via controle do pino pelo PIO
*
* Explicativo :
*
* 1.
*    configura o clock interno do perif�rico
*
*  Cristal  |  _ _ _                        Aberto    _ _ _
*   |   |   | |     | --> Clk principal       /      |     |
*   | X | --> | PMC | --> Clk perif�ricos - -   ---->|PIOC |
*   |   |   | |_ _ _| --> ....                       |_ _ _|
*           |
*             uC
*
* 2.
*    desativa o watchdog para que o uc n�o seja reinicializado
*    sem necessitarmos.
*
* 3.
*    Ativamos o clock do perif�rico a ser utilizado
*
*  Cristal  |  _ _ _      Fechado  _ _ _
*   |   |   | |     | -->         |     |
*   | X | --> | PMC | ----------->|PIOC |
*   |   |   | |_ _ _| -->         |_ _ _|
*           |
*             uC
*
* 4.
*   Agora come�amos a configurar no PIOC o pino respons�vel por controlar
*   o LED:
*
*     _ _ _
*    |     |out
* -->|PIOC |---------> LED
*    |_ _ _|PIOC6
*
*
*   Para isso � preciso saber que o PIOC possui diversos registradores
*   que configuram seu funcionamento interno, cada PIO pode tratar at�
*   32 pinos diferentes, sendo que cada pino pode ter uma configura��o
*   pr�pria, esse controle � poss�vel pois cada registrador � composto de 32 bits
*   e cada bit � respons�vel por um pino espec�fico do PIO:
*
*   ---------------------------------------------------------
*   | bit 31 | bit 30 | ... ... ... | bit 2 | bit 1 | bit 0 |
*   ---------------------------------------------------------
*   | PIOC31 | PIOC30 |             | PIOC2 | PIOC1 | PIOC0 |
*   ---------------------------------------------------------
*
*   Exemplo: desejamos configurar o pino PIOC 8 como sendo sa�da.
*
*   Para isso escrevemos no registrador OER (output enable register) no
*   bit 8
*
*   PIOC->PIO_OER = (1 << 8);
*
*   O comando anterior pode ser lido como :
*   O registrador Output Enable do perif�rico PIOC ativa o bit 8.
*
*   # Enable/Disable/Status
*
*   Cada registrador de configura��o desse microcontrolador possui
*   geralmente 3 registradores : Enable/Disable/Status
*
*    Exemplo para o Output Register:
*    - ENABLE  : Output Enable Register   : OER : (write only)
*    - Disable : Output Disable Register  : ODR : (write only)
*    - Status  : Output Status Register   : OSR : (read  only)
*
*   O valor efetivo do registrador (por exemplo do Output Enable) est�
*   salve no OSR. Por�m para alterarmos seu valor � necess�rio fazer o
*   acesso por um dos dois registradores: Enable/Disable, ja que o
*   OSR e read-only.
*
*   - Enable :
*     Torna 1 os bits no registrador de STATUS
*
*   - Disable :
*     Torna 0 os bits no registrador de STATUS
*
*   Exemplo, OSR inicializado em 0 e desejamos colocar em 1 o bit 8 do OSR, devemos :
*
*    1.
*    n[0] (instante 0)
*    ---------------------   ---------------------   ---------------------
*    |OER | 000000000000 |   |ODR | 000000000000 |   |OSR | 000000000000 |
*    ---------------------   ---------------------   ---------------------

*
*    2.
*    PIO->PIO_OER = (1 << 8) | (1 << 2);
*
*    n[1]
*    ---------------------   ---------------------   ---------------------
*    |OER | 000100000100 |   |ODR | 000000000000 |   |OSR | 000000000000 |
*    ---------------------   ---------------------   ---------------------
*
*    n[2]
*    ---------------------   ---------------------   ---------------------
*    |OER | 000000000000 |   |ODR | 000000000000 |   |OSR | 000100000100 |
*    ---------------------   ---------------------   ---------------------
*
*
*    Esse comando faz com que o bit 8 no OSR v� de 0 para 1 e automaticamente
*    o bit rec�m programando no OER vai para 0.
*
*    3. Mas como n�o podemos acessar o OSR, para zerarmos algum de seus bits
*    basta escrevermos no ODR (Disable)
*
*    PIO->PIO_ODR = (1<<2);
*
*    n[3]
*    ---------------------   ---------------------   ---------------------
*    |OER | 000000000000 |   |ODR | 000000000100 |   |OSR | 000000000000 |
*    ---------------------   ---------------------   ---------------------
*
*    n[4]
     ---------------------   ---------------------   ---------------------
*    |OER | 000000000000 |   |ODR | 000000000000 |   |OSR | 000100000000 |
*    ---------------------   ---------------------   ---------------------
*
************************************************************************/

/**
 * @Brief Defines
 * Aqui inclu�mos as bibliotecas (head files) necess�rias
 * para o complementar o nosso arquivo principal main.c
 *
 * O arquivo asf.h � um head file que cont�m includes para outros arquivos,
 * iremos ver como isso funciona futuramente.
 */
#include "asf.h"
#include "conf_clock.h"

/**
 * @Brief LED
 * Declaramos alguns #defines que ir� facilitar o desenvolvimento desse projeto.
 *	LED_PIO_ID	 - 12
 *	LED_PIO		   - Ponteiro para estrutura que cont�m os registradores do PIOC
 *  LED_PIN		   - PIO C 8, � o pino que o LED est� conectado
 *  LED_PIN_MASK - M�scara para configurarmos o LED
 *
 *
 * LED_PIO � uma estrutura que cont�m os registradores (endere�o de mem�ria) do PIO em quest�o,
 * podemos acessar cada registrador desse perif�rico da seguinte maneira :
 *	PIOC->PIO_OER = (1 << 8);
 * nesse caso, o registrador Output Enable register do PIOC ir� configurar o bit
 * 8 como 1, ou seja, ir� configurar o pino PIOC8 como sendo sa�da.
 *
 */

#define LED_PIO_ID		ID_PIOC
#define LED_PIO       PIOC
#define LED_PIN			  8
#define LED_PIN_MASK	(1<<LED_PIN)

//LED externo ( EXT1 PIO B pino 3 (pino 5 da placa)
#define LED1_PIO_ID		ID_PIOB
#define LED1_PIO       PIOB
#define LED1_PIN			  3 
#define LED1_PIN_MASK	(1<<LED1_PIN)

//LED externo ( EXT1 PIO A pino 6 (pino 5 da placa)
#define LED2_PIO_ID		ID_PIOA
#define LED2_PIO       PIOA
#define LED2_PIN			  6
#define LED2_PIN_MASK	(1<<LED2_PIN)

//IN 1 do motor no PD 11
#define IN1  ID_PIOD
#define IN1_PIO		PIOD
#define IN1_BIT		11
#define IN1_BIT_MASK (1<<IN1_BIT)

//IN 2 do motor no PC 19
#define IN2  ID_PIOC
#define IN2_PIO	    PIOC
#define IN2_BIT		19
#define IN2_BIT_MASK (1<<IN2_BIT)

//Bot�o
#define BUT_PIO_ID      ID_PIOA
#define BUT_PIO         PIOA
#define BUT_PIN		    11
#define BUT_PIN_MASK    (1 << BUT_PIN)
#define BUT_DEBOUNCING_VALUE  79

//IM� PB2
#define IM_PIO_ID      ID_PIOB
#define IM_PIO         PIOB
#define IM_PIN		    2
#define IM_PIN_MASK    (1 << IM_PIN)
#define IM_DEBOUNCING_VALUE  79
/**
 * @Brief Inicializa o pino do LED
 */
void IN1Config(int estado){
	PMC->PMC_PCER0    = (1<<IN1);	    // Ativa clock do perif�rico no PMC
	IN1_PIO->PIO_PER  = IN1_BIT_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	IN1_PIO->PIO_OER  = IN1_BIT_MASK;           // Ativa sa�da                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicializa��o desejada
    IN1_PIO->PIO_CODR = IN1_BIT_MASK;       // Coloca 0 na sa�da                (CLEAR Output Data register)
  else
    IN1_PIO->PIO_SODR = IN1_BIT_MASK;       // Coloca 1 na sa�da                (SET Output Data register)
};
void IN2Config(int estado){
	PMC->PMC_PCER0    = (1<<IN2);	    // Ativa clock do perif�rico no PMC
	IN2_PIO->PIO_PER  = IN2_BIT_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	IN2_PIO->PIO_OER  = IN2_BIT_MASK;           // Ativa sa�da                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicializa��o desejada
    IN2_PIO->PIO_CODR = IN2_BIT_MASK;       // Coloca 0 na sa�da                (CLEAR Output Data register)
  else
    IN2_PIO->PIO_SODR = IN2_BIT_MASK;       // Coloca 1 na sa�da                (SET Output Data register)
};
/**
 * @Brief Inicializa o pino do LED
 */
void ledConfig(int estado){
	PMC->PMC_PCER0    = (1<<LED_PIO_ID);	    // Ativa clock do perif�rico no PMC
	LED_PIO->PIO_PER  = LED_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED_PIO->PIO_OER  = LED_PIN_MASK;           // Ativa sa�da                      (Output ENABLE register)
  if(!estado)                                 // Checa pela inicializa��o desejada
    LED_PIO->PIO_CODR = LED_PIN_MASK;       // Coloca 0 na sa�da                (CLEAR Output Data register)
  else
    LED_PIO->PIO_SODR = LED_PIN_MASK;       // Coloca 1 na sa�da                (SET Output Data register)
};
void ledConfig1(int estado){
	PMC->PMC_PCER0    = (1<<LED1_PIO_ID);	    // Ativa clock do perif�rico no PMC
	LED1_PIO->PIO_PER  = LED1_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED1_PIO->PIO_OER  = LED1_PIN_MASK;           // Ativa sa�da                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicializa��o desejada
	LED1_PIO->PIO_CODR = LED1_PIN_MASK;       // Coloca 0 na sa�da                (CLEAR Output Data register)
	else
	LED1_PIO->PIO_SODR = LED1_PIN_MASK;       // Coloca 1 na sa�da                (SET Output Data register)
};
void ledConfig2(int estado){
	PMC->PMC_PCER0    = (1<<LED2_PIO_ID);	    // Ativa clock do perif�rico no PMC
	LED2_PIO->PIO_PER  = LED2_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LED2_PIO->PIO_OER  = LED2_PIN_MASK;           // Ativa sa�da                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicializa��o desejada
	LED2_PIO->PIO_CODR = LED2_PIN_MASK;       // Coloca 0 na sa�da                (CLEAR Output Data register)
	else
	LED2_PIO->PIO_SODR = LED2_PIN_MASK;       // Coloca 1 na sa�da                (SET Output Data register)
};


/************************************************************************/
/* Main                                                                 */
/************************************************************************/
int main(void)
{

	/************************************************************************/
	/* 1. Inicializa��o do clock interno                                    */
	/************************************************************************/

	/**
	* Perif�rico : PMC
	* @Brief Inicializa clock do microcontrolador em 300Mhz
	* A configura��o do clock dessa placa est� no arquivo:
	* conf_clock.h
    *
    * O clock aqui est� configurado apra 100Mhz !
	*/
	
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* Inicializa perifericos                                               */
	/************************************************************************/
	// Configura LED em modo sa�da
	ledConfig(1);
	ledConfig1(1);
	ledConfig2(1);

  /************************************************************************/
	/* 2. Desativa o watchdog                                               */
	/************************************************************************/

    /**
	* Perif�rico : Watchdog
	* @Brief Desabilita o watchdog
	* Watchdog � uma fun��o do microcontrolador respons�vel
	* por verificar poss�veis travamentos.
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* 3. Ativa clock no PIO Configura��o do PMC	                          */
	/************************************************************************/

	/**
	* Perif�rico : PMC
	* @Brief Peripheral Clock Enable Register
	* O PMC � o perif�rico respons�vel pelo controle de energia dos
	* demais perif�ricos.
	* Inicializamos aqui o clock do perif�rico PIO C e D.
	*/
	//PMC->PMC_PCER0 = (1<<IN1_PIO_ID);
	PMC->PMC_PCER0 = (1<< IN1);//PMC ->PCER0 (PMC do pio D)
	PMC->PMC_PCER0 = (1<< IN2);//PMC ->PCER0 (PMC do pio C)
	

	/************************************************************************/
	/* PIO				                                                          */
	/************************************************************************/

	/**
	* Perif�rico : PIO C
	* @Brief Output Enable Register
	* Configuramos o pino como sa�da
	*/
	// OER: When a bit in this register is at zero, the
	//corresponding I/O line is used as an input only. When the bit is at one, the corresponding I/O line is driven by the
	//PIO Controller (Pg 350 manual)
	
	IN1_PIO->PIO_OER = IN1_BIT_MASK;// Output Enable Register PIOD
	IN2_PIO->PIO_OER = IN2_BIT_MASK;// Output Enable Register PIOC


	/**
	* Perif�rico : PIO C
	* @Brief Peripheral Enable Register
	* Fazemos com que o controle do pino seja realizado pelo PIO
	* e n�o por outro perif�rico
	*/
	//When a pin is multiplexed with one or two peripheral functions, the selection is controlled with the Enable Register
	//(PIO_PER) and the Disable Register (PIO_PDR).). A value of one indicates the pin is controlled by the PIO
	//Controller(Pg 349 Manual) 
	
	IN1_PIO->PIO_PER = IN1_BIT_MASK;//  Enable Register PIOD
	IN2_PIO->PIO_PER = IN2_BIT_MASK;//  Enable Register PIOC
	

	/**
	* Perif�rico : PIO  
	* @Brief Clear/Set Output Data Register
	* Aqui controlamos o valor da sa�da desse pino
	* no caso especifico colocamos 0 (acende o LED)
	* O registrador :
	*   - PIO_SODR : coloca 1 nesse pino
	*	- PIO_CODR : coloca 0 nesse pino
	*/
	
	/*
	The level driven on an I/O line can be determined by writing in the Set Output Data Register (PIO_SODR) and the
	Clear Output Data Register (PIO_CODR). These write operations, respectively, set and clear the Output Data
	Status Register (PIO_ODSR), which represents the data driven on the I/O lines. Writing in PIO_OER and
	PIO_ODR manages PIO_OSR whether the pin is configured to be controlled by the PIO Controller or assigned to
	a peripheral function. This enables configuration of the I/O line prior to setting it to be managed by the PIO
	Controller.
	Similarly, writing in PIO_SODR and PIO_CODR affects PIO_ODSR. This is important as it defines the first level
	driven on the I/O line.(Pg 350 Manual)
	*/
	
	//PIOD->PIO_CODR = (1 << 11);// Clear Output Data Register PIOD
	IN1_PIO->PIO_SODR = IN1_BIT_MASK;
	IN2_PIO->PIO_CODR = IN2_BIT_MASK;// Clear Output Data Register PIOC**********************************************************************/


	// Configura botao
	PMC->PMC_PCER0      = (1<<BUT_PIO_ID);     // Ativa clock do perif�rico no PMC
	BUT_PIO->PIO_ODR	  = BUT_PIN_MASK;        // Desativa sa�da                   (Output DISABLE register)
	BUT_PIO->PIO_PER	  = BUT_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
	BUT_PIO->PIO_PUER	  = BUT_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
	BUT_PIO->PIO_IFER	  = BUT_PIN_MASK;        // Ativa debouncing
	BUT_PIO->PIO_IFSCER = BUT_PIN_MASK;        // Ativa clock periferico
	BUT_PIO->PIO_SCDR	  = BUT_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
	/************************************************************************/
	
	// Configura im�
		PMC->PMC_PCER0      = (1<<IM_PIO_ID);     // Ativa clock do perif�rico no PMC
		IM_PIO->PIO_ODR	  = IM_PIN_MASK;        // Desativa sa�da                   (Output DISABLE register)
		IM_PIO->PIO_PER	  = IM_PIN_MASK;        // Ativa controle do pino no PIO    (PIO ENABLE register)
		IM_PIO->PIO_PUER	  = IM_PIN_MASK;        // Ativa pull-up no PIO             (PullUp ENABLE register)
		IM_PIO->PIO_IFER	  = IM_PIN_MASK;        // Ativa debouncing
		IM_PIO->PIO_IFSCER = IM_PIN_MASK;        // Ativa clock periferico
		IM_PIO->PIO_SCDR	  = IM_DEBOUNCING_VALUE;// Configura a frequencia do debouncing
	/* Super loop                                                           */
	/************************************************************************/
	//while(1){};
	/**
	* @Brief Diferente de um c�digo comum que executa em um pc, no uC deve estar
	* sempre executando um c�digo, por isso utilizamos esse loop infingir.
	*/
	while(1){
		/*
		 * @Brief Verifica constantemente o status do bot�o
		 * 1 : n�o apertado
		 * 0 : apertado
		 */
		//PARA BOT�O DA PLACA E LED
		if(BUT_PIO->PIO_PDSR & (BUT_PIN_MASK)){//LIGA MOTOR E LED Gira horario
				IN1_PIO->PIO_CODR = IN1_BIT_MASK;//clear do pino ligado no led(por acaso � um led)
				IN2_PIO->PIO_CODR = IN2_BIT_MASK;//clear do pino ligado no led(por acaso � um led)
				PIOB->PIO_CODR = (1<<LED1_PIN);
				PIOA->PIO_SODR = (1<<LED2_PIN);
		}
			else{//DESLIGA MOTOR E LED
				IN2_PIO->PIO_SODR = IN2_BIT_MASK;//set do pino  que esta ligado no led(por acaso � um led)
				IN1_PIO->PIO_CODR = IN1_BIT_MASK;//clear do pino ligado no led(por acaso � um led)
				PIOB->PIO_SODR = (1<<LED1_PIN);
				PIOA->PIO_CODR = (1<<LED2_PIN);
		}
		/*
		//PARA IM�
		if(IM_PIO->PIO_PDSR & (IM_PIN_MASK)){//LIGA MOTOR E LED Gira antihorario
			IM_PIO->PIO_CODR = IM_PIN_MASK;
			IN1_PIO->PIO_CODR = IN1_BIT_MASK;//clear do pino ligado no led(por acaso � um led)
			IN2_PIO->PIO_SODR = IN2_BIT_MASK;//clear do pino ligado no led(por acaso � um led)
		}
		else{
			IM_PIO->PIO_SODR = IM_PIN_MASK ;
			IN2_PIO->PIO_CODR = IN2_BIT_MASK;//set do pino  que esta ligado no led(por acaso � um led)
			IN1_PIO->PIO_CODR = IN1_BIT_MASK;//clear do pino ligado no led(por acaso � um led)
		}
		*/
	};

		};

