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
#define LED_PIO_ID	 ID_PIOC
#define LED_PIO		 PIOC
#define LED_PIN			 8
#define LED_PIN_MASK (1<<LED_PIN)

//Criando defines dos LEDs da placa OLED
#define OLED_LED_1  ID_PIOD
#define LED_PIO		PIOD
#define LED1_BIT		11
#define LED1_BIT_MASK (1<<LED_PIN)


#define OLED_LED_2  ID_PIOC
#define LED_PIO	    PIOC
#define LED2_BIT		19
#define LED2_BIT_MASK (1<<LED_PIN)


#define OLED_LED_3  ID_PIOD
#define LED_PIO	    PIOD
#define LED3_BIT		26
#define LED3_BIT_MASK (1<<LED_PIN)
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
	PMC->PMC_PCER0 = (1<<LED_PIO_ID);
	PMC->PMC_PCER0 = (1<< OLED_LED_1);//PMC ->PCER0 (PMC do pio D)
	PMC->PMC_PCER0 = (1<< OLED_LED_2);//PMC ->PCER0 (PMC do pio C)
	PMC->PMC_PCER0 = (1<< OLED_LED_3);//PMC ->PCER0 (PMC do pio D)

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
	PIOC->PIO_OER = (1 << 8);// Output Enable Register PIOC
	PIOD->PIO_OER = (1 << 11);// Output Enable Register PIOD
	PIOC->PIO_OER = (1 << 19);// Output Enable Register PIOC
	PIOD->PIO_OER = (1 << 26);// Output Enable Register PIOD

	/**
	* Perif�rico : PIO C
	* @Brief Peripheral Enable Register
	* Fazemos com que o controle do pino seja realizado pelo PIO
	* e n�o por outro perif�rico
	*/
	//When a pin is multiplexed with one or two peripheral functions, the selection is controlled with the Enable Register
	//(PIO_PER) and the Disable Register (PIO_PDR).). A value of one indicates the pin is controlled by the PIO
	//Controller(Pg 349 Manual) 
	PIOC->PIO_PER = (1 << 8);//LED da Placa
	PIOD->PIO_PER = (1 << 11);//  Enable Register PIOD
	PIOC->PIO_PER = (1 << 19);//  Enable Register PIOC
	PIOD->PIO_PER = (1 << 26);//  Enable Register PIOD

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
	PIOC->PIO_CODR = (1 << 8);//LED da Placa
	PIOD->PIO_CODR = (1 << 11);// Clear Output Data Register PIOD
	PIOC->PIO_CODR = (1 << 19);// Clear Output Data Register PIOC
	PIOD->PIO_CODR = (1 << 26);// Clear Output Data Register PIOD

	/************************************************************************/
	/* Super loop                                                           */
	/************************************************************************/

	/**
	* @Brief Diferente de um c�digo comum que executa em um pc, no uC deve estar
	* sempre executando um c�digo, por isso utilizamos esse loop infingir.
	*/
	while(1){
		delay_ms(100);
			PIOC->PIO_SODR = (1 << 8);// LED placa
			PIOD->PIO_SODR = (1 << 11);// LED 1 placa OLED
			PIOC->PIO_SODR = (1 << 14);// LED 2 placa OLED
			PIOD->PIO_SODR = (1 << 26);// LED 3 placa OLED
		delay_ms(100);
			PIOC->PIO_CODR = (1 << 8);
			PIOD->PIO_CODR = (1 << 11);
			PIOC->PIO_CODR = (1 << 14);
			PIOD->PIO_CODR = (1 << 26);

	};
}
