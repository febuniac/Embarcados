/**************************************************************************
* Rafael Corsi   - Insper
* rafael.corsi@insper.edu.br
*
* Computação Embarcada
*
* 07-PIO-SAIDA
*
* Objetivo : Acionar um LED via controle do pino pelo PIO
*
* Explicativo :
*
* 1.
*    configura o clock interno do periférico
*
*  Cristal  |  _ _ _                        Aberto    _ _ _
*   |   |   | |     | --> Clk principal       /      |     |
*   | X | --> | PMC | --> Clk periféricos - -   ---->|PIOC |
*   |   |   | |_ _ _| --> ....                       |_ _ _|
*           |
*             uC
*
* 2.
*    desativa o watchdog para que o uc não seja reinicializado
*    sem necessitarmos.
*
* 3.
*    Ativamos o clock do periférico a ser utilizado
*
*  Cristal  |  _ _ _      Fechado  _ _ _
*   |   |   | |     | -->         |     |
*   | X | --> | PMC | ----------->|PIOC |
*   |   |   | |_ _ _| -->         |_ _ _|
*           |
*             uC
*
* 4.
*   Agora começamos a configurar no PIOC o pino responsável por controlar
*   o LED:
*
*     _ _ _
*    |     |out
* -->|PIOC |---------> LED
*    |_ _ _|PIOC6
*
*
*   Para isso é preciso saber que o PIOC possui diversos registradores
*   que configuram seu funcionamento interno, cada PIO pode tratar até
*   32 pinos diferentes, sendo que cada pino pode ter uma configuração
*   própria, esse controle é possível pois cada registrador é composto de 32 bits
*   e cada bit é responsável por um pino específico do PIO:
*
*   ---------------------------------------------------------
*   | bit 31 | bit 30 | ... ... ... | bit 2 | bit 1 | bit 0 |
*   ---------------------------------------------------------
*   | PIOC31 | PIOC30 |             | PIOC2 | PIOC1 | PIOC0 |
*   ---------------------------------------------------------
*
*   Exemplo: desejamos configurar o pino PIOC 8 como sendo saída.
*
*   Para isso escrevemos no registrador OER (output enable register) no
*   bit 8
*
*   PIOC->PIO_OER = (1 << 8);
*
*   O comando anterior pode ser lido como :
*   O registrador Output Enable do periférico PIOC ativa o bit 8.
*
*   # Enable/Disable/Status
*
*   Cada registrador de configuração desse microcontrolador possui
*   geralmente 3 registradores : Enable/Disable/Status
*
*    Exemplo para o Output Register:
*    - ENABLE  : Output Enable Register   : OER : (write only)
*    - Disable : Output Disable Register  : ODR : (write only)
*    - Status  : Output Status Register   : OSR : (read  only)
*
*   O valor efetivo do registrador (por exemplo do Output Enable) está
*   salve no OSR. Porém para alterarmos seu valor é necessário fazer o
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
*    Esse comando faz com que o bit 8 no OSR vá de 0 para 1 e automaticamente
*    o bit recém programando no OER vai para 0.
*
*    3. Mas como não podemos acessar o OSR, para zerarmos algum de seus bits
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
	
	sysclk_init();

  /************************************************************************/
	/* 2. Desativa o watchdog                                               */
	/************************************************************************/

    /**
	* Periférico : Watchdog
	* @Brief Desabilita o watchdog
	* Watchdog é uma função do microcontrolador responsável
	* por verificar possíveis travamentos.
	*/
	WDT->WDT_MR = WDT_MR_WDDIS;

	/************************************************************************/
	/* 3. Ativa clock no PIO Configuração do PMC	                          */
	/************************************************************************/

	/**
	* Periférico : PMC
	* @Brief Peripheral Clock Enable Register
	* O PMC é o periférico responsável pelo controle de energia dos
	* demais periféricos.
	* Inicializamos aqui o clock do periférico PIO C e D.
	*/
	PMC->PMC_PCER0 = (1<<LED_PIO_ID);
	PMC->PMC_PCER0 = (1<< OLED_LED_1);//PMC ->PCER0 (PMC do pio D)
	PMC->PMC_PCER0 = (1<< OLED_LED_2);//PMC ->PCER0 (PMC do pio C)
	PMC->PMC_PCER0 = (1<< OLED_LED_3);//PMC ->PCER0 (PMC do pio D)

	/************************************************************************/
	/* PIO				                                                          */
	/************************************************************************/

	/**
	* Periférico : PIO C
	* @Brief Output Enable Register
	* Configuramos o pino como saída
	*/
	// OER: When a bit in this register is at zero, the
	//corresponding I/O line is used as an input only. When the bit is at one, the corresponding I/O line is driven by the
	//PIO Controller (Pg 350 manual)
	PIOC->PIO_OER = (1 << 8);// Output Enable Register PIOC
	PIOD->PIO_OER = (1 << 11);// Output Enable Register PIOD
	PIOC->PIO_OER = (1 << 19);// Output Enable Register PIOC
	PIOD->PIO_OER = (1 << 26);// Output Enable Register PIOD

	/**
	* Periférico : PIO C
	* @Brief Peripheral Enable Register
	* Fazemos com que o controle do pino seja realizado pelo PIO
	* e não por outro periférico
	*/
	//When a pin is multiplexed with one or two peripheral functions, the selection is controlled with the Enable Register
	//(PIO_PER) and the Disable Register (PIO_PDR).). A value of one indicates the pin is controlled by the PIO
	//Controller(Pg 349 Manual) 
	PIOC->PIO_PER = (1 << 8);//LED da Placa
	PIOD->PIO_PER = (1 << 11);//  Enable Register PIOD
	PIOC->PIO_PER = (1 << 19);//  Enable Register PIOC
	PIOD->PIO_PER = (1 << 26);//  Enable Register PIOD

	/**
	* Periférico : PIO  
	* @Brief Clear/Set Output Data Register
	* Aqui controlamos o valor da saída desse pino
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
	* @Brief Diferente de um código comum que executa em um pc, no uC deve estar
	* sempre executando um código, por isso utilizamos esse loop infingir.
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
