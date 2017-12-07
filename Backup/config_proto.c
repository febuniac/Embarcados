/* LED DA PLACA*/
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN			8
#define LED_PIN_MASK	(1<<LED_PIN)

/*LEDs EXTERNO VERDE E VERMELHO*/
//LED Verde ( EXT2 PIO A pino 24 (PA24) (pino 10 da placa)
#define LEDVerde_PIO_ID		ID_PIOA
#define LEDVerde_PIO        PIOA
#define LEDVerde_PIN	    24 
#define LEDVerde_PIN_MASK	(1<<LEDVerde_PIN)

//LED Vermelho ( EXT2 PIO A pino 6 (PA6) (pino 5 da placa)
#define LEDVermelho_PIO_ID		ID_PIOA
#define LEDVermelho_PIO        PIOA
#define LEDVermelho_PIN	    6
#define LEDVermelho_PIN_MASK	(1<<LEDVermelho_PIN)

//LED Amarelo  no PD11 ( EXT2 PIO D pino 11 (PD11) (pino 6 da placa)
#define LEDAmarelo_PIO_ID      	ID_PIOD
#define LEDAmarelo_PIO			PIOD
#define LEDAmarelo_BIT			11
#define LEDAmarelo_BIT_MASK 	(1<<IN1_BIT)



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
void ledConfig3(int estado){
	PMC->PMC_PCER0      = (1<<LEDAmarelo_PIO_ID);	    // Ativa clock do periférico no PMC
	LEDAmarelo_PIO->PIO_PER   = LEDAmarelo_PIN_MASK;           // Ativa controle do pino no PIO    (PIO ENABLE register)
	LEDAmarelo_PIO->PIO_OER   = LEDAmarelo_PIN_MASK;           // Ativa saída                      (Output ENABLE register)
	if(!estado)                                 // Checa pela inicialização desejada
	LEDAmarelo_PIO->PIO_CODR  = LEDAmarelo_PIN_MASK;       // Coloca 0 na saída                (CLEAR Output Data register)
	else
	LEDAmarelo_PIO->PIO_SODR  = LEDAmarelo_PIN_MASK;       // Coloca 1 na saída                (SET Output Data register)
};


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

void desligaLEDamarelo(){
	LEDAmarelo_PIO->PIO_CODR = (LEDAmarelo_PIN_MASK);//1<<LEDVerde_PIN
}
void ligaLEDamarelo(){
	LEDAmarelo_PIO->PIO_SODR = (LEDAmarelo_PIN_MASK);//1<<LEDVerde_PIN
}


if (temperatura<=globalTemp1)
{
void ligaLEDverde();
void desligaLEDvermelho();
void desligaLEDamarelo();
	
}
else if(temperatura<globalTemp2)
{
void desligaLEDvermelho();
void desligaLEDverde();
void ligaLEDamarelo();
}
else if(temperatura==globalTemp3)
{
void ligaLEDvermelho();
void desligaLEDverde();
void desligaLEDamarelo();
}
else if(temperatura>globalTemp3)
{
void piscaLEDvermelho();
void desligaLEDverde();
void desligaLEDamarelo();
}

//na main
// ledConfig(1);
// ledConfig1(0);
// ledConfig2(0);
// ledConfig3(0);