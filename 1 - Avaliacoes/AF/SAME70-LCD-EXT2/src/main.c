#include "asf.h"
#include "image.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ioport.h"
#include "logo.h"

#define YEAR        2017
#define MOUNTH      9
#define DAY         20
#define WEEK        13
#define HOUR        9
#define MINUTE      5
#define SECOND      0

/** ASCII char definition for backspace. */
#define ASCII_BS    0x7F
/** ASCII char definition for carriage return. */
#define ASCII_CR    13
/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095UL)

/** The conversion data is done flag */
volatile bool is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;
volatile uint8_t flag_led0 = 0;//variável global que faz a  decisão se o LED está em modo “pisca pisca"

#define USART_COM     USART1
#define USART_COM_ID  ID_USART1
/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 11
#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- SAME70 LCD DEMO --"STRING_EOL	\
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

struct ili9488_opt_t g_ili9488_display_opt;
  uint8_t *pstring;
  uint32_t character =0;
  /* buffer para recebimento de dados */
  uint8_t bufferRX[100];
  
  /* buffer para transmissão de dados */
  uint8_t bufferTX[100];
  
    /* buffer para recebimento de dados */
    uint8_t bufferRX1[100];
    
    /* buffer para transmissão de dados */
    uint8_t bufferTX1[100];
/**
}
/**
 * \brief Configure UART console.
 */
static void configure_console(void)// configuraa UART e que o printf vai para a placa
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/** 
 * converte valor lido do ADC para temperatura em graus celsius
 * input : ADC reg value
 * output: Temperature in celsius
 */
static int32_t convert_adc_to_temp(int32_t ADC_value){//entra ADC e devolve a temperatura em Celsius
  
  int32_t ul_vol;
  int32_t ul_temp;
  
	ul_vol = ADC_value * VOLT_REF / MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  ul_temp = (ul_vol - 720)  * 100 / 233 + 27;
  return(ul_temp);
}

/**
 * \brief AFEC interrupt callback function.
 */
static void AFEC_Temp_callback(void) //Toda vez que o ADC tiver um valor novo essa função é chamada
{
	g_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);//salva o novo valor da converSão de temp
	is_conversion_done = true;
}
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
void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	
    /****************************************************************
	* Devemos indicar ao TC que a interrupção foi satisfeita.
    ******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);

	afec_start_software_conversion(AFEC0);
}

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
/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */
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
int main(void)
{
	uint32_t rtc;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
	/* Initialize the board. */
	sysclk_init();
	board_init();
	ioport_init();
	/** Configura timer 0 */
	TC1_init(1);
	/** Configura RTC */
	RTC_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);
	/* Set direction and pullup on the given button IOPORT */
	ioport_set_pin_dir(GPIO_PUSH_BUTTON_1, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(GPIO_PUSH_BUTTON_1, IOPORT_MODE_PULLUP);

	/* Initialize display parameter */
	g_ili9488_display_opt.ul_width = ILI9488_LCD_WIDTH;
	g_ili9488_display_opt.ul_height = ILI9488_LCD_HEIGHT;
	g_ili9488_display_opt.foreground_color = COLOR_CONVERT(COLOR_BLACK);//cor do quadrado
	g_ili9488_display_opt.background_color = COLOR_CONVERT(COLOR_BLACK);//cor do quadrado
	//formatar int usando sprintf
	/* Initialize LCD */
	ili9488_init(&g_ili9488_display_opt);
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, ILI9488_LCD_HEIGHT-1);
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLUEVIOLET));//cor do plano de fundo
	ili9488_draw_filled_rectangle(0, 0, ILI9488_LCD_WIDTH-1, 120-1);//retangulo de cima do texto
	ili9488_draw_filled_rectangle(0, 360, ILI9488_LCD_WIDTH-1, 480-1);//retangulo de baixo do texto
	//ili9488_draw_pixmap(100, 120, 100, 100, logoImage);//imagem(x,y da posicao x, y do tamanho)

	/* Escreve na tela */
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLUE));//bacground da caixa de escrita
	ili9488_draw_filled_rectangle(0, 300, ILI9488_LCD_WIDTH-1, 315);//retangulo para escrever
	ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));// cor da letra
	ili9488_draw_string(15, 150, (uint8_t *)"TEMPERATURE:");// o que está escrito e sua posicao
	ili9488_draw_string(80, 300, (uint8_t *)"CLOCK:");// o que está escrito e sua posicao
	ili9488_draw_string(15, 100, (uint8_t *)"BUNIAC PROVA FINAL");// o que está escrito e sua posicao
	
		
  /************************************* 
   * Ativa e configura AFEC (lendo temperatura da placa)
   *************************************/  

  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

  /* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

  /* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

  /* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);
  
  /* Configura trigger por software */
  afec_set_trigger(AFEC0, AFEC_TRIG_SW);
  
  /* configura call back */
 	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_11,	AFEC_Temp_callback, 1); 
   
  /*** Configuracao específica do canal AFEC ***/
  struct afec_ch_config afec_ch_cfg;
  afec_ch_get_config_defaults(&afec_ch_cfg);
  afec_ch_cfg.gain = AFEC_GAINVALUE_0;
  afec_ch_set_config(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, &afec_ch_cfg);
  
  /*
   * Calibracao:
	 * Because the internal ADC offset is 0x200, it should cancel it and shift
	 * down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_TEMP_SENSOR, 0x200);

  /***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(AFEC0, &afec_temp_sensor_cfg);

  /* Selecina canal e inicializa conversão */  
	afec_channel_enable(AFEC0, AFEC_CHANNEL_TEMP_SENSOR);
  

  afec_start_software_conversion(AFEC0);
	TC1_init(1);
	while (1) {
		if(is_conversion_done == true) {
			is_conversion_done = false;
			int temp = (int)convert_adc_to_temp(g_ul_value);
			rtc_get_time(RTC,&hour,&minute,&second);
			//printf("TEMPERATURA : %d  \r\n", (int)convert_adc_to_temp(g_ul_value) );
			//printf("Horário : %d  \r",hour,minute,second);
			sprintf(bufferTX1, " TEMPERATURA: %d \n",temp);
			printf("Horário :");
			sprintf(bufferTX, " %d : %d : %d \n",hour,minute,second);
			usart_puts(bufferTX);
			usart_puts(bufferTX1);
			unsigned int tensao = convert_adc_to_temp(g_ul_value);   // example value
			unsigned char buffer[7];      // buffer to put string

			//int pb= sprintf(buffer," %d", tensao);
			itoa(tensao, buffer, 10);   // make the string
			//lcd_puts(buffer);             // show the string
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLACK));
			ili9488_draw_filled_rectangle(140, 210,(uint8_t *)buffer, 250);
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_BLUE));//bacground da caixa de escrita
			ili9488_draw_filled_rectangle(180, 300, ILI9488_LCD_WIDTH-1, 315);//retangulo para escrever
			ili9488_set_foreground_color(COLOR_CONVERT(COLOR_WHITE));// cor da letra
			ili9488_draw_string(140, 210, (uint8_t *)buffer);// o que está escrito e sua posicao
			ili9488_draw_string(150, 300, (uint8_t *)bufferTX);
			if(temp <35){
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_GREEN));//bacground da caixa de escrita
				ili9488_draw_filled_circle(300,150,8);
			}
			else if(temp < 36){
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_YELLOW));//bacground da caixa de escrita
				ili9488_draw_filled_circle(300,150,8);

				
			}
			else{
				ili9488_set_foreground_color(COLOR_CONVERT(COLOR_RED));//bacground da caixa de escrita
				ili9488_draw_filled_circle(300,150,8);

			}
			
		}
	}
	return 0;
}
