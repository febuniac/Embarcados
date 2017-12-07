/**
 *
 * \file
 *
 * \brief WINC1500 Weather Client Example.
 *
 * Copyright (c) 2016 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/** \mainpage
 * \section intro Introduction
 * This example demonstrates the use of the WINC1500 with the SAM Xplained Pro
 * board to retrieve weather information from openweathermap.org server.<br>
 * It uses the following hardware:
 * - the SAM Xplained Pro.
 * - the WINC1500 on EXT1.
 *
 * \section files Main Files
 * - main.c : Initialize the WINC1500 and retrieve information.
 *
 * \section usage Usage
 * -# Build the program and download it into the board.
 * -# On the computer, open and configure a terminal application as the follows.
 * \code
 *    Baud Rate : 115200
 *    Data : 8bit
 *    Parity bit : none
 *    Stop bit : 1bit
 *    Flow control : none
 * \endcode
 * -# Start the application.
 * -# In the terminal window, the following text should appear:
 * \code
 *    -- WINC1500 weather client example --
 *    -- SAMXXX_XPLAINED_PRO --
 *    -- Compiled: xxx xx xxxx xx:xx:xx --
 *    Provision Mode started.
 *    Connect to [atmelconfig.com] via AP[WINC1500_xx:xx] and fill up the page.
 *    wifi_cb: M2M_WIFI_CONNECTED
 *    wifi_cb: IP address is xxx.xxx.xxx.xxx
 *    wifi_cb: M2M_WIFI_DISCONNECTED
 *    wifi_cb: M2M_WIFI_RESP_PROVISION_INFO
 *    wifi_cb: M2M_WIFI_CONNECTED
 *    wifi_cb: IP address is xxx.xxx.xxx.xxx
 *    resolve_cb: api.openweathermap.org IP address is xxx.xxx.xxx.xxx
 *    City: Paris
 *    Temperature: 24.50
 *    Weather Condition: sky is clear
 * \endcode
 *
 * \section compinfo Compilation Information
 * This software was written for the GNU GCC compiler using Atmel Studio 6.2
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com">Atmel</A>.\n
 */

#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"



#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --"STRING_EOL	\
	"-- "BOARD_NAME " --"STRING_EOL	\
	"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095UL)
volatile bool is_conversion_done = false;

/** The conversion data value */
volatile uint32_t g_ul_value = 0;
/* Canal do sensor de temperatura */
#define AFEC_CHANNEL_TEMP_SENSOR 11
/* HARDWARE PART*/

/* LED DA PLACA*/
#define LED_PIO_ID		ID_PIOC
#define LED_PIO         PIOC
#define LED_PIN			8
#define LED_PIN_MASK	(1<<LED_PIN)

/*LEDs EXTERNO VERDE, VERMELHO E AMARELO*/
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
#define LEDAmarelo_PIN			11
#define LEDAmarelo_PIN_MASK 	(1<<LEDAmarelo_PIN)

void TC1_init(int freq_TC);

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};
static uint8_t gau8PostBuffer[MAIN_WIFI_M2M_BUFFER_SIZE*2] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

uint8_t stop = 0;
//Variavéis que salvam minha temperaturas
uint8_t globalTemp1= 34;
uint8_t globalTemp2= 36;
uint8_t globalTemp3= 37;
uint8_t connectedON = false;

char valor [5];
uint8_t recveivedOk = false;
/**
 * \brief Configure UART console.
 */
static void configure_console(void)
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


/* 
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 */
 /* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap)
{
  int dots = 0;
  register u_long acc = 0, addr = 0;

  do {
	  register char cc = *cp;

	  switch (cc) {
	    case '0':
	    case '1':
	    case '2':
	    case '3':
	    case '4':
	    case '5':
	    case '6':
	    case '7':
	    case '8':
	    case '9':
	        acc = acc * 10 + (cc - '0');
	        break;

	    case '.':
	        if (++dots > 3) {
		    return 0;
	        }
	        /* Fall through */

	    case '\0':
	        if (acc > 255) {
		    return 0;
	        }
	        addr = addr << 8 | acc;
	        acc = 0;
	        break;

	    default:
	        return 0;
    }
  } while (*cp++) ;

  /* Normalize the address */
  if (dots < 3) {
	  addr <<= 8 * (3 - dots) ;
  }

  /* Store it if requested */
  if (ap) {
	  ap->s_addr = _htonl(addr);
  }

  return 1;    
}


/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
			(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	/* Check for socket event on TCP socket. */
	if (sock == tcp_client_socket) {
		switch (u8Msg) {
		case SOCKET_MSG_CONNECT:
		{
			printf("socket_msg_connect\n"); 
			if (gbTcpConnection) {
				tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
				if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR) {
					connectedON = true;
					//printf("%s",gau8ReceivedBuffer);
				} else {
					printf("socket_cb: connect error!\r\n");
					gbTcpConnection = false;
					close(tcp_client_socket);
					tcp_client_socket = -1;
					connectedON=false;
				}
			}
		}
		break;

		case SOCKET_MSG_RECV:
		{
			char *pcIndxPtr = NULL;
			char *pcEndPtr = NULL;
			char tempString[32];
			
			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
		
			if (pstrRecv && pstrRecv->s16BufferSize > 0) {			
				if(stop == 0){	
					recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
					stop = 1;
				}
				else{				
					printf("-------------- \n");
					pcIndxPtr = strstr(pstrRecv->pu8Buffer, "<p>temp:");
					printf(tempString);
					sprintf(tempString, "%c%c", *(pcIndxPtr+8), *(pcIndxPtr+9));
					globalTemp1  = atoi(tempString);
					printf("Temp 1 = %d\n", globalTemp1);
					sprintf(tempString, "%c%c", *(pcIndxPtr+11), *(pcIndxPtr+12));
					globalTemp2  = atoi(tempString);
					printf("Temp 2 = %d\n", globalTemp2);
					sprintf(tempString, "%c%c", *(pcIndxPtr+14), *(pcIndxPtr+15));
					globalTemp3  = atoi(tempString);
					printf("Temp 3 = %d\n", globalTemp3);
					stop = 0;
				}
				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));

				//if(recveivedOk == false){
					//recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
					//recveivedOk = true;
				//}
			} else {
				printf("socket_cb: recv error!\r\n");
				close(tcp_client_socket);
				tcp_client_socket = -1;
			}
		break;
		}

		default:
			break;
		}
	}
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5) {
		name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			gbConnectedWifi = false;
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		gbConnectedWifi = true;
		
    /* Obtain the IP Address by network name */
		//gethostbyname((uint8_t *)server_host_name);
		break;
	}

	default:
	{
		break;
	}
	}
}
/**
 * CONFIGURAÇÃO DOS LEDs
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
void TC1_Handler(void){
	volatile uint32_t ul_dummy;
	
 
	ul_dummy = tc_get_status(TC0, 1);

	UNUSED(ul_dummy);
	
	afec_start_software_conversion(AFEC0);
	//memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
	//sprintf((char *)gau8ReceivedBuffer, "%s", MAIN_PREFIX_BUFFER);
	//printf("\n");
	memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
	sprintf((char *)gau8ReceivedBuffer, "%s", MAIN_PREFIX_BUFFER);
	if(connectedON){
		printf("send : %d \n",gau8ReceivedBuffer );
		send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
		memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
		recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
		recveivedOk =false;
	}

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
/*FUNÇÕES DOS LEDs*/

void desligaLEDverde(){
	LEDVerde_PIO->PIO_CODR = (LEDVerde_PIN_MASK);
}

void ligaLEDverde(){
	LEDVerde_PIO->PIO_SODR = (LEDVerde_PIN_MASK);
}

void desligaLEDvermelho(){
	LEDVermelho_PIO->PIO_CODR = (LEDVermelho_PIN_MASK);
}

void ligaLEDvermelho(){
	LEDVermelho_PIO->PIO_SODR = (LEDVermelho_PIN_MASK);
}

void piscaLEDvermelho(){
	pio_toggle_pin_group(LEDVermelho_PIO, LEDVermelho_PIN_MASK);
}

void desligaLEDamarelo(){
	LEDAmarelo_PIO->PIO_CODR = (LEDAmarelo_PIN_MASK);
}
void ligaLEDamarelo(){
	LEDAmarelo_PIO->PIO_SODR = (LEDAmarelo_PIN_MASK);
}

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */
int main(void)
{
	tstrWifiInitParam param;
	int8_t ret;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	struct sockaddr_in addr_in;
	
	/* Initialize the board. */
	sysclk_init();
	board_init();
	
	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	/* Initialize the BSP. */
	nm_bsp_init();
	/*Inicializando os LEDs com seus valores iniciais*/
	ledConfig(0);
	ledConfig1(1);
	ledConfig2(1);
	ledConfig3(1);
	
	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret) {
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1) {
		}
	}

	/* Initialize socket API. */
	socketInit();
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to router. */
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);	

	/** Configura timer 1 */
	//TC1_init(2);
	
	/*AFEC CONFIG*/
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

	while (1) {

		m2m_wifi_handle_events(NULL);

		if (gbConnectedWifi && !gbTcpConnection) {
			//if (gbHostIpByName) {
				/* Open TCP client socket. */
				if (tcp_client_socket < 0) {
					if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
						printf("main: failed to create TCP client socket error!\r\n");
						continue;
					}
				}

        inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);
       
        //addr_in.sin_addr.s_addr =  0xc0a8008a;
        printf("inet_aton : 0x%X \n", addr_in.sin_addr.s_addr);
        
				/* Connect TCP client socket. */
				addr_in.sin_family = AF_INET;
				addr_in.sin_port = _htons(MAIN_SERVER_PORT);
				//addr_in.sin_addr.s_addr =
				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
					printf("main: failed to connect socket error!\r\n");
					continue;
				}

				gbTcpConnection = true;
			//}
		}
	}

	if(is_conversion_done == true) {
		is_conversion_done = false;
		int temperatura =(int)convert_adc_to_temp(g_ul_value);	
		printf("TEMPERATURA : %d  \r\n", temperatura );
		
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
	return 0;
	}
}
