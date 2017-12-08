#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"
/*POST*/
#define POST_SUFIX "HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: "
#define HTTP_END "\r\n"
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
//LED Verde ( EXT2 PIO C pino 13 (PA24) (pino 3 da placa)
#define LEDVerde_PIO_ID		ID_PIOC
#define LEDVerde_PIO        PIOC
#define LEDVerde_PIN	    13
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

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --"STRING_EOL	\
"-- "BOARD_NAME " --"STRING_EOL	\
"-- Compiled: "__DATE__ " "__TIME__ " --"STRING_EOL
void TC1_init(int freq_TC);
uint8_t stop = 0;
uint8_t recveivedOk = false;
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
/** Wi-Fi connection state */
static uint8_t wifi_connected;


/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

/*Variavéis que salvam minha temperaturas*/
uint8_t globalTemp1= 34;
uint8_t globalTemp2= 36;
uint8_t globalTemp3= 37;
uint8_t connectedON = false;

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
				wifi_connected = 0;
			}

			break;
		}

		case M2M_WIFI_REQ_DHCP_CONF:
		{
			uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
			printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
			pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
			wifi_connected = M2M_WIFI_CONNECTED;
			
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

void build_post(uint8_t *buff, char *route, char *query) {

	// Get content length
	static uint8_t content_length[20] = {0};
	sprintf(content_length, "%lu", strlen(query));

	sprintf(buff,"%s %s %s%s%s%s%s","POST",route,POST_SUFIX,content_length,HTTP_END,HTTP_END,query);

	printf("%s\n", buff);
}
/**
* \brief Main application function.
*
* Initialize system, UART console, network then start weather client.
*
* \return Program return value.
*/

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
	/** Configura timer 1 */
	TC1_init(1);
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
	/* Initialize socket module. */
	socketInit();
	
	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_SERVER_PORT);
	inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);
	printf("Inet aton : %d", addr_in.sin_addr);

	while(1){
		m2m_wifi_handle_events(NULL);
		
		build_post(gau8PostBuffer, "/temperatures", "temp=Shalom");
		if(is_conversion_done == true) {
			is_conversion_done = false;
			int temperatura =(int)convert_adc_to_temp(g_ul_value);
			printf("TEMPERATURA : %d  \r\n", temperatura );
			printf("T1 : %d  \r\n", globalTemp1);
			printf("T2 : %d  \r\n", globalTemp2);
			printf("T3 : %d  \r\n", globalTemp3);			
		
			
			/*
			if (temperatura<=globalTemp1)
			{
				ligaLEDverde();
				desligaLEDvermelho();
				desligaLEDamarelo();
				build_post(gau8PostBuffer, "/temp1", "temp=Esta tudo bem");
			}
			else if(temperatura<globalTemp2)
			{
				desligaLEDvermelho();
				desligaLEDverde();
				ligaLEDamarelo();
				build_post(gau8PostBuffer, "/temp1", "temp=Cara, fique atento!");
			}
			else if (temperatura ==globalTemp3){
				ligaLEDvermelho();
				desligaLEDverde();
				desligaLEDamarelo();
				build_post(gau8PostBuffer, "/temp1", "temp=TÁ QUENTE!!!!!");
			}
			else
			{	
				piscaLEDvermelho();
				desligaLEDverde();
				desligaLEDamarelo();
				build_post(gau8PostBuffer, "/temp1", "temp=SALVE-SE QUEM PUDER!!!!!");
			}
			
		}
		*/
		
		
		if (wifi_connected == M2M_WIFI_CONNECTED) {
			/* Open client socket. */
			if (tcp_client_socket < 0) {
				printf("socket init \n");
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
					printf("main: failed to create TCP client socket error!\r\n");
					continue;
				}

				/* Connect server */
				printf("socket connecting\n");
				
				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR) {
					close(tcp_client_socket);
					tcp_client_socket = -1;
					printf("error\n");
					}else{
					gbTcpConnection = true;
				}
				
			}
		}
	}
	
	return 0;
}




