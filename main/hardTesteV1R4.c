#include <stdio.h>																// Acesso as opercoes de entradas e/ou saidas.
#include <driver/adc.h>															// Modulo conversor ADC.
#include "freertos/FreeRTOS.h"													// Acesso aos termos.
#include "freertos/task.h"														// Acesso as prioridades da TASK.
#include "driver/gpio.h"														// Acesso ao uso das GPIOs.
#include "esp_log.h"															// Acesso ao uso dos LOGs.
#include <esp_http_server.h>    												// => httpd_uri_t
#include "rom/ets_sys.h"														// Acesso a escala de tempo em micro segundos.
#include <esp_event.h>          												// => esp_event_base_t
#include <nvs_flash.h>          												// => nvs_flash_init
#include <sys/param.h>          												// => MIN()
#include <esp_event.h>          												// => esp_event_base_t
#include "esp_tls_crypto.h"														// => esp_crypto_base64_encode
#include "esp_netif.h"                                                          // 
#include "esp_wifi.h"                                                           // 

/* RTOS */
#define CONFIG_FREERTOS_HZ 100													// Definicao da Espressif. Escala de tempo base (vTaskDelay).

/* KEYPAD */
#define ___keyCK	4							                                // Seleciona o pino de 'clock' para o registrador.
#define ___keyWR	16							                                // Seleciona o pino de 'data out' para o registrador.
#define ___keyLD	2							                                // Seleciona o pino de 'load' para o registrador.
#define ___keyRD	15							                                // Seleciona o pino de 'data in' para o registrador.

/* GERAL */
#define bitX(valor,bit) (valor&(1<<bit))				                        // Testa e retorna o 'bit' de 'valor'.
#define bit1(valor,bit) valor |= (1<<bit)				                        // Faz o 'bit' de 'valor' =1.
#define bit0(valor,bit) valor &= ~(1<<bit)				                        // Faz o 'bit' de 'valor' =0.

/* LCD */
#define ___lcdCK	17                       			                        // Seleciona o pino de 'clock' para o registrador.
#define ___lcdDT	18                       			                        // Seleciona o pino de 'data' para o registrador.
#define ___lcdLD	5                       			                        // Seleciona o pino de 'load' para o registrador.
#define ___RS		2                       			                        // Bit do registrador.
#define ___EN		3                       			                        // Bit do registrador.

/* Entrada */
#define ___gpiCK	12                       			                        // Seleciona o pino de 'clock' para o registrador.
#define ___gpiDT	13                       			                        // Seleciona o pino de 'data in' para o registrador.
#define ___gpiLD	14                       			                        // Seleciona o pino de 'load' para o registrador.
// char valorEntrada = 0, valorSaida = 0;										    // Var. global entrada e saida.

/* Saida */
#define ___gpoCK	12                       			                        // Seleciona o pino de 'clock' para o registrador.
#define ___gpoDT	27                       			                        // Seleciona o pino de 'data out' para o registrador.
#define ___gpoLD	14                       			                        // Seleciona o pino de 'load' para o registrador.

/* Expansao */
#define ___expCK	32                       			                        // Seleciona o pino de 'clock' para o registrador.
#define ___expLD	33                       			                        // Seleciona o pino de 'load' para o registrador.
#define ___expWR	25                       			                        // Seleciona o pino de 'data out' para o registrador.
#define ___expRD	26                       			                        // Seleciona o pino de 'data in' para o registrador.

/* Motor de Passo */
#define ___mpSLP    22                                                          // Seleciona o pino de 'Sleep'. Determina se motor ativo (1).
#define ___mpSTP    21                                                          // Seleciona o pino de 'Step'. Pulso para o passo do motor.
#define ___mpDIR    19                                                          // Seleciona o pino de 'Direction'. Determina o sentido do motor.

/* DHT 11/22 */
#define ___dhtPin	23															// Seleciona o pino de acesso ao DHT 11/22.

/* WiFi */
static const char *TAG = "Wifi_Fred";                                           // Identificacao da 'Task'.
int vlrQuery1, vlrQuery2, vlrQuery3;                                            // Var. para 'Query' do metodo 'GET'.
char vlrAscIn[]={"0000"};                                                       // Var. da entrada para retorno HTML.

// Configuracao de conexao
#define CONFIG_EXAMPLE_GPIO_RANGE_MIN				0
#define CONFIG_EXAMPLE_GPIO_RANGE_MAX				33
#define CONFIG_EXAMPLE_CONNECT_WIFI												// Sim
#define CONFIG_EXAMPLE_WIFI_SSID					"IoT_AP"
#define CONFIG_EXAMPLE_WIFI_PASSWORD				"12345678"
#define CONFIG_EXAMPLE_WIFI_SCAN_METHOD_ALL_CHANNEL	
#define CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL								// Sim
#define CONFIG_EXAMPLE_CONNECT_IPV6												// Sim
#define CONFIG_EXAMPLE_CONNECT_IPV6_PREF_LOCAL_LINK								// Sim
#define CONFIG_EXAMPLE_BASIC_AUTH												// Sim
#define CONFIG_EXAMPLE_BASIC_AUTH_USERNAME			"ESP32"						// Formulario de acesso: Ususario
#define CONFIG_EXAMPLE_BASIC_AUTH_PASSWORD			"ESPWebServer"				// Formulario de acesso: Senha
#define EXAMPLE_WIFI_SCAN_METHOD                    WIFI_ALL_CHANNEL_SCAN       // 
// #define EXAMPLE_WIFI_SCAN_METHOD                    WIFI_FAST_SCAN
#define EXAMPLE_DO_CONNECT                                                      // 
// Limites de busca
#define CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD		-127
// #define CONFIG_EXAMPLE_WIFI_AUTH_OPEN											// 
// #define CONFIG_EXAMPLE_WIFI_AUTH_WEP
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA_PSK
#define CONFIG_EXAMPLE_WIFI_AUTH_WPA2_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA_WPA2_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA2_ENTERPRISE
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA3_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WPA2_WPA3_PSK
// #define CONFIG_EXAMPLE_WIFI_AUTH_WAPI_PSK
#define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_LINK_LOCAL
// #define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_GLOBAL
// #define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_SITE_LOCAL
// #define EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE         ESP_IP6_ADDR_IS_UNIQUE_LOCAL
#define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD         WIFI_CONNECT_AP_BY_SIGNAL
// #define EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD         WIFI_CONNECT_AP_BY_SECURITY
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_OPEN
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WEP
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA_PSK
#define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA_WPA2_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_ENTERPRISE
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA3_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WPA2_WPA3_PSK
// #define EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD       WIFI_AUTH_WAPI_PSK

// Pelo SDKCONFIG
#define CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM       10
#define CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM      32
#define CONFIG_ESP32_WIFI_TX_BUFFER_TYPE             1
#define CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM      32
#define CONFIG_ESP32_WIFI_TX_BA_WIN                  6
#define CONFIG_ESP32_WIFI_RX_BA_WIN                  6            
#define CONFIG_ESP32_WIFI_SOFTAP_BEACON_MAX_LEN      752
#define CONFIG_ESP32_WIFI_MGMT_SBUF_NUM              32

#define MAX_HTTP_RECV_BUFFER 						512
#define MAX_HTTP_OUTPUT_BUFFER 						2048

/* Bloco Inicio: Teclado */
unsigned char tecTecNew,tecTecOld;				                                // Var. global para rotina anti-repeticao.

unsigned char __keyScan(void)					                                // Le as linhas.
{
	unsigned char entrada=0x00;				                                    // Var local temporaria para a entrada.
	unsigned char tst=0x00;				                                        // Var local temporaria para o laco.

	gpio_set_level(___keyLD,1);					                                // Ativa o pino da carga do dado.
	for(tst=0;tst<8;tst++)							                            // Laco para varrer os bits.
	{
		if(gpio_get_level(___keyRD)==1) bit1(entrada,(7-tst));		            // Se o pino de entrada estiver ativado...
		else			                bit0(entrada,(7-tst));		            // ... senao...
        gpio_set_level(___keyCK,1);					                            // Gera um pulso de clock no registrador.
        gpio_set_level(___keyCK,0);					                            //
	}
	gpio_set_level(___keyLD,0);						                            // Desativa o pino da carga do dado.
	// printf("Out: (0x%02X)\r\n",entrada);
	return (entrada);								                            // Retorna com o valor
}

void __keySerial(unsigned char __vlrK1)				                            // Envia as colunas para os registradores.
{
	unsigned char __tmp1;
	for(__tmp1=0;__tmp1<8;__tmp1++)					                            // Laco para serializar.
	{
        if(bitX(__vlrK1,(7-__tmp1)))gpio_set_level(___keyWR,1);		            // Verifica o valor do bit, se 1...
        else                        gpio_set_level(___keyWR,0);		            // ... e se for 0.
        gpio_set_level(___keyCK,1);					                            // Gera um pulso de clock no registrador.
        gpio_set_level(___keyCK,0);					                            //
	}
    gpio_set_level(___keyLD,1);						                            // Gera um pulso para carregar o dado.
    gpio_set_level(___keyLD,0);						                            //
}

const unsigned char ___teclaMatriz[4][4]=			                            // Definicao do valor das teclas.
{
	// {'1','2','3','A'},
	// {'4','5','6','B'},
	// {'7','8','9','C'},
	// {'F','0','E','D'}
    {'1','4','7','F'},
	{'2','5','8','0'},
	{'3','6','9','E'},
	{'A','B','C','D'}
};

void __keyTest(unsigned char keyTmp, unsigned char matPos)	                    // Verifica e recupera valor na matriz.
{
	if(keyTmp==1) tecTecNew=___teclaMatriz[0][matPos];	                        // Se for a 1ª linha...
	if(keyTmp==2) tecTecNew=___teclaMatriz[1][matPos];	                        // Se for a 2ª linha...
	if(keyTmp==4) tecTecNew=___teclaMatriz[2][matPos];	                        // Se for a 3ª linha...
	if(keyTmp==8) tecTecNew=___teclaMatriz[3][matPos];	                        // Se for a 4ª linha...
    // if(keyTmp==1) tecTecNew=___teclaMatriz[matPos][0];	                        // Se for a 1ª linha...
	// if(keyTmp==2) tecTecNew=___teclaMatriz[matPos][1];	                        // Se for a 2ª linha...
	// if(keyTmp==4) tecTecNew=___teclaMatriz[matPos][2];	                        // Se for a 3ª linha...
	// if(keyTmp==8) tecTecNew=___teclaMatriz[matPos][3];	                        // Se for a 4ª linha...
}

unsigned char tecla(void)							                            // Rotina de varredura e verificacao do teclado.
{
	unsigned char ___tmpKey=0x00;						                        // Var local temporaria com o valor da tecla.
	unsigned char ___tmpCol=0x01;						                        // Var local temporaria com o valor da coluna.
    // unsigned char ___tmpLin=0x01;						                    // Var local temporaria com o valor da linha.
	unsigned char ___tmpPos=0x00;						                        // Var local temporaria com o valor da posicao.

	for(___tmpPos=0;___tmpPos<4;___tmpPos++)			                        // Laco de varredura e verificacao.
	{
		__keySerial(___tmpCol);							                        // Seleciona a coluna.
        // __keySerial(___tmpLin);							                    // Seleciona a linha.
		vTaskDelay(1);									                        // Aguarda estabilizar.
		___tmpKey=__keyScan();							                        // Faz varredura do teclado.
		__keyTest(___tmpKey,___tmpPos);					                        // Verifica o valor se a tecla for acionada.
		___tmpCol=___tmpCol<<1;							                        // Proxima coluna.
        // ___tmpLin=___tmpLin<<1;							                    // Proxima linha.
	}
	__keySerial(0x00);									                        // Limpa o registrador de deslocamento.

	/* Codigo anti-repeticao */
	if(tecTecNew!=tecTecOld)	tecTecOld=tecTecNew;	                        // Se o valor atual eh diferente da anterior, salva atual e ... 
	else	tecTecNew=0x00;								                        // ...se nao... Salva como nao acionada ou nao liberada.						
	return (tecTecNew);									                        // ...retorna com o valor.
}

void tecladoIniciar(void)							                            // Rotina para iniciar o hardware.
{
	gpio_reset_pin(___keyCK);						// Limpa configuracoes anteriores.
	gpio_reset_pin(___keyWR);						// Limpa configuracoes anteriores.
	gpio_reset_pin(___keyLD);						// Limpa configuracoes anteriores.
	gpio_reset_pin(___keyRD);						// Limpa configuracoes anteriores.

    gpio_set_direction(___keyCK, GPIO_MODE_OUTPUT);	// Configura o pino como saida.
    gpio_set_direction(___keyWR, GPIO_MODE_OUTPUT);	// Configura o pino como saida.
    gpio_set_direction(___keyLD, GPIO_MODE_OUTPUT);	// Configura o pino como saida.
    gpio_set_direction(___keyRD, GPIO_MODE_INPUT);	// Configura o pino como entrada.

	gpio_set_level(___keyCK,0);						// Limpa o pino.
	gpio_set_level(___keyWR,0);						// Limpa o pino.
	gpio_set_level(___keyLD,0);						// Limpa o pino.
	gpio_set_level(___keyRD,0);						// Limpa o pino.
}
/* Bloco Fim: Teclado */

/* Bloco Inicio: LCD */
void __lcdCls(void)										                        // Limpa o registrador.
{
	unsigned char __tmp0;									                    // Var local temporaria.
	gpio_set_level(___lcdDT,0);								                    // Desliga o bit.						
	for(__tmp0=0;__tmp0<8;__tmp0++)                                             // Laco para zerar o registrador.
	{
		gpio_set_level(___lcdCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___lcdCK,0);                                             // 
	}
}

void __lcdSerial(unsigned char __vlrL1)					                        // Serializa o dado.
{
	unsigned char __tmp1;														// Var local temporaria.
	for(__tmp1=0;__tmp1<8;__tmp1++)												// Laco para serializar.
	{
		if(bitX(__vlrL1,(7-__tmp1)))gpio_set_level(___lcdDT,1);					// Verifica o valor do bit, se 1...
		else gpio_set_level(___lcdDT,0);										// ... e se for 0.				
		gpio_set_level(___lcdCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___lcdCK,0);                                             // 
	}							
	gpio_set_level(___lcdLD,1); 							                    // Gera um pulso para carregar o dado.
	gpio_set_level(___lcdLD,0);                                                 // 
}

void __lcd1Bit(unsigned char valor, unsigned char pinoRs)                       // Rotina de acesso ao LCD por registrador.
{
	unsigned char __tmp0;														// Var local temporaria.
	__lcdCls();																	// Limpa o registrador.
	__tmp0= valor & 0xF0;														// Separa a unidade.
	bit1(__tmp0,___EN);															// Acrescenta o bit '___EN'.
	if(pinoRs)	bit1(__tmp0,___RS);												// Se dado ___RS=1...
	else		bit0(__tmp0,___RS);												// ... senao ___RS=0-.
	__lcdSerial(__tmp0);														// Serializa o dado.
	vTaskDelay(pdMS_TO_TICKS(10));
	bit0(__tmp0,___EN);															// Remove o bit '___EN' (gerando a borda de descida).
	if(pinoRs)	bit1(__tmp0,___RS);												// Se dado ___RS=1
	else		bit0(__tmp0,___RS);												// Senao ___RS=0
	__lcdSerial(__tmp0);														// Serializa o dado.
	__tmp0=(valor & 0x0F)<<4;													// Separa a dezena e posiciona.
	bit1(__tmp0,___EN);															// Acrescenta o bit '___EN'.
	if(pinoRs)	bit1(__tmp0,___RS);												// Se dado ___RS=1
	else		bit0(__tmp0,___RS);												// Senao ___RS=0
	__lcdSerial(__tmp0);														// Serializa o dado.
	vTaskDelay(pdMS_TO_TICKS(10));
	bit0(__tmp0,___EN);															// Remove o bit '___EN' (gerando a borda de descida).
	if(pinoRs)	bit1(__tmp0,___RS);												// Se dado ___RS=1...
	else		bit0(__tmp0,___RS);												// ... senao ___RS=0.
	__lcdSerial(__tmp0);														// Serializa o dado.
}

void lcdIniciar(void)									                        // Inicializa o LCD.
{
	gpio_reset_pin(___lcdCK);													// Reinic
	gpio_set_direction(___lcdCK, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_reset_pin(___lcdDT);
	gpio_set_direction(___lcdDT, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_reset_pin(___lcdLD);
	gpio_set_direction(___lcdLD, GPIO_MODE_OUTPUT);								// Configura o pino como saida.

	__lcd1Bit(0x02,0);															// Habilita o uso em 4 bits.
	__lcd1Bit(0x28,0);															// Habilita  duas linhas, 5x7 e cursor simples.
//	__lcd1Bit(0x0E,0);															// Liga display e cursor.
	__lcd1Bit(0x0C,0);															// Liga display.
	__lcd1Bit(0x01,0);															// Limpa memoria do LCD e posiciona em HOME.
}

void lcdString(char *letras, unsigned char linha, unsigned char coluna)	        // Envia a string para o LCD.
{
	if(linha==1)__lcd1Bit((127+coluna),0);					                    // Se for a 1a. linha...
	if(linha==2)__lcd1Bit((191+coluna),0);					                    // Se for a 2a. linha...
	while(*letras)											                    // Enquanto houver caracteres validos...
	{
		__lcd1Bit(*letras,1);								                    // ...envia o caracter e...
		letras++;											                    // ...avanca para o proximo caracter.
	}
}
/* Bloco Fim: LCD */

/* Bloco Inicio: GPIOs */
void gpoIniciar(void)															// Inicializa o hardware da saida.
{
    gpio_reset_pin(___gpoCK);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___gpoDT);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___gpoLD);													// Limpa configuracoes anteriores.
    gpio_set_direction(___gpoCK, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
    gpio_set_direction(___gpoDT, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
    gpio_set_direction(___gpoLD, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_set_level(___gpoCK,0);													// Limpa o pino.
	gpio_set_level(___gpoDT,0);													// Limpa o pino.
	gpio_set_level(___gpoLD,0);													// Limpa o pino.

	/* Limpa o registrador */
	unsigned char __tmp010;									                    // Var local temporaria.					
	for(__tmp010=0;__tmp010<8;__tmp010++)                                       // Laco para zerar o registrador.
	{
		gpio_set_level(___gpoCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___gpoCK,0);                                             // 
	}
}

void gpiIniciar(void)															// Inicializa o hardware da entrada.
{
    gpio_reset_pin(___gpiCK);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___gpiDT);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___gpiLD);													// Limpa configuracoes anteriores.
    gpio_set_direction(___gpiCK, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
    gpio_set_direction(___gpiDT, GPIO_MODE_INPUT);								// Configura o pino como entrada.
    gpio_set_direction(___gpiLD, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_set_level(___gpiCK,0);													// Limpa o pino.
	gpio_set_level(___gpiDT,0);													// Limpa o pino.
	gpio_set_level(___gpiLD,0);													// Limpa o pino.

	/* Limpa o registrador */
	unsigned char __tmp011;									                    // Var local temporaria.					
	for(__tmp011=0;__tmp011<8;__tmp011++)                                       // Laco para zerar o registrador.
	{
		gpio_set_level(___gpiCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___gpiCK,0);                                             // 
	}
}

void gpoDado(char vlrSaida)														// Envia um dado para o GPO (saida).
{
	unsigned char __tmp001;														// Var local temporaria.
	for(__tmp001=0;__tmp001<8;__tmp001++)										// Laco para serializar.
	{
		if(bitX(vlrSaida,(7-__tmp001)))gpio_set_level(___gpoDT,1);				// Verifica o valor do bit, se 1...
		else gpio_set_level(___gpoDT,0);										// ... e se for 0.				
		gpio_set_level(___gpoCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___gpoCK,0);                                             // 
	}							
	gpio_set_level(___gpoLD,1); 							                    // Gera um pulso para carregar o dado.
	gpio_set_level(___gpoLD,0);                                                 // 
}

char gpiDado(void)																// Le um dado da GPI (entrada).
{
	unsigned char entrada=0x00;				                                    // Var local temporaria para a entrada.
	unsigned char tmp002=0x00;				                                    // Var local temporaria para o laco.

	gpio_set_level(___gpiLD,1);					                                // Ativa o pino da carga do dado.
	for(tmp002=0;tmp002<8;tmp002++)							                    // Laco para varrer os bits.
	{
		if(gpio_get_level(___gpiDT)==1) bit1(entrada,(7-tmp002));		        // Se o pino de entrada estiver ativado...
		else			                bit0(entrada,(7-tmp002));		        // ... senao...
        gpio_set_level(___gpiCK,1);					                            // Gera um pulso de clock no registrador.
        gpio_set_level(___gpiCK,0);					                            //
	}
	gpio_set_level(___gpiLD,0);						                            // Desativa o pino da carga do dado.
	return (entrada);								                            // Retorna com o valor
}
/* Bloco Fim: GPIOs */

void hex2Asc(char vlrHex, char *vlrAsc)											// Converte decimal em hexa(ASCII)
{
	char uni,dez;																// Variavel temporaria da unidade e dezena.
	uni = vlrHex & 0x0F;														// Separa a unidade.
	dez = (vlrHex &0xF0)>>4;													// Separa a dezena.

	if(uni>9) uni += 0x37;														// Se maior que 9, sera de 'A' ate 'F'.
	else 	  uni += 0x30;														// Senao  so converte em ASCII.
	if(dez>9) dez += 0x37;														// Se maior que 9, sera de 'A' ate 'F'.
	else 	  dez += 0x30;														// Senao  so converte em ASCII.
	vlrAsc[2] = dez;															// Salva valor para retorno.
	vlrAsc[3] = uni;															// Salva valor para retorno.
}

/* Bloco Inicio: Expansao */
void expIniciar(void)															// Inicializa o hardware da expansao.
{
	gpio_reset_pin(___expCK);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___expWR);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___expRD);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___expLD);													// Limpa configuracoes anteriores.
    gpio_set_direction(___expCK, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_set_direction(___expWR, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
    gpio_set_direction(___expRD, GPIO_MODE_INPUT);								// Configura o pino como entrada.
    gpio_set_direction(___expLD, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_set_level(___expCK,0);													// Limpa o pino.
	gpio_set_level(___expWR,0);													// Limpa o pino.
	gpio_set_level(___expRD,0);													// Limpa o pino.
	gpio_set_level(___expLD,0);													// Limpa o pino.

	/* Limpa o registrador */
	unsigned char __tmp012;									                    // Var local temporaria.					
	for(__tmp012=0;__tmp012<8;__tmp012++)                                       // Laco para zerar o registrador.
	{
		gpio_set_level(___expCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___expCK,0);                                             // 
	}
}

char expGPI(void)																// Le um dado da entrada da expansao.
{
	unsigned char expIn=0x00;				                                    // Var local temporaria para a entrada.
	unsigned char tmp004=0x00;				                                    // Var local temporaria para o laco.

	gpio_set_level(___expLD,1);					                                // Ativa o pino da carga do dado.
	for(tmp004=0;tmp004<8;tmp004++)							                    // Laco para varrer os bits.
	{
		if(gpio_get_level(___expRD)==1) bit1(expIn,(7-tmp004));		        	// Se o pino de entrada estiver ativado...
		else			                bit0(expIn,(7-tmp004));		        	// ... senao...
        gpio_set_level(___expCK,1);					                            // Gera um pulso de clock no registrador.
        gpio_set_level(___expCK,0);					                            //
	}
	gpio_set_level(___expLD,0);						                            // Desativa o pino da carga do dado.
	return (expIn);								                            	// Retorna com o valor
}

void expGPO(char vlrOut)														// Envia um dado para saida da expansao.
{
	unsigned char __tmp003;														// Var local temporaria.
	for(__tmp003=0;__tmp003<8;__tmp003++)										// Laco para serializar.
	{
		if(bitX(vlrOut,(7-__tmp003)))gpio_set_level(___expWR,1);				// Verifica o valor do bit, se 1...
		else gpio_set_level(___expWR,0);										// ... e se for 0.				
		gpio_set_level(___expCK,1);									            // Gera um pulso de clock no registrador.
		gpio_set_level(___expCK,0);                                             // 
	}							
	gpio_set_level(___expLD,1); 							                    // Gera um pulso para carregar o dado.
	gpio_set_level(___expLD,0);                                                 //
}
/* Bloco Fim: Expansao */

/* Bloco Inicio: Modulo Motor de Passo */
void mpIniciar(void)															// Inicializa o hardware do modulo DRV8825/A4988.
{
	gpio_reset_pin(___mpDIR);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___mpSLP);													// Limpa configuracoes anteriores.
	gpio_reset_pin(___mpSTP);													// Limpa configuracoes anteriores.
    gpio_set_direction(___mpDIR, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
    gpio_set_direction(___mpSLP, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
    gpio_set_direction(___mpSTP, GPIO_MODE_OUTPUT);								// Configura o pino como saida.
	gpio_set_level(___mpDIR,0);													// Limpa o pino.
	gpio_set_level(___mpSLP,0);													// Limpa o pino.
	gpio_set_level(___mpSTP,0);													// Limpa o pino.
}

void mpPulso(void)																// Gera um pulso no modulo DRV8825/A4988
{
	gpio_set_level(___mpSTP,1);													// Ativa o pino.
	gpio_set_level(___mpSTP,0);													// Desliga o pino.
}

void mpAngulo(unsigned int ang, unsigned char sent, unsigned char passo)		// Aciona o motor de passo com os argumentos fornecidos.
{
	unsigned int cntPulsos=0;													// Var. temp. para calculo.
	// 360o. /1,8o. = 200p
	// y = (200/360).x (...) y = 0,5556.x
	cntPulsos = ang * 0.5556 * passo;											// Calcula o numero de passos para o angulo e precisao escolhidos.
	gpio_set_level(___mpSLP,1);													// Sai do modo 'Sleep' (habilita as saidas).								
	gpio_set_level(___mpDIR,sent);												// Ajusta o sentido (Horario e antihorario).
	while(cntPulsos!=0)															// Enquanto tiver valor...
	{
		mpPulso();																// Gera um pulso para o modulo.
		cntPulsos--;															// Decrementa o valor...
		vTaskDelay(1); 									                        // Aguarda estabilizar a saida.
	}
	gpio_set_level(___mpSLP,0);													// Entra no modo 'Sleep' (desliga as saidas).
}
/* Bloco Fim: Modulo Motor de Passo */

void int2Asc(unsigned int valor, char *buffer, char digi) 						// Converte INT em ASCII: Valor(Bin), Matriz, Numero de digitos (0 a 5). 
{
	if(digi>5) digi=5;															// Previne erros.
	switch(digi)																// Seleciona o numero de algarismos.
	{
			case 0: 															// Nao converte o Valor(Bin).
				break;															// Retorno.
			case 1:																// Um algarismo.
				buffer[0]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
			case 2:																// Dois algarismos.
				buffer[0]=(valor/10)       +0x30; 								// Separa a dezena.
				buffer[1]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
			case 3:																// Tres algarismos.
				buffer[0]=(valor/100)      +0x30; 								// Separa a centena.
				buffer[1]=((valor/10)%10)  +0x30; 								// Separa a dezena.
				buffer[2]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.	
			case 4:																// Quatro algarismos.
				buffer[0]=(valor/1000)     +0x30; 								// Separa a unidade de milhar.
				buffer[1]=((valor/100)%10) +0x30; 								// Separa a centena.
				buffer[2]=((valor/10)%10)  +0x30; 								// Separa a dezena.
				buffer[3]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
			case 5:																// Cinco algarismos.
				buffer[0]=(valor/10000)    +0x30; 								// Separa a dezena de milhar.
				buffer[1]=((valor/1000)%10)+0x30; 								// Separa a unidade de milhar
				buffer[2]=((valor/100)%10) +0x30; 								// Separa a centena.
				buffer[3]=((valor/10)%10)  +0x30; 								// Separa a dezena.
				buffer[4]=(valor%10)       +0x30; 								// Separa a unidade.
				break;															// Retorno.
	}
}

void adcIniciar(void)															// Inicializa o hardware do modulo conversor ADC.
{
	adc1_config_width(ADC_WIDTH_BIT_12); 										// Config. a resolucao do ADC para 12bits.
	adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11); 					// Config. a atenuacao do Canal 0 (GPIO36).
	adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_11); 					// Config. a atenuacao do Canal 3 (GPIO39).
	/*
	Attenuation			Measurable input voltage range
	ADC_ATTEN_DB_0		100 mV ~ 950 mV
	ADC_ATTEN_DB_2_5	100 mV ~ 1250 mV
	ADC_ATTEN_DB_6		150 mV ~ 1750 mV
	ADC_ATTEN_DB_11 	150 mV ~ 2450 mV
	*/
}

/* Bloco Inicio: DHT 11/22 */
unsigned char   timeOut=0;														// Flag que indica o limite do tempo.
unsigned int    valor16bRh, valor16bTp;											// Var. global para salvar os valores do DHT 11/22.

struct dnthRegistro_t															// Estrutura de armazenamento.
{
    unsigned char rhInteiro;													// Parte inteira do valor da umidade.
    unsigned char rhDecimal;													// Parte decimal do valor da umidade.
    unsigned char tempInteiro;													// Parte inteira do valor da temperatura.
    unsigned char tempDecimal;													// Parte decimal do valor da temperatura.
    unsigned char checksum;
}dnthRegistro;																	// Variavel de acesso aos dados.

void dhtxxIniciar(void)															// Deve ser utilizada para disparar a leitura.
{
	gpio_reset_pin(___dhtPin);													// Limpa configuracoes anteriores.
	gpio_set_direction(___dhtPin, GPIO_MODE_OUTPUT);							// Configura o pino como saida.
	gpio_set_level(___dhtPin,0);                                             	// Coloca o Pino em '0'.
	ets_delay_us(20 * 1000);									                // Aguardar 20ms.
	gpio_set_level(___dhtPin,1);                                             	// Coloca o Pino em '1'.
	ets_delay_us(30);									                		// Aguardar 30us.
	gpio_set_direction(___dhtPin, GPIO_MODE_INPUT);								// Configura o pino como entrada.
}

char dhtxxLer(void)																// Executa a leitura dos bits com limite de tempo.
{
	char i, dado = 0;															// Var. temporaria.
	unsigned int k;        														// 'k' eh usado para contar 1 bit durante a leitura.
	if(timeOut) return (0);														// Se o tempo jah estorou o limite... volta.
	for(i = 0; i < 8; i++)														// Laco de leitura com limite de tempo.
	{
		k = 0;																	// Inicia o processo.
		while(!(gpio_get_level(___dhtPin) & 1))                 				// Laco que aguarda (subida do sinal) com timeout.
		{ 
			k++;																// Incrementa.
			if(k > 10000)														// Verifica se esta no limite do tempo.
			{
				timeOut = 1;													// Estourou o limite do tempo, ativa a flag.
				break;															// Interrompe o processo.
			}
			ets_delay_us(1);													// Aguarda 1us. Limite da largura do sinal.
		}
		ets_delay_us(30);														// Aguarda 1us. Limite da mudanca de nivel do sinal.
		if(gpio_get_level(___dhtPin)==1) dado=((dado<<1) | 1);					// Verifica se o dado eh '0' ou '1'. Se ativado, agregar o valor.
		else dado=(dado<<1);                    								// Senao, apenas desloca um bit a esquerda.
      
		while(gpio_get_level(___dhtPin) ==1)                 					// Laco que aguarda (descida do sinal) com timeout.
		{ 
            k++;																// Incrementa.
            if(k > 10000)														// Verifica se esta no limite do tempo.
            {
                timeOut = 1;													// Estourou o limite do tempo, ativa a flag.
                break;															// Interrompe o processo.
            }
			ets_delay_us(1);													// Aguarda 1us. Limite da largura do sinal.
		}
    }
  return (dado);																// Retorna com valor lido.
}

char dhtxxChecar(void)															// Verifica a resposta do DHT 11/22.
{
	ets_delay_us(40);															// Aguarda 40us. Limite do tempo de resposta anterior.
	if(gpio_get_level(___dhtPin)==0)											// Testa o pino se esta em 0.
	{
		ets_delay_us(80);														// Aguarda 80us. Limite do tempo de resposta.
		if(gpio_get_level(___dhtPin)==1)										// Testa o pino se esta em 1.
    	{
			ets_delay_us(50);													// Aguarda 50us. Limite do tempo de resposta.
			return (1);															// Ok!
    	}
	}
	return (0);																	// Falhou!
}

char __dhtHex2Dec(char valor) 													// Converte os valores do DHT11.
{
    unsigned cont, tst=1, charconv=0;											// Var. local temporaria.
    for(cont=0;cont<8;cont++) 													// Laco de varredura do Byte.
    {
		if((bitX(valor,cont))==1) charconv = charconv + tst;					// Testa todos os bits.
        tst += tst;																// Dobra o valor (1,2,4,8...).
    }
    return (charconv);															// Retorna o valor convertido.
}

char dhtxxValor(char valordht) 													// Faz leitura do do DHT escolhido.
{
    char testar, temp;															// Var. local temporaria.
    if(valordht<1)valordht=1;													// Limita o valor minimo.
    if(valordht>2)valordht=2;													// Limita o valor maximo.
    if(dhtxxChecar())															// Verifica a comunicacao antes de ler.
    {
        dnthRegistro.rhInteiro=dhtxxLer();										// Faz a leitura do 1o. grupo de dados e salva.
        dnthRegistro.rhDecimal=dhtxxLer();										// Faz a leitura do 2o. grupo de dados e salva.
        dnthRegistro.tempInteiro=dhtxxLer();									// Faz a leitura do 3o. grupo de dados e salva.
        dnthRegistro.tempDecimal=dhtxxLer();									// Faz a leitura do 4o. grupo de dados e salva.
        dnthRegistro.checksum=dhtxxLer();										// Faz a leitura do 5o. grupo de dados e salva.
    }
    
    if(valordht==1)																// Se for o DHT11...
    {
        temp = __dhtHex2Dec(dnthRegistro.rhInteiro);							// Converte o valor da umidade.
        valor16bRh = temp * 10;													// Corrige o valor.
        temp = __dhtHex2Dec(dnthRegistro.tempInteiro);							// Converte o valor da temperatura.
        valor16bTp = temp * 10;													// Corrige o valor.
    }
    
    if(valordht==2)																// Se for o DHT22...
    {
        valor16bRh = (dnthRegistro.rhInteiro<<8) | (dnthRegistro.rhDecimal);	// Combina parte inteira e decimal da umidade.
        valor16bTp = (dnthRegistro.tempInteiro<<8) | (dnthRegistro.tempDecimal);// Combina parte inteira e decimal da temperatura.
    }
    
	// Verifica se o conteudo dos dados sao validos.
    testar=dnthRegistro.rhInteiro+dnthRegistro.rhDecimal+dnthRegistro.tempInteiro+dnthRegistro.tempDecimal;
    if (dnthRegistro.checksum !=testar) return (0);								// Falha de leitura ou dados invalidos.
    else return (1);															// Ok!
}

void dhtxx(unsigned char modelo,unsigned int *umidade,unsigned int *temperatura)// Acesso ao DHT por modelo e variaveis.
{	
	if(dhtxxValor(modelo))														// Verifica se os dados sao validos...
	{
		*umidade = valor16bRh;													// Salva o valor da umidade.
		*temperatura = valor16bTp;												// Salva o valor da temperatura.
	}
}
/* Bloco Fim: DHT 11/22 */

/* Bloco Inicio: WiFi */
static int s_active_interfaces = 0;
static xSemaphoreHandle s_semph_get_ip_addrs;
static esp_netif_t *s_example_esp_netif = NULL;
#define MAX_IP6_ADDRS_PER_NETIF (5)
#define NR_OF_IP_ADDRESSES_TO_WAIT_FOR (s_active_interfaces)

/* Tipos de enderecos IPv6 a serem exibidos em eventos IPv6. */
static esp_ip6_addr_t s_ipv6_addr;

static const char *s_ipv6_addr_types[] = 
{
    "ESP_IP6_ADDR_IS_UNKNOWN",
    "ESP_IP6_ADDR_IS_GLOBAL",
    "ESP_IP6_ADDR_IS_LINK_LOCAL",
    "ESP_IP6_ADDR_IS_SITE_LOCAL",
    "ESP_IP6_ADDR_IS_UNIQUE_LOCAL",
    "ESP_IP6_ADDR_IS_IPV4_MAPPED_IPV6"
};

static void on_wifi_connect(void *esp_netif, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_netif_create_ip6_linklocal(esp_netif);
}

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Wi-Fi disconectado, tentando reconectar...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) 
    {
        return;
    }
    ESP_ERROR_CHECK(err);
}

/**
 * @brief Verifica se a descricao 'netif' contem o prefixo especificado.
 * Todos os 'netifs' criados dentro do componente de conexao comum sao prefixados com o modulo TAG, 
 * entao ele retorna 'true' se o 'netif' especificado for de propriedade deste modulo.
 */
static bool is_our_netif(const char *prefix, esp_netif_t *netif)
{
    return strncmp(prefix, esp_netif_get_desc(netif), strlen(prefix) - 1) == 0;
}

static esp_ip4_addr_t s_ip_addr;

static void on_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    if (!is_our_netif(TAG, event->esp_netif)) {
        ESP_LOGW(TAG, "Got IPv4 from another interface \"%s\": ignored", esp_netif_get_desc(event->esp_netif));
        return;
    }
    ESP_LOGI(TAG, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));
    memcpy(&s_ip_addr, &event->ip_info.ip, sizeof(s_ip_addr));
    xSemaphoreGive(s_semph_get_ip_addrs);
}

static void on_got_ipv6(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip6_t *event = (ip_event_got_ip6_t *)event_data;
    if (!is_our_netif(TAG, event->esp_netif)) {
        ESP_LOGW(TAG, "Got IPv6 from another netif: ignored");
        return;
    }
    esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&event->ip6_info.ip);
    ESP_LOGI(TAG, "Got IPv6 event: Interface \"%s\" address: " IPV6STR ", type: %s", esp_netif_get_desc(event->esp_netif),
             IPV62STR(event->ip6_info.ip), s_ipv6_addr_types[ipv6_type]);
    if (ipv6_type == EXAMPLE_CONNECT_PREFERRED_IPV6_TYPE) {
        memcpy(&s_ipv6_addr, &event->ip6_info.ip, sizeof(s_ipv6_addr));
        xSemaphoreGive(s_semph_get_ip_addrs);
    }
}

esp_netif_t *get_example_netif(void)
{
    return s_example_esp_netif;
}

esp_netif_t *get_example_netif_from_desc(const char *desc)
{
    esp_netif_t *netif = NULL;
    char *expected_desc;
    asprintf(&expected_desc, "%s: %s", TAG, desc);
    while ((netif = esp_netif_next(netif)) != NULL) {
        if (strcmp(esp_netif_get_desc(netif), expected_desc) == 0) {
            free(expected_desc);
            return netif;
        }
    }
    free(expected_desc);
    return netif;
}

static esp_netif_t *wifi_start(void)
{
    char *desc;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    // Prefixe a descricao da interface com o TAG do modulo.
    // Atencao: a interface desc eh usada em testes para capturar detalhes reais da conexao (IP, gw, mask).
    asprintf(&desc, "%s: %s", TAG, esp_netif_config.if_desc);
    esp_netif_config.if_desc = desc;
    esp_netif_config.route_prio = 128;
    esp_netif_t *netif = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
    free(desc);
    esp_wifi_set_default_wifi_sta_handlers();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &on_wifi_connect, netif));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_GOT_IP6, &on_got_ipv6, NULL));
#endif

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_EXAMPLE_WIFI_SSID,
            .password = CONFIG_EXAMPLE_WIFI_PASSWORD,
            .scan_method = EXAMPLE_WIFI_SCAN_METHOD,
            .sort_method = EXAMPLE_WIFI_CONNECT_AP_SORT_METHOD,
            .threshold.rssi = CONFIG_EXAMPLE_WIFI_SCAN_RSSI_THRESHOLD,
            .threshold.authmode = EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD,
        },
    };
    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();
    return netif;
}

static void wifi_stop(void)
{
    esp_netif_t *wifi_netif = get_example_netif_from_desc("sta");
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_GOT_IP6, &on_got_ipv6));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, &on_wifi_connect));
#endif
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) {
        return;
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(wifi_netif));
    esp_netif_destroy(wifi_netif);
    s_example_esp_netif = NULL;
}

/* Configura conexao, Wi-Fi e/ou Ethernet. */
static void start(void)
{

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    s_example_esp_netif = wifi_start();
    s_active_interfaces++;
#endif

#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    s_example_esp_netif = eth_start();
    s_active_interfaces++;
#endif

// Gera erro: 'warning: extra tokens at end of...'
// #ifdef CONFIG_EXAMPLE_CONNECT_WIFI && CONFIG_EXAMPLE_CONNECT_ETHERNET
// #ifdef defined (CONFIG_EXAMPLE_CONNECT_WIFI) && defined (CONFIG_EXAMPLE_CONNECT_ETHERNET)
//     /* if both intefaces at once, clear out to indicate that multiple netifs are active */
//     s_example_esp_netif = NULL;
// #endif

#ifdef EXAMPLE_DO_CONNECT
    /* Cria um 'semaphore' se ao menos uma interface estiver ativa. */
    s_semph_get_ip_addrs = xSemaphoreCreateCounting(NR_OF_IP_ADDRESSES_TO_WAIT_FOR, 0);
#endif

}

static void stop(void)
{
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    wifi_stop();
    s_active_interfaces--;
#endif

#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    eth_stop();
    s_active_interfaces--;
#endif
}

esp_err_t example_connect(void)
{
#ifdef EXAMPLE_DO_CONNECT
    if (s_semph_get_ip_addrs != NULL) 
	{
        return ESP_ERR_INVALID_STATE;
    }
#endif
    start();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&stop));
    ESP_LOGI(TAG, "Aguardando IP(s).");
    for (int i = 0; i < NR_OF_IP_ADDRESSES_TO_WAIT_FOR; ++i) 
	{
        xSemaphoreTake(s_semph_get_ip_addrs, portMAX_DELAY);
    }
    // Iteragir sobre interfaces ativas e imprima IPs de "nossos" netifs.
    esp_netif_t *netif = NULL;
    esp_netif_ip_info_t ip;
    for (int i = 0; i < esp_netif_get_nr_of_ifs(); ++i) 
	{
        netif = esp_netif_next(netif);
        if (is_our_netif(TAG, netif)) 
		{
            ESP_LOGI(TAG, "Conectado ao %s", esp_netif_get_desc(netif));
            ESP_ERROR_CHECK(esp_netif_get_ip_info(netif, &ip));

            ESP_LOGI(TAG, "- Endereco IPv4: " IPSTR, IP2STR(&ip.ip));
#ifdef CONFIG_EXAMPLE_CONNECT_IPV6
            esp_ip6_addr_t ip6[MAX_IP6_ADDRS_PER_NETIF];
            int ip6_addrs = esp_netif_get_all_ip6(netif, ip6);
            for (int j = 0; j < ip6_addrs; ++j) {
                esp_ip6_addr_type_t ipv6_type = esp_netif_ip6_get_addr_type(&(ip6[j]));
                ESP_LOGI(TAG, "- IPv6 address: " IPV6STR ", type: %s", IPV62STR(ip6[j]), s_ipv6_addr_types[ipv6_type]);
            }
#endif

        }
    }
    return ESP_OK;
}

esp_err_t example_disconnect(void)
{
    if (s_semph_get_ip_addrs == NULL) 
	{
        return ESP_ERR_INVALID_STATE;
    }
    vSemaphoreDelete(s_semph_get_ip_addrs);
    s_semph_get_ip_addrs = NULL;
    stop();
    ESP_ERROR_CHECK(esp_unregister_shutdown_handler(&stop));
    return ESP_OK;
}

#ifdef CONFIG_EXAMPLE_BASIC_AUTH

typedef struct 
{
    char    *username;
    char    *password;
} basic_auth_info_t;

#define HTTPD_401      "Erro: 401 Nao Autorizado"           					// Resposta HTTP 401.

static char *http_auth_basic(const char *username, const char *password)
{
    int out;
    char *user_info = NULL;
    char *digest = NULL;
    size_t n = 0;
    asprintf(&user_info, "%s:%s", username, password);
    if (!user_info) {
        ESP_LOGE(TAG, "Nao ha memoria suficiente para informacoes do usuario.");
        return NULL;
    }
    esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char *)user_info, strlen(user_info));

    /* 6: O comprimento da string "Basic".
     * n: Numero de bytes para um formato de codificacao base64.
     * 1: Numero de bytes reservado que sera usado para preencher com zero.
    */
    digest = calloc(1, 6 + n + 1);
    if (digest) {
        strcpy(digest, "Basic ");
        esp_crypto_base64_encode((unsigned char *)digest + 6, n, (size_t *)&out, (const unsigned char *)user_info, strlen(user_info));
    }
    free(user_info);
    return digest;
}

/* Um manipulador HTTP GET para autorizacao. */
static esp_err_t basic_auth_get_handler(httpd_req_t *req)
{
    char *buf = NULL;
    size_t buf_len = 0;
    basic_auth_info_t *basic_auth_info = req->user_ctx;

    buf_len = httpd_req_get_hdr_value_len(req, "Autorizacao") + 1;
    if (buf_len > 1) 
	{
        buf = calloc(1, buf_len);
        if (!buf) 
		{
            ESP_LOGE(TAG, "Sem memoria suficiente para autorizacao basica");
            return ESP_ERR_NO_MEM;
        }

        if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) 
		{
            ESP_LOGI(TAG, "Cabecalho encontrado => Autorizacao: %s", buf);
        } else 
		{
            ESP_LOGE(TAG, "Nenhum valor de autenticacao recebido.");
        }

        char *auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
        if (!auth_credentials) 
		{
            ESP_LOGE(TAG, "Nao ha memoria suficiente para credenciais de autorizacao basica.");
            free(buf);
            return ESP_ERR_NO_MEM;
        }

        if (strncmp(auth_credentials, buf, buf_len)) 
		{
            ESP_LOGE(TAG, "Nao autenticado.");
            httpd_resp_set_status(req, HTTPD_401);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
        } else 
		{
            ESP_LOGI(TAG, "Autenticado!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
            if (!basic_auth_resp) 
			{
                ESP_LOGE(TAG, "Sem memoria suficiente para resposta de autorizacao basica.");
                free(auth_credentials);
                free(buf);
                return ESP_ERR_NO_MEM;
            }
            httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
            free(basic_auth_resp);
        }
        free(auth_credentials);
        free(buf);
    } else 
	{
        ESP_LOGE(TAG, "Nenhum cabecalho de autenticacao recebido.");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
    }

    return ESP_OK;
}

static httpd_uri_t basic_auth = 
{
    .uri       = "/basic_auth",
    .method    = HTTP_GET,
    .handler   = basic_auth_get_handler,
};

static void httpd_register_basic_auth(httpd_handle_t server)
{
    basic_auth_info_t *basic_auth_info = calloc(1, sizeof(basic_auth_info_t));
    if (basic_auth_info) 
	{
        basic_auth_info->username = CONFIG_EXAMPLE_BASIC_AUTH_USERNAME;
        basic_auth_info->password = CONFIG_EXAMPLE_BASIC_AUTH_PASSWORD;

        basic_auth.user_ctx = basic_auth_info;
        httpd_register_uri_handler(server, &basic_auth);
    }
}
#endif

/* Um manipulador(handler) para o HTTP GET */
static esp_err_t geral_get_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;

    /* Obtem o comprimento da string do valor do cabecalho e aloca memoria para comprimento + 1, byte extra para terminacao nula. */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) 
    {
        buf = malloc(buf_len);
        /* Copia a string de valor de terminacao nula no buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) 
        {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) 
		{
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) 
		{
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Le o comprimento da string de consulta do URL e aloca memoria para comprimento + 1, byte extra para terminacao nula. */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Obtem o valor da chave esperada da string de consulta. */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Define alguns cabecalhos personalizados. */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Envia a resposta com cabecalhos personalizados e corpo definido como a string passada no contexto do usuario. */
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* Depois de enviar a resposta HTTP, os antigos cabecalhos de solicitacao HTTP sao perdidos. 
	   Verificar se os cabecalhos de solicitacao HTTP podem ser lidos agora. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) 
    {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static esp_err_t gpio_get_handler(httpd_req_t *req)
{
    char *buf;
    size_t buf_len;

    /* Obtem o comprimento da string do valor do cabecalho e aloca memoria para comprimento + 1, byte extra para terminacao nula. */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) 
    {
        buf = malloc(buf_len);
        /* Copia a string de valor de terminacao nula no buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) 
        {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) 
		{
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) 
		{
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Le o comprimento da string de consulta do URL e aloca memoria para comprimento + 1, byte extra para terminacao nula. */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) 
        {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            char vlrSaida[4];
            // unsigned int vlrMP;
            /* Obtem o valor da chave esperada da string de consulta. */
            if (httpd_query_key_value(buf, "saida", vlrSaida, sizeof(vlrSaida)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Parametro Saida= %s", vlrSaida);
                vlrQuery1=atol(vlrSaida);
            }
            if (httpd_query_key_value(buf, "ang", param, sizeof(param)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Angulo= %s", param);
                mpAngulo(atol(param),0,1);													// X graus a Dir. 1:1
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) 
			{
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
            }
        }
        free(buf);
    }

    /* Define alguns cabecalhos personalizados. */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Envia a resposta com cabecalhos personalizados e corpo definido como a string passada no contexto do usuario. */
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* Depois de enviar a resposta HTTP, os antigos cabecalhos de solicitacao HTTP sao perdidos. 
	   Verificar se os cabecalhos de solicitacao HTTP podem ser lidos agora. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) 
    {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

//             if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) 
// 			{
//                 ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
//                 // valorSaida=param;
//                 // vlrQuery1=param;
//             }
//             if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) 
// 			{
//                 ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
//             }
//             if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) 
// 			{
//                 ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
//             }
//         }
//         free(buf);
//     }

static const httpd_uri_t hello = 
{
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = geral_get_handler,
    /* String de resposta no contexto do usuario para demonstrar seu uso. */
    .user_ctx  = "Hello World!"
	// .user_ctx  = NULL															// Usar se nao houver retorno.
};

static const httpd_uri_t gpio = 
{
    .uri       = "/gpio",
    .method    = HTTP_GET,
    .handler   = gpio_get_handler,
    .user_ctx  = vlrAscIn
};

/* Um manipulador(handler) para o HTTP POST */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

    while (remaining > 0) 
    {
        if ((ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)))) <= 0) // Ler os dados da solicitacao.
        {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) 
            {
                continue;  														// Tente receber novamente se o tempo limite for atingido.
            }
            return ESP_FAIL;
        }
        httpd_resp_send_chunk(req, buf, ret);									// Envie de volta os mesmos dados.
        remaining -= ret;

        /* Log dos dados recebidos */
        ESP_LOGI(TAG, "========== Dados Recebidos =========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }
    httpd_resp_send_chunk(req, NULL, 0);										// Fim da resposta.
    return ESP_OK;
}

static const httpd_uri_t echo = 
{
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) 
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI nao esta disponivel.");
        /* Retorne ESP_OK para manter o socket subjacente aberto. */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) 
    {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI nao esta disponivel.");
        /* Retorne ESP_FAIL para fechar o socket subjacente. */
        return ESP_FAIL;
    }
    /* Para qualquer outro URI, envie 404 e feche o socket. */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Erro 404. Nao foi encontrado.");
    return ESP_FAIL;
}

static esp_err_t ctrl_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) 
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
        return ESP_FAIL;
    }

    if (buf == '0') 
    {
        /* Manipuladores de URI podem ser desregistrados usando a string de URI. */
        ESP_LOGI(TAG, "Removendo URIs: /hello, /echo.");
        httpd_unregister_uri(req->handle, "/hello");
        httpd_unregister_uri(req->handle, "/echo");

        // httpd_unregister_uri(req->handle, "/alex");
        // httpd_unregister_uri(req->handle, "/fred");
        // httpd_unregister_uri(req->handle, "/henrique");
        // httpd_unregister_uri(req->handle, "/jose");
        // httpd_unregister_uri(req->handle, "/leandro");
        // httpd_unregister_uri(req->handle, "/terrao");

        /* Registrar o manipulador de erros personalizado. */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    else 
    {
        ESP_LOGI(TAG, "Registrando URIs: /hello, /echo.");
        httpd_register_uri_handler(req->handle, &hello);
        httpd_register_uri_handler(req->handle, &echo);

        // httpd_register_uri_handler(req->handle, &alex);
        // httpd_register_uri_handler(req->handle, &fred);
        // httpd_register_uri_handler(req->handle, &henrique);
        // httpd_register_uri_handler(req->handle, &jose);
        // httpd_register_uri_handler(req->handle, &leandro);
        // httpd_register_uri_handler(req->handle, &terrao);

        /* Cancelar o registro do manipulador de erros personalizado. */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, NULL);
    }

    /* Responde com o corpo vazio. */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t out_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

    if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) 
    {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) httpd_resp_send_408(req);
        return ESP_FAIL;
    }

    if (buf > 0x29 && buf < 0x40)                                               // Entre '0' e '9'
    {
        ESP_LOGI(TAG, "Valor enviado. ");
        gpoDado(buf);													        // Envia ao GPO.
    }

    /* Responde com o corpo vazio. */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t ctrl = 
{
    .uri       = "/ctrl",
    .method    = HTTP_PUT,
    .handler   = ctrl_put_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t saida = 
{
    .uri       = "/saida",
    .method    = HTTP_PUT,
    .handler   = out_put_handler,
    .user_ctx  = NULL
};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 10; // Editado: numero de ate 10.

    // Inicia o Server httpd.
    ESP_LOGI(TAG, "Iniciando o Server na porta: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) 
    {
        // Configura o manipulador de URIs.
        ESP_LOGI(TAG, "Registrando as URIs.");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &ctrl);
        httpd_register_uri_handler(server, &saida);
        httpd_register_uri_handler(server, &gpio);
        // httpd_register_uri_handler(server, &mpasso);

        #ifdef CONFIG_EXAMPLE_BASIC_AUTH
        	httpd_register_basic_auth(server);
        #endif
        return server;
    }
    ESP_LOGI(TAG, "Falha ao iniciar o Server!");
    return NULL;
}

static esp_err_t stop_webserver(httpd_handle_t server)
{
    return httpd_stop(server);  // Parar o httpd server
}

static void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) 
    {
        ESP_LOGI(TAG, "Parando WebServer");
        if (stop_webserver(*server) == ESP_OK) *server = NULL;
        else ESP_LOGE(TAG, "Falha ao parar http server");
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) 
    {
        ESP_LOGI(TAG, "Iniciando WebServer");
        *server = start_webserver();
    }
}
/* Bloco Fim: WiFi */

void app_main(void)																// App principal.
{
	char *ourTaskName = pcTaskGetName("HardTest");								// Nome da TASK.
	ESP_LOGI(ourTaskName,"App Iniciado.");										// Imprime Info.

	/* Bloco do Wifi */
	static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
    server = start_webserver();													// Iniciar o servidor pela primeira vez.

	vTaskDelay(50); 									                        // 500ms. Aguarda LCD estabilizar.
    unsigned char vlrTecla = 0;													// Var. temp. da tecla.
	char valorEntrada = 0, valorSaida = 0;										// Var. temp. entrada e saida.
	char tstExpIn, tstExpOut;													// Var. temp. para teste da expansao.
    char vlrStr[] = {"T[_]"};													// Var. para valor da tecla.
	char vlrGpi[] = {"I[__]"};													// Var. em Hexa da entrada.
	char vlrGpo[] = {"O[__]"};													// Var. em Hexa da saida.
	char vlrExp[] = {"E[__]"};													// Var. em Hexa da expansao.
	char vlrMp[] = {"M[_]"};													// Var. indicando o sentido do motor de passo.
	unsigned int  vlrAn0,vlrAn1;												// Var. para o valor binario das entradas analogicas.
	char vlrAsc0[]={"0000"};													// Var. para o valor em ASCII.
	char vlrAsc1[]={"0000"};													// Var. para o valor em ASCII.
	unsigned int umidade, temperatura;											// 
	char vlrAsc2[]={"0000"};													// Var. para o valor em ASCII.
	char vlrAsc3[]={"0000"};													// Var. para o valor em ASCII.

	lcdIniciar();																// Inicia o hardware do LCD.
    tecladoIniciar();															// Inicia o hardware do teclado.
	gpoIniciar();																// Inicia o hardware do GPO.
	gpiIniciar();																// Inicia o hardware do GPI.
	expIniciar();																// Inicia o hardware da expansao.
	mpIniciar();																// Inicia o hardware do modulo DRV8825/A4988.
	adcIniciar();																// Inicia o hardware do modulo conversor ADC.

    lcdString("-= ESP32S =-",1,3);												// Splash
	vTaskDelay(100); 									                        // Aguarda 1000ms. 
	lcdString("            ",1,3);												// Apaga a linha.
    while(1)																	// Loop infinito.
    {
		/* Teste do teclado */
        vlrTecla = tecla();														// Le o teclado.
		if(vlrTecla>0) vlrStr[2] = vlrTecla;									// Se valido, salva.
        lcdString(vlrStr,1,1);													// Envia ao LCD.
		/* */

		/* Teste do GPI */
		valorEntrada = gpiDado();												// Le a GPI.
        int2Asc(valorEntrada,vlrAscIn,4);
		hex2Asc(valorEntrada,vlrGpi);											// Converte o valor.
		lcdString(vlrGpi,2,1);													// Envia ao LCD.
		/* */
		
		/* Teste do GPO */
		// valorSaida = valorEntrada;												// Equate.
        valorSaida =(char) vlrQuery1;
		gpoDado(valorSaida);													// Envia ao GPO.
        // gpoDado(vlrQuery1);													    // Envia ao GPO.
		hex2Asc(valorSaida,vlrGpo);												// Converte o valor.
		lcdString(vlrGpo,2,7);													// Envia ao LCD.
        // lcdString(vlrQuery1,1,1);													// Envia ao LCD.
		/* */

		/* Teste da Expansao */
		tstExpIn = expGPI();													// Le a expansao.
		tstExpOut = tstExpIn;													// Equate.
		hex2Asc(tstExpOut,vlrExp);												// Converte o valor.
		expGPO(tstExpOut);														// Envia para a expansao.
		lcdString(vlrExp,1,7);													// Envia ao LCD.
		/* */

		/* Teste do Modulo DRV8825/A4988 */
		if(vlrTecla=='1')														// Se a tecla for acionada...
		{
			vlrMp[2]='0';														// Direita.
			lcdString(vlrMp,2,13);												// Envia ao LCD.
			mpAngulo(90,0,1);													// 90 graus a Dir. 1:1
		}
		
		if(vlrTecla=='3')														// Se a tecla for acionada...
		{
			vlrMp[2]='1';														// Esquerda.
			lcdString(vlrMp,2,13);												// Envia ao LCD.
			mpAngulo(90,1,1);													// 90 graus a Esq. 1:1
		}
		
		vlrMp[2]='_';															// Padrao.
		lcdString(vlrMp,2,13);													// Envia ao LCD.
		/* */

		/* Teste do modulo conversor ADC. */
		if(vlrTecla=='A')														// Se a tecla for acionada...
		{
			vlrAn0 = adc1_get_raw(ADC1_CHANNEL_0);								// Le o pino GPIO36.
			vlrAn1 = adc1_get_raw(ADC1_CHANNEL_3);								// Le o pino GPIO39.
			int2Asc(vlrAn0,vlrAsc0,4); 											// Converte o valor 'int' em ASCII.
			int2Asc(vlrAn1,vlrAsc1,4); 											// Converte o valor 'int' em ASCII.
			lcdString(vlrAsc0,1,1); 											// Envia o valor convertido ao LCD.
			lcdString(vlrAsc1,1,7); 											// Envia o valor convertido ao LCD.
			vTaskDelay(100); 									                // Aguarda 1000ms.
		}
		/* */

		/* Teste do DHT11/DHT22*/
		if(vlrTecla=='C')														// Se a tecla for acionada...
		{
			dhtxxIniciar();														// Dispara a leitura no DHT 11.
			dhtxx(1,&umidade,&temperatura);										// Processo de selecao e armazenamento.
			int2Asc(umidade,vlrAsc2,4); 										// Converte o valor 'int' em ASCII.
			int2Asc(temperatura,vlrAsc3,4); 									// Converte o valor 'int' em ASCII.
			lcdString(vlrAsc2,2,1);												// Envia ao LCD.
			lcdString(vlrAsc3,2,7);												// Envia ao LCD.
			vTaskDelay(100); 									                // Aguarda 1000ms.
		}

		if(vlrTecla=='D')														// Se a tecla for acionada...
		{
			dhtxxIniciar();														// Dispara a leitura no DHT 22.
			dhtxx(2,&umidade,&temperatura);										// Processo de selecao e armazenamento.
			int2Asc(umidade,vlrAsc2,4); 										// Converte o valor 'int' em ASCII.
			int2Asc(temperatura,vlrAsc3,4); 									// Converte o valor 'int' em ASCII.
			lcdString(vlrAsc2,2,1);												// Envia ao LCD.
			lcdString(vlrAsc3,2,7);												// Envia ao LCD.
			vTaskDelay(100); 									                // Aguarda 1000ms.
		}
		/* */

		vTaskDelay(10); 														// Aguarda 100ms.
    }
}
