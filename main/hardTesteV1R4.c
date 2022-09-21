#include <stdio.h>																// Acesso as opercoes de entradas e/ou saidas.
#include <driver/adc.h>															// Modulo conversor ADC.
#include "freertos/FreeRTOS.h"													// Acesso aos termos.
#include "freertos/task.h"														// Acesso as prioridades da TASK.
#include "driver/gpio.h"														// Acesso ao uso das GPIOs.
#include "esp_log.h"															// Acesso ao uso dos LOGs.
#include "rom/ets_sys.h"

/* RTOS */
#define CONFIG_FREERTOS_HZ 100													// Definicao da Espressif.

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

void app_main(void)																// App principal.
{
	char *ourTaskName = pcTaskGetName("HardTest");								// Nome da TASK.
	ESP_LOGI(ourTaskName,"App Iniciado.");										// Imprime Info.

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
		hex2Asc(valorEntrada,vlrGpi);											// Converte o valor.
		lcdString(vlrGpi,2,1);													// Envia ao LCD.
		/* */
		
		/* Teste do GPO */
		valorSaida = valorEntrada;												// Equate.
		gpoDado(valorSaida);													// Envia ao GPO.
		hex2Asc(valorSaida,vlrGpo);												// Converte o valor.
		lcdString(vlrGpo,2,7);													// Envia ao LCD.
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

