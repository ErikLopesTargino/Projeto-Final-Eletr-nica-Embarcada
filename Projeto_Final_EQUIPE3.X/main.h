#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h>
#include "mcc_generated_files/mcc.h"

// Constantes e defini��es
#define ZERO 0                              // Valor ZERO para uso geral
#define INI_QUADRO_COMUNI '$'               // Caractere que indica o in�cio do quadro de comunica��o
#define FIM_QUADRO_COMUNI 0x0D              // Caractere que indica o fim do quadro de comunica��o
#define flip_matrix false                   // Define se a matriz de LEDs deve ser invertida horizontalmente
#define MAX_SOLICITACOES 5                  // N�mero m�ximo de solicita��es que podem ser armazenadas na fila
#define PROCESSADO 8                        // Valor indicativo de que uma solicita��o foi processada
#define PWM_ATIVO 512                       // Valor de PWM para ativar o motor com um duty cycle de 40%

// Estrutura para armazenar solicita��es de movimento do elevador
typedef struct {
    uint8_t origem;    // Andar de origem da solicita��o
    uint8_t destino;   // Andar de destino da solicita��o
} solicitacao;

// Vari�veis globais
int16_t velocidade;
solicitacao filaSolicitacoes[MAX_SOLICITACOES]; // Fila para armazenar as solicita��es do elevador
uint8_t matriz[8];                              // Array para armazenar os estados da matriz de LEDs
uint8_t totalSolicitacoes = 0;                  // Contador do total de solicita��es na fila
uint8_t andar_atual = 0;                        // Andar atual do elevador, iniciando do andar 0
uint8_t flagDestinos[4] = {0, 0, 0, 0};         // Flags para os destinos em cada andar
uint8_t flagOrigens[4] = {0, 0, 0, 0};          // Flags para as origens em cada andar

// Enumera��es para estados de movimento do elevador
enum movimento {descendo, subindo, parado};
enum movimento estado = parado;                // Estado atual do movimento do elevador
enum movimento estado_anterior = parado;       // Armazena o estado anterior do elevador

uint8_t buffer[2];                             // Buffer para armazenar dados recebidos via comunica��o
uint8_t cont_RX = 0;                           // Contador para os dados recebidos
uint8_t posicao = 0;                           // Posi��o atual do elevador
float tempo_captura;                           // Vari�vel para armazenar o tempo de captura

// Configura��o inicial da matriz de LEDs
const uint8_t matrix_conf[] = {
    0x09, 0x00,  // Decode mode = 0
    0x0A, 0x00,  // Intensity 1/32
    0x0B, 0x07,  // Scan Limit
    0x0C, 0x01,  // Shutdown mode = 1
    0x0F, 0x01,  // Display-Test = 1
    0x0F, 0x00,  // Display-Test = 0
};

// Tipo enum para sele��o do dispositivo no barramento SPI
typedef enum {
    spiCS             // CS da matriz de LEDs (MAX7219)
} spiCS_t;

// Declara��es de fun��es
void initMatrix();                          // Inicializa a matriz de LEDs
void matrixUpdate();                        // Atualiza a matriz de LEDs com os valores atuais
void txSpi(uint8_t *data, size_t dataSize); // Transmite dados via SPI
 //OBS: adicionar as demais fun��es

#endif