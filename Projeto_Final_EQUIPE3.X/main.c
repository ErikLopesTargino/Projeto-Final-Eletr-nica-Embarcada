#include "mcc_generated_files/mcc.h"
#include "main.h"

// Fun��o que converte um caractere ASCII em seu equivalente bin�rio.
uint8_t ascii2bin(uint8_t caractereAscii){
    if (caractereAscii < 0x3A) {                // Verifica se o caractere � um d�gito num�rico ('0' a '9').
        return caractereAscii - 0x30;           // Converte '0'..'9' para 0..9 em bin�rio.
    } else {                                    // Caso o caractere seja uma letra ('A' a 'F').
        return caractereAscii - 0x37;           // Converte 'A'..'F' para 10..15 em bin�rio.
    }
}

// Fun��o que converte um valor bin�rio em seu equivalente ASCII.
uint8_t bin2ascii(uint8_t valorBinario){    
    valorBinario &= 0x0F;                   // Mant�m apenas os 4 bits inferiores.
    if (valorBinario < 10) {                // Verifica se est� no intervalo de 0 a 9.
        return valorBinario + 0x30;         // Converte 0..9 para '0'..'9' em ASCII.
    } else {                                // Caso seja um n�mero de 10 a 15.
        return valorBinario + 0x37;         // Converte 10..15 para 'A'..'F' em ASCII.
    }
}

// Fun��o que adiciona uma solicita��o de movimento do elevador � fila
void adicionaSolicitacaoNaFila(uint8_t pisoOrigem, uint8_t pisoDestino) {
    // Verifica se a solicita��o � v�lida e pode ser atendida.
    if (pisoOrigem != pisoDestino && totalSolicitacoes <= MAX_SOLICITACOES) {
        filaSolicitacoes[totalSolicitacoes].origem = pisoOrigem;            // Adiciona a origem � fila.
        filaSolicitacoes[totalSolicitacoes].destino = pisoDestino;          // Adiciona o destino � fila.
        flagOrigens[pisoOrigem] = 1;                                        // Marca o piso solicitado como origem.
        flagDestinos[pisoDestino] = 1;                                      // Marca o piso solicitado como destino.
        totalSolicitacoes++;                                                // Incrementa o n�mero de solicita��es.
    }
}

// Fun��o que controla a opera��o do motor do elevador com base no estado de movimento
void controlaMotorElevador(enum movimento aux) {
    switch(aux) {
        case parado:                            // Para o elevador.
            PWM3_LoadDutyValue(ZERO);           // Define o valor de PWM como ZERO para parar o motor.
            break;
        case subindo:                           // Move o elevador para cima.
            DIR_SetHigh();                      // Ativa o sinal para subir.
            PWM3_LoadDutyValue(PWM_ATIVO);      // Ativa o motor para subir (40% Duty_Cycle).
            break;
        case descendo:                          // Move o elevador para baixo.
            DIR_SetLow();                       // Ativa o sinal para descer.
            PWM3_LoadDutyValue(PWM_ATIVO);      // Ativa o motor para descer (40% Duty_Cycle).
            break;
    }
}

// Configura a matriz de LEDs para indicar que o elevador est� subindo.
void subindoMatriz() {

}

// Configura a matriz de LEDs para indicar que o elevador est� descendo.
void descendoMatriz() {
    
}

// Configura a matriz de LEDs para indicar que o elevador est� parado.
void paradoMatriz() {

}
// Fun��o para parar o elevador e atualizar o estado e a origem para um andar espec�fico
void PararElevadorEAtualizarEstado(int andar) {
    estado = parado;                            // Define o estado do elevador para parado
    controlaMotorElevador(estado);              // Atualiza o motor do elevador para refletir o novo estado
    __delay_ms(2000);                           // Espera 2 segundos para permitir embarque/desembarque
    flagOrigens[andar] = 0;                     // Zera a flag de origem para o andar atual
}

// Fun��o para atualizar as solicita��es na fila para um andar espec�fico
void AtualizarFilaSolicitacoes(int andar, bool checarDestino) {
    for (int i = 0; i < 5; i++) {                                           // Itera sobre as solicita��es na fila
        if (filaSolicitacoes[i].origem == andar) {                          // Verifica se a origem da solicita��o � o andar atual
            filaSolicitacoes[i].origem = PROCESSADO;                        // Marca a origem como processada
            if (checarDestino && filaSolicitacoes[i].destino == andar) {    // Se necess�rio, verifica tamb�m o destino
                filaSolicitacoes[i].destino = PROCESSADO;                   // Marca o destino como processado
                totalSolicitacoes--;                                        // Decrementa o total de solicita��es pendentes
            }
        }
    }
}

// Fun��o para atualizar a flag de destino para um andar espec�fico
void AtualizarFlagDestino(int andar) {
    flagDestinos[andar] = 0;                                 // Inicializa a flag de destino como desativada
    for (int i = 0; i < 5; i++) {                            // Itera sobre as solicita��es na fila
        if (filaSolicitacoes[i].destino == andar) {          // Verifica se o destino da solicita��o � o andar atual
            flagDestinos[andar] = 1;                         // Ativa a flag de destino para o andar
            break;                                           // Encerra o loop assim que encontra um destino correspondente
        }
    }
}

// Fun��o principal para completar a solicita��o em um andar espec�fico
void CompletaSolicitacao(int andar) {
    bool temDestino = flagDestinos[andar] == 1;         // Verifica se h� destino no andar
    bool temOrigem = flagOrigens[andar] == 1;           // Verifica se h� origem no andar
    if (temOrigem) {                                    // Se h� origem no andar
        PararElevadorEAtualizarEstado(andar);           // Para o elevador e atualiza o estado e a origem
        AtualizarFilaSolicitacoes(andar, temDestino);   // Atualiza as solicita��es na fila
    }
    
    if (temDestino && !temOrigem) {                     // Se h� destino mas n�o h� origem no andar
        PararElevadorEAtualizarEstado(andar);           // Para o elevador e atualiza o estado
    }
    AtualizarFlagDestino(andar);                        // Atualiza a flag de destino para o andar
}

// Decide se o elevador deve subir ou descer a partir do andar 1
void determinaProximoDestinoPiso1(){
    
}

// Decide se o elevador deve subir ou descer a partir do andar 2
void determinaProximoDestinoPiso2()
{
   
}

// Rotina para lidar com as opera��es no andar 0
void Andar0() {

}

// Rotina para lidar com as opera��es no andar 1
void Andar1() {
   
}

// Rotina para lidar com as opera��es no andar 2
void Andar2() {
   
}

// Rotina para lidar com as opera��es no andar 3
void Andar3() {
    
}

// Muda a posi��o do elevador com base em uma interrup��o
void Interrupt_CCP4(uint16_t valor_capturado) {
    
}

// Inicializa e configura a matriz de LEDs
void initMatrix() {
    for (uint8_t i = 0; i < 8; i++) {
        matriz[i] = 0;                                          // Zera os d�gitos da matriz (desliga LEDs).
    }
    // Configura os par�metros da matriz de LEDs.
    for (uint8_t i = 0; i < 12; i += 2) {
        uint8_t dado[2];
        for (uint8_t j = 0; j < 2; j++) {
            dado[j] = matrix_conf[i + j];
        }
        CS_SetLow();
        SPI1_ExchangeBlock(dado, 2);
        CS_SetHigh();
    }
}

//Fun��o que atualiza a matriz de LEDs com os valores predefinidos
void matrixUpdate() {
    uint8_t data[2];                            // Buffer para transmiss�o SPI.
    if (flip_matrix) {
        for (uint8_t i = 8; i > 0; i--) {
            data[0] = i;                        // Define o d�gito da matriz a ser atualizado.
            data[1] = matriz[i - 1];            // Define o valor para o d�gito correspondente.
            txSpi(data, 2);                     // Transmite os dados para a matriz de LEDs.
        }
    } else {
        uint8_t indice = 7;                     // �ndice para acessar os dados da matriz.
        for (uint8_t i = 1; i < 9; i++) {
            data[0] = i;                        // Define o d�gito da matriz a ser atualizado.
            data[1] = matriz[indice];           // Define o valor para o d�gito correspondente.
            txSpi(data, 2);                     // Transmite os dados para a matriz de LEDs.
            indice--;                           // Decrementa o �ndice para acessar o pr�ximo d�gito.
        }
    }
}

// Fun��o para transmitir dados via SPI
void txSpi(uint8_t *data, size_t dataSize) {
    CS_SetLow();                                // Ativa o Chip Select para iniciar a transmiss�o.
    SPI1_ExchangeBlock(data, dataSize);         // Realiza a transmiss�o de dados.
    CS_SetHigh();                               // Desativa o Chip Select ap�s a transmiss�o.
}

void main(void)
{
    SYSTEM_Initialize();
    
    IOCBF0_SetInterruptHandler(Andar0);    // Interrup��o para o andar 0
    IOCBF3_SetInterruptHandler(Andar1);    // Interrup��o para o andar 1
    CMP1_SetInterruptHandler(Andar2);      // Interrup��o para o andar 2
    CMP2_SetInterruptHandler(Andar3);      // Interrup��o para o andar 3
    CCP4_SetCallBack(Interrupt_CCP4);
    
    CS_SetHigh();                          // CS desativado
    SPI1_Open(SPI1_DEFAULT);               // Configura MSSP1
    initMatrix();                          // Inicializa a matriz de led

    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    // Configura a fila inicialmente com todas as solicita��es marcadas como processadas
    for (int i=0;i<5;i++){
        filaSolicitacoes[i].origem = PROCESSADO;
        filaSolicitacoes[i].destino = PROCESSADO;
    }
    uint8_t rx_char;

    while (1)
    {
        if(EUSART_is_rx_ready()){                                                        // Verifica se h� um byte dispon�vel para leitura
            while(EUSART_is_rx_ready()){                                                 // L� o byte recebido
                rx_char = EUSART_Read();    
            }                                     
            if (rx_char == INI_QUADRO_COMUNI){                                           // Verifica se o byte recebido marca o in�cio do quadro
                cont_RX = 0;                                                             // Reinicia o contador de bytes recebidos
            }
            else if (rx_char == FIM_QUADRO_COMUNI && cont_RX == 2){                      // Verifica se o byte recebido � o fim do quadro e se dois bytes foram recebidos
                adicionaSolicitacaoNaFila(ascii2bin(buffer[0]),ascii2bin(buffer[1]));    // Processa os bytes recebidos e registra a solicita��o
                Andar0();                                                                // Executa a rotina de interrup��o do andar 0
            }
            else
                if (cont_RX < 2){               // Se ainda n�o foram recebidos dois bytes
                    buffer[cont_RX] = rx_char;  // Armazena o byte recebido no buffer
                    cont_RX++;                  // Incrementa o contador de bytes recebidos
                }
        }
    }
}