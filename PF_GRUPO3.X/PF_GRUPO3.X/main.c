#include "mcc_generated_files/mcc.h"
#include "main.h"

// Função que converte um caractere ASCII em seu equivalente binário.
uint8_t ascii2bin(uint8_t caractereAscii){
    if (caractereAscii < 0x3A) {                // Verifica se o caractere é um dígito numérico ('0' a '9').
        return caractereAscii - 0x30;           // Converte '0'..'9' para 0..9 em binário.
    } else {                                    // Caso o caractere seja uma letra ('A' a 'F').
        return caractereAscii - 0x37;           // Converte 'A'..'F' para 10..15 em binário.
    }
}

// Função que converte um valor binário em seu equivalente ASCII.
uint8_t bin2ascii(uint8_t valorBinario){    
    valorBinario &= 0x0F;                   // Mantém apenas os 4 bits inferiores.
    if (valorBinario < 10) {                // Verifica se está no intervalo de 0 a 9.
        return valorBinario + 0x30;         // Converte 0..9 para '0'..'9' em ASCII.
    } else {                                // Caso seja um número de 10 a 15.
        return valorBinario + 0x37;         // Converte 10..15 para 'A'..'F' em ASCII.
    }
}

// Função que adiciona uma solicitação de movimento do elevador à fila
void adicionaSolicitacaoNaFila(uint8_t pisoOrigem, uint8_t pisoDestino) {
    // Verifica se a solicitação é válida e pode ser atendida.
    if (pisoOrigem != pisoDestino && totalSolicitacoes <= MAX_SOLICITACOES) {
        filaSolicitacoes[totalSolicitacoes].origem = pisoOrigem;            // Adiciona a origem à fila.
        filaSolicitacoes[totalSolicitacoes].destino = pisoDestino;          // Adiciona o destino à fila.
        flagOrigens[pisoOrigem] = 1;                                        // Marca o piso solicitado como origem.
        flagDestinos[pisoDestino] = 1;                                      // Marca o piso solicitado como destino.
        totalSolicitacoes++;                                                // Incrementa o número de solicitações.
    }
}

// Função que controla a operação do motor do elevador com base no estado de movimento
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

// Configura a matriz de LEDs para indicar que o elevador está subindo.
void subindoMatriz() {
    // Configura a representação de uma seta apontando para cima na matriz de LEDs.
    matriz[4] = 0b00000000;             // Linha superior da seta.
    matriz[5] = 0b01000000;             // Linha do meio da seta.
    matriz[6] = 0b10000000;             // Base da seta.
    matriz[7] = 0b01000000;             // Finaliza a forma da seta.
    matrixUpdate();                     // Atualiza a matriz de LEDs.
}

// Configura a matriz de LEDs para indicar que o elevador está descendo.
void descendoMatriz() {
    // Configura a representação de uma seta apontando para baixo na matriz de LEDs.
    matriz[4] = 0b00000000;             // Linha superior da seta.
    matriz[5] = 0b01000000;             // Linha do meio da seta.
    matriz[6] = 0b00100000;             // Ponta da seta.
    matriz[7] = 0b01000000;             // Finaliza a forma da seta.
    matrixUpdate();                     // Atualiza a matriz de LEDs.
}

// Configura a matriz de LEDs para indicar que o elevador está parado.
void paradoMatriz() {
    // Configura a representação de uma linha reta indicando estado parado na matriz de LEDs.
    matriz[4] = 0b00000000;             // Linha superior.
    matriz[5] = 0b01000000;             // Linha do meio.
    matriz[6] = 0b01000000;             // Linha inferior.
    matriz[7] = 0b01000000;             // Finaliza a representação.
    matrixUpdate();                     // Atualiza a matriz de LEDs.
}

// Função principal para completar a solicitação em um andar específico
void CompletaSolicitacao (int x)
{
    int i;
    if (flagDestinos[x] == 1 && flagOrigens[x] == 1){ // Se há destinos e origens no andar atual
        estado = parado;                      // Para o elevador
        controlaMotorElevador(estado);
        __delay_ms(2000);                     // Espera 2s para embarque/desembarque
        flagOrigens[x] = 0;                       // Zera a flag de origens do andar atual
        for (i=0;i<5;i++){                    // Verifica quais elementos da fila correspondem
            if (filaSolicitacoes[i].origem == x)          // ao andar atual e os marca como ATENDIDO.
                filaSolicitacoes[i].origem = PROCESSADO;
            if (filaSolicitacoes[i].destino == x && filaSolicitacoes[i].origem == PROCESSADO){
                filaSolicitacoes[i].destino = PROCESSADO;   // Apenas marca o destino como atendido se a origem também estiver
                totalSolicitacoes--;        // Decrementa solicitações
            }
        }    
    }
    else if (flagOrigens[x] == 1){              // Se há apenas origens no andar atual
        estado = parado;                    // Para o elevador
        controlaMotorElevador(estado);
        __delay_ms(2000);                   // Espera 2s para embarque
        flagOrigens[x] = 0;                     // Zera a flag de origens do andar atual
        for (i=0;i<5;i++){                  // Verifica quais origens da fila correspondem
            if (filaSolicitacoes[i].origem == x)        // ao andar atual e as marca como atendidas.
                filaSolicitacoes[i].origem = PROCESSADO;  
        } 
    }
    else if (flagDestinos[x] == 1){             // Se há apenas destinos no andar atual
        estado = parado;                    // Para o elevador
        controlaMotorElevador(estado);
        __delay_ms(2000);                   // Espera 2s para embarque
        for (i=0;i<5;i++){
            if (filaSolicitacoes[i].destino == x && filaSolicitacoes[i].origem == PROCESSADO){
                filaSolicitacoes[i].destino = PROCESSADO; // Apenas marca o destino como atendido se a origem também estiver
                totalSolicitacoes--;      // Decrementa solicitações
            }
        } 
    }
    
    for (i=0;i<5;i++){
        if (filaSolicitacoes[i].destino == x){ // Verifica se há destinos não atendidos no andar atual
            flagDestinos[x] = 1;       // Se sim, ativa a flag
            break;   
        }
        else                       // Se não, desativa
            flagDestinos[x] = 0;        
    }
}

// Decide se o elevador deve subir ou descer a partir do andar 1
void determinaProximoDestinoPiso1(){
    // Se há solicitações acima ou abaixo, ou se o estado atual não requer mudança
    if ((flagOrigens[0] == 1 && (flagOrigens[2] == 1 || flagOrigens[3] == 1)) || 
        (estado == subindo && (flagOrigens[2] == 1 || flagOrigens[3] == 1 || flagDestinos[2] == 1 || flagDestinos[3] == 1)) ||
        (estado == descendo && (flagOrigens[0] == 1 || (flagDestinos[2] == 1 && flagDestinos[3] == 1)))) {
        controlaMotorElevador(estado);
    }
    else {
        // Mudança de direção necessária
        estado = (estado == subindo) ? descendo : subindo;
        __delay_ms(500);
        controlaMotorElevador(estado);
    }
}

// Decide se o elevador deve subir ou descer a partir do andar 2
void determinaProximoDestinoPiso2()
{
    // Verifica se há solicitações acima ou abaixo do piso 2, ou se o estado atual não requer mudança.
    if ((flagOrigens[3] == 1 && (flagOrigens[0] == 1 || flagOrigens[1] == 1)) ||
        (estado == subindo && (flagOrigens[3] == 1 || flagDestinos[3] == 1)) ||
        (estado == descendo && ((flagOrigens[0] == 1 || flagOrigens[1] == 1) || (flagDestinos[3] == 1 && (flagDestinos[0] == 0 && flagDestinos[1] == 0))))){ 
        // Se alguma das condições acima é verdadeira, mantém o movimento atual do elevador.
        controlaMotorElevador(estado);                    
    }
    else {
        // Se nenhuma das condições acima for atendida, significa que é necessário mudar a direção do elevador.
        estado = (estado == subindo) ? descendo : subindo;  // Alterna o estado entre subindo e descendo, dependendo do estado atual.
        __delay_ms(500);
        controlaMotorElevador(estado);                      // Aplica a mudança de estado ao motor do elevador.
    }
}

// Rotina para lidar com as operações no andar 0
void Andar0() {
    andar_atual = 0;                                // Define o andar atual como 0.
    CompletaSolicitacao(andar_atual);               // Processa solicitações pendentes para o andar 0.
    // Verifica se é necessário parar o elevador devido a uma mudança de estado.
    if (estado != parado) { 
        estado = parado;                            // Muda o estado para 'parado'.
        controlaMotorElevador(estado);              // Envia o comando para parar o motor do elevador.
        __delay_ms(500);                            // Pausa para estabilização após a parada.
    }
    // Verifica se há novas solicitações para determinar o próximo movimento do elevador.
    if (totalSolicitacoes > 0) {
        estado = subindo;                           // Define o estado para 'subindo'.
        controlaMotorElevador(estado);              // Envia o comando para subir.
    }
    // Representação do número 0 na matriz de LED.
    matriz[0] = 0b01111110; 
    matriz[1] = 0b10000001;
    matriz[2] = 0b10000001;
    matriz[3] = 0b01111110;
    // Atualiza o estado do elevador na matriz de LED.
    if (estado == subindo) {
        subindoMatriz();
    } else if (estado == descendo) {
        descendoMatriz();
    } else {
        paradoMatriz();
    }
}

// Rotina para lidar com as operações no andar 1
void Andar1() {
    andar_atual = 1;                            // Define o andar atual como 1.
    estado_anterior = estado;                   // Armazena temporariamente o estado atual.
    CompletaSolicitacao(andar_atual);           // Processa solicitações pendentes para o andar 1.
    estado = estado_anterior;                   // Restaura o estado anterior.
    determinaProximoDestinoPiso1();             // Decide a próxima ação do elevador baseada no andar 1.

    // Representação do número 1 na matriz de LED.
    matriz[0] = 0b00000000; 
    matriz[1] = 0b00100001;
    matriz[2] = 0b11111111;
    matriz[3] = 0b00000001;
    // Atualiza o estado do elevador na matriz de LED.
    if (estado == subindo) {
        subindoMatriz();
    } else if (estado == descendo) {
        descendoMatriz();
    } else {
        paradoMatriz();
    }
}

// Rotina para lidar com as operações no andar 2
void Andar2() {
    andar_atual = 2;                        // Define o andar atual como 2.
    estado_anterior = estado;               // Armazena temporariamente o estado atual.
    CompletaSolicitacao(andar_atual);       // Processa solicitações pendentes para o andar 2.
    estado = estado_anterior;               // Restaura o estado anterior.
    determinaProximoDestinoPiso2();         // Decide a próxima ação do elevador baseada no andar 2.

    // Representação do número 2 na matriz de LED.
    matriz[0] = 0b01000011; 
    matriz[1] = 0b10000101;       
    matriz[2] = 0b10001001;     
    matriz[3] = 0b01110001;
    // Atualiza o estado do elevador na matriz de LED.
    if (estado == subindo) {
        subindoMatriz();
    } else if (estado == descendo) {
        descendoMatriz();
    } else {
        paradoMatriz();
    }
}

// Rotina para lidar com as operações no andar 3
void Andar3() {
    andar_atual = 3;                    // Define o andar atual como 3.
    CompletaSolicitacao(andar_atual);   // Processa solicitações pendentes para o andar 3.
    // Verifica se é necessário parar o elevador devido a uma mudança de estado.
    if (estado != parado) {
        estado = parado;                // Muda o estado para 'parado'.
        controlaMotorElevador(estado);  // Envia o comando para parar o motor do elevador.
        __delay_ms(500);                // Pausa para estabilização após a parada.
    }
    estado = descendo;                  // Define o estado para 'descendo'.
    controlaMotorElevador(estado);      // Envia o comando para descer.

    // Representação do número 3 na matriz de LED.
    matriz[0] = 0b01000010; 
    matriz[1] = 0b10010001;
    matriz[2] = 0b10010001;
    matriz[3] = 0b01101110;
    // Atualiza o estado do elevador na matriz de LED.
    if (estado == subindo) {
        subindoMatriz();
    } else if (estado == descendo) {
        descendoMatriz();
    } else {
        paradoMatriz();
    }
}

// Muda a posição do elevador com base em uma interrupção
void Interrupt_CCP4(uint16_t valor_capturado) {
    tempo_captura = ((float)valor_capturado)*0.0000001;            // Converte o valor capturado para segundos.
    if (estado = subindo)                                       // Incrementa a posição se o elevador está subindo.
        posicao++;
    else                                                        // Decrementa a posição se o elevador está descendo.
        posicao--;
}

// Inicializa e configura a matriz de LEDs
void initMatrix() {
    for (uint8_t i = 0; i < 8; i++) {
        matriz[i] = 0;                                          // Zera os dígitos da matriz (desliga LEDs).
    }
    // Configura os parâmetros da matriz de LEDs.
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

//Função que atualiza a matriz de LEDs com os valores predefinidos
void matrixUpdate() {
    uint8_t data[2];                            // Buffer para transmissão SPI.
    if (flip_matrix) {
        for (uint8_t i = 8; i > 0; i--) {
            data[0] = i;                        // Define o dígito da matriz a ser atualizado.
            data[1] = matriz[i - 1];            // Define o valor para o dígito correspondente.
            txSpi(data, 2);                     // Transmite os dados para a matriz de LEDs.
        }
    } else {
        uint8_t indice = 7;                     // Índice para acessar os dados da matriz.
        for (uint8_t i = 1; i < 9; i++) {
            data[0] = i;                        // Define o dígito da matriz a ser atualizado.
            data[1] = matriz[indice];           // Define o valor para o dígito correspondente.
            txSpi(data, 2);                     // Transmite os dados para a matriz de LEDs.
            indice--;                           // Decrementa o índice para acessar o próximo dígito.
        }
    }
}

// Função para transmitir dados via SPI
void txSpi(uint8_t *data, size_t dataSize) {
    CS_SetLow();                                // Ativa o Chip Select para iniciar a transmissão.
    SPI1_ExchangeBlock(data, dataSize);         // Realiza a transmissão de dados.
    CS_SetHigh();                               // Desativa o Chip Select após a transmissão.
}

void main(void)
{
    SYSTEM_Initialize();
    
    IOCBF0_SetInterruptHandler(Andar0);    // Interrupção para o andar 0
    IOCBF3_SetInterruptHandler(Andar1);    // Interrupção para o andar 1
    CMP1_SetInterruptHandler(Andar2);      // Interrupção para o andar 2
    CMP2_SetInterruptHandler(Andar3);      // Interrupção para o andar 3
    CCP4_SetCallBack(Interrupt_CCP4);
    //TMR0_SetInterruptHandler(trasmiteDadosUart);
    
    CS_SetHigh();                          // CS desativado
    SPI1_Open(SPI1_DEFAULT);               // Configura MSSP1
    initMatrix();                          // Inicializa a matriz de led

    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    // Configura a fila inicialmente com todas as solicitações marcadas como processadas
    for (int i=0;i<5;i++){
        filaSolicitacoes[i].origem = PROCESSADO;
        filaSolicitacoes[i].destino = PROCESSADO;
    }
    uint8_t rx_char;

    while (1)
    {
        if(EUSART_is_rx_ready()){                                                        // Verifica se há um byte disponível para leitura
            while(EUSART_is_rx_ready()){                                                 // Lê o byte recebido
                rx_char = EUSART_Read();    
            }                                     
            if (rx_char == INI_QUADRO_COMUNI){                                           // Verifica se o byte recebido marca o início do quadro
                cont_RX = 0;                                                             // Reinicia o contador de bytes recebidos
            }
            else if (rx_char == FIM_QUADRO_COMUNI && cont_RX == 2){                      // Verifica se o byte recebido é o fim do quadro e se dois bytes foram recebidos
                adicionaSolicitacaoNaFila(ascii2bin(buffer[0]),ascii2bin(buffer[1]));    // Processa os bytes recebidos e registra a solicitação
                Andar0();                                                                // Executa a rotina de interrupção do andar 0
            }
            else
                if (cont_RX < 2){               // Se ainda não foram recebidos dois bytes
                    buffer[cont_RX] = rx_char;  // Armazena o byte recebido no buffer
                    cont_RX++;                  // Incrementa o contador de bytes recebidos
                }
        }
    }
}