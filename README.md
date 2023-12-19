# Projeto Final

## Projeto criado utilizando o GitHub Desktop, Vscode e MPLAB X IDE

Curso de Eletrônica Embarcada

  Este documento expõe o processo de criação de um programa específico para a administração de um sistema de elevador. O controle deste elevador é realizado através do microcontrolador PIC16F1827, funcionando com uma tensão de 5V. O design do programa permite aos usuários a escolha conveniente do andar almejado. Uma vez selecionado, o elevador se dirige ao andar indicado, mantendo os usuários informados sobre o andar atual durante o percurso. Quando alcança o destino, o elevador faz uma pausa de dois segundos para verificar se há novos chamados. Se houver uma nova solicitação nesse intervalo, ele se dirige ao novo andar requisitado. Caso contrário, retorna ao térreo.

  O painel de controle do elevador apresenta informações valiosas como o andar presente, a direção do movimento e os andares destinados, tudo isso exibido em uma matriz de LEDs. A operação do elevador é gerenciada pelo microcontrolador, que processa comandos por meio de uma interface serial.
