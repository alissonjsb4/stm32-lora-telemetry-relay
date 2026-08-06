# stm32-lora-telemetry-relay

Enlace de telemetria de longo alcance para radiossonda com duas placas
NUCLEO-WL55JC1: o nó de campo captura o stream serial da sonda, valida os pacotes
e os retransmite por LoRa; a estação-base recebe, decodifica e entrega o dado
formatado ao PC pela serial. Projeto em dupla.

## Uso

Hardware: 2× STM32 NUCLEO-WL55JC1, antenas de 915 MHz e uma fonte serial a
19200 baud (radiossonda ou equivalente).

1. Importar `src/Field_Node/` e `src/Base_Station/` no STM32CubeIDE e compilar.
2. Gravar cada firmware na sua placa.
3. Abrir terminal serial na estação-base a 115200 baud para ver a telemetria
   decodificada (ID, posição, altitude, tensão, temperatura, status de GPS).

## Funcionamento

- Nó de campo: recepção UART por DMA em modo circular, sem bloquear a CPU; uma
  máquina de estados localiza o pacote por `SYNC_WORD` e valida o checksum antes
  de transmitir por LoRa.
- Estação-base: orientada a interrupção, em recepção contínua; decodifica o
  payload binário e formata para o PC via USART2.
- Definição do pacote compartilhada em `src/common/protocol.h`.

| Parâmetro LoRa | Valor |
|---|---|
| Frequência | 915,0 MHz |
| Spreading factor | 10 |
| Largura de banda | 125 kHz |
| Coding rate | 4/8 |
| Potência de transmissão | 22 dBm |
| Header | implícito, tamanho fixo |

## Resultados

Enlace ponta a ponta validado em bancada: pacotes da radiossonda aprovados no
checksum, retransmitidos e decodificados na base com RSSI −50 dBm e SNR 9 na
configuração de teste. Relatório técnico completo em `docs/`.

## Notas

- SF 10 com CR 4/8 privilegia alcance e robustez sobre taxa de dados — payload
  curto de telemetria não precisa de banda.
- O modo de sleep do rádio no nó de campo ficou como pendência para operação a
  bateria.

## Estrutura

    src/Field_Node/      transmissor: captura UART/DMA, FSM de validação, LoRa TX
    src/Base_Station/    receptor: LoRa RX por interrupção, decodificação, USART2
    src/common/          protocol.h compartilhado entre os nós
    docs/                relatório técnico (PDF)

## Autores

Alisson Jaime Sales Barros e Danilo Mota Alencar Filho.
