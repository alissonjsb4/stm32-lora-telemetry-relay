#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>


// Enum para os 4 tipos de AÇÃO do comando
typedef enum {
    CMD_TYPE_READ    = 0x01, // Ler o valor de um parâmetro da RS41
    CMD_TYPE_WRITE   = 0x02, // Escrever um novo valor em um parâmetro da RS41
    CMD_TYPE_EXECUTE = 0x03, // Mandar a RS41 executar uma ação (ex: reset)
    CMD_TYPE_REQUEST = 0x04  // Solicitar um tipo específico de resposta (ex: telemetria)
} CommandType_t;


// Enum para os ALVOS do comando (parâmetros e ações)
typedef enum {
    // Parâmetros que podem ser lidos/escritos
    PARAM_DEEP_SLEEP_INTERVAL = 0x10, // uint32_t
    PARAM_TX_INTERVAL         = 0x11, // uint16_t

    // Ações que podem ser executadas
    ACTION_RESET_MCU          = 0x80,

    // Requisições que podem ser feitas
    REQUEST_TELEMETRY_PACKET  = 0xA0
} ParameterID_t;


#define MAX_PAYLOAD_DATA_SIZE 16 // Define um tamanho máximo para o campo de dados


typedef struct __attribute__((packed)) {
    uint32_t      sequence_number;      // Número de sequência para rastreamento
    CommandType_t command_type;         // Ação a ser tomada (READ, WRITE, EXECUTE, REQUEST)
    ParameterID_t parameter_id;         // Alvo da ação (qual parâmetro ou ação)
    uint8_t       payload_len;          // Comprimento dos dados no campo 'payload_data'
    uint8_t       payload_data;         // Buffer para os dados
} CommandPacket_t;


// --- Parâmetros LoRa (IDÊNTICOS aos da Placa 1) ---
typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    int32_t  latitude_raw;
    int32_t  longitude_raw;
    int32_t  altitude_raw;
    uint16_t voltage_mv;
    int8_t   radio_temp_c;
    uint8_t  sats_and_fix;
} LoRaPayload_t;

#define TELEMETRY_PAYLOAD_SIZE sizeof(LoRaPayload_t)


// --- Parâmetros LoRa ---
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             22        // dBm
#define LORA_BANDWIDTH                              0         // 0: 125 kHz
#define LORA_SPREADING_FACTOR                       12        // SF10
#define LORA_CODINGRATE                             4         // 4: 4/8
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_LOWDR_OPT                              0x01      // Default: 0x00. For SF>10, 0x01

#endif
