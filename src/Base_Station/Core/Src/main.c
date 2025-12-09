/* ============================================================================
 * Firmware da Placa 2 - LoRa_Base (ESTAÇÃO BASE) - Versão Receptor Puro
 * ============================================================================
 * Versão: 3.0
 * Autor: Gemini
 * Data: 9 de Outubro de 2025
 *
 * NOTA DA VERSÃO:
 * Este firmware foi simplificado para atuar como um receptor dedicado.
 * A lógica de envio de comandos, menus de usuário e gerenciamento de
 * timeouts foi removida.
 *
 * 1. O rádio LoRa é colocado em modo de recepção contínua na inicialização.
 * 2. O MCU entra em modo de baixo consumo (Sleep) e aguarda a interrupção
 * do rádio (IRQ_RX_DONE).
 * 3. Ao receber um pacote, ele o decodifica e exibe os dados de telemetria
 * no terminal serial do PC.
 * 4. Imediatamente após, volta ao modo de recepção para aguardar o próximo
 * pacote.
 * ========================================================================= */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32wlxx_nucleo.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "radio_driver.h"
#include "protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// --- Definições do Payload de Telemetria (deve ser IDÊNTICO ao do nó de campo) ---
typedef struct __attribute__((packed)) {
    uint32_t packet_id;
    int32_t  latitude_raw;
    int32_t  longitude_raw;
    int32_t  altitude_raw;
    uint16_t voltage_mv;
    int8_t   radio_temp_c;
    uint8_t  sats_and_fix;
} LoRaPayload_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_PAYLOAD_SIZE sizeof(LoRaPayload_t)

// --- Parâmetros LoRa (IDÊNTICOS aos da Placa 1) ---
#define RF_FREQUENCY                                915000000 // Hz
#define TX_OUTPUT_POWER                             22        // dBm (Não usado para TX, mas mantido por consistência)
#define LORA_BANDWIDTH                              0         // 0: 125 kHz
#define LORA_SPREADING_FACTOR                       10        // SF10
#define LORA_CODINGRATE                             4         // 4: 4/8
#define LORA_PREAMBLE_LENGTH                        8
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t lora_rx_buffer[TELEMETRY_PAYLOAD_SIZE]; // Buffer para receber telemetria

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ProcessTelemetryResponse(uint8_t* buffer, uint8_t size, PacketStatus_t packet_info);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
// Redireciona a saída do printf para a USART2 (nossa porta de debug para o PC)
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SUBGHZ_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);

  printf("\r\n--- LoRa_Base (ESTACAO BASE) ---\r\n");
  printf("Modo: Receptor Puro\r\n");

  Radio_Init();
  //printf("Radio LoRa inicializado. Aguardando telemetria...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // O processamento acontece 100% no callback de interrupção do rádio.
    // O processador pode entrar em modo sleep para economizar energia.
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  /* USER CODE END WHILE */
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Decodifica e exibe o pacote de telemetria recebido.
  */
void ProcessTelemetryResponse(uint8_t* buffer, uint8_t size, PacketStatus_t packet_info)
{
    if (size != TELEMETRY_PAYLOAD_SIZE) {
        //printf("ERRO: Pacote com tamanho inesperado: %d (esperado: %d)\r\n", size, TELEMETRY_PAYLOAD_SIZE);
        return;
    }

    LoRaPayload_t telemetry_data;
    memcpy(&telemetry_data, buffer, sizeof(LoRaPayload_t));

    float latitude = telemetry_data.latitude_raw / 10000000.0f;
    float longitude = telemetry_data.longitude_raw / 10000000.0f;
    float altitude_m = telemetry_data.altitude_raw / 1000.0f;

    bool gpsFixOK = (telemetry_data.sats_and_fix >> 7) & 0x01;
    uint8_t satCount = telemetry_data.sats_and_fix & 0x7F;

    printf("{\"ID\": %lu, \"Lat\": %.7f, \"Lon\": %.7f, \"Alt\": %.2f, \"Volt\": %u, \"Temp\": %d, \"GPS\": \"%s\", \"Sats\": %u, \"RSSI\": %d, \"SNR\": %d}\r\n", telemetry_data.packet_id, latitude, longitude, altitude_m, telemetry_data.voltage_mv, telemetry_data.radio_temp_c, gpsFixOK ? "FIX OK" : "NO FIX", satCount, packet_info.Params.LoRa.RssiPkt, packet_info.Params.LoRa.SnrPkt);
}

/**
  * @brief Inicializa o rádio Sub-GHz e o coloca em modo de recepção contínua.
  */
void Radio_Init(void)
{
    SUBGRF_Init(RadioOnDioIrq);

    SUBGRF_SetStandby(STDBY_RC);
    SUBGRF_SetPacketType(PACKET_TYPE_LORA);
    SUBGRF_SetRfFrequency(RF_FREQUENCY);
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER); // Mesmo não transmitindo, é bom configurar.

    ModulationParams_t modulationParams;
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SPREADING_FACTOR;
    modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
    modulationParams.Params.LoRa.CodingRate = LORA_CODINGRATE;
    modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
    SUBGRF_SetModulationParams(&modulationParams);

    // Configuração para receber pacotes de telemetria
    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = TELEMETRY_PAYLOAD_SIZE; // <-- Ponto chave!
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    // Configura interrupções apenas para eventos de recepção
    SUBGRF_SetDioIrqParams(IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                           IRQ_RX_DONE | IRQ_CRC_ERROR | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE, IRQ_RADIO_NONE);

    // Inicia em modo de recepção contínua
    SUBGRF_SetRx(0);
}

/**
  * @brief Callback de interrupção do rádio. Trata apenas eventos de recepção.
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    switch (radioIrq)
    {
        case IRQ_RX_DONE:
            {
                uint8_t received_size = 0;
                PacketStatus_t packetStatus;

                BSP_LED_Toggle(LED_GREEN);

                SUBGRF_GetPayload(lora_rx_buffer, &received_size, TELEMETRY_PAYLOAD_SIZE);
                SUBGRF_GetPacketStatus(&packetStatus);

                //printf("\r\nPacote LoRa Recebido! RSSI: %d dBm, SNR: %d\r\n",
                       //packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);

                ProcessTelemetryResponse(lora_rx_buffer, received_size, packetStatus);

                // Volta para o modo de recepção para aguardar o próximo pacote
                SUBGRF_SetRx(0);
            }
            break;

        case IRQ_CRC_ERROR:
            //printf("WARN: Erro de CRC no pacote LoRa.\r\n");
            SUBGRF_SetRx(0); // Volta a escutar
            break;

        case IRQ_RX_TX_TIMEOUT:
            //printf("WARN: Timeout de recepção LoRa.\r\n");
            SUBGRF_SetRx(0); // Volta a escutar
            break;

        default:
            break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
