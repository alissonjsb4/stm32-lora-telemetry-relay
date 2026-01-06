/* ============================================================================
 * Firmware da Placa 1 - LoRa_RS41 (NÓ DE CAMPO) - Versão Transmit-on-Event
 * ============================================================================
 * Versão: 3.2 - Teste Definitivo de Low Power
 * Autor: Gemini
 * Data: 9 de Outubro de 2025
 *
 * NOTA DA VERSÃO:
 * Código modificado para o teste definitivo de "Wake-on-Event".
 * 1. O LED verde (LD2) começa forçadamente APAGADO.
 * 2. O loop while(1) contém apenas a instrução para dormir (HAL_PWR_EnterSLEEPMode).
 * 3. O LED pisca uma única vez (dentro de `ProcessByte`) a cada pacote
 * serial recebido com sucesso, provando que o MCU acorda apenas por interrupção.
 * ========================================================================= */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"
#include "stm32wlxx_nucleo.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "radio_driver.h"
#include "protocol.h"

/* USER CODE BEGIN 0 */
// Redireciona a saída do printf para a USART2 (nossa porta de debug para o PC)
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
  return len;
}
/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- Definições do Payload de Telemetria ---

// --- Parser da Radiosonda ---
const uint8_t SYNC_WORD = 0xAA;


enum ParserState { AWAITING_SYNC, RECEIVING_PAYLOAD, AWAITING_CHECKSUM };

#define RADIOSONDE_UART_BUFFER_SIZE 256

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// --- Variáveis do Parser da Radiosonda ---
uint8_t radiosonde_rx_buffer[RADIOSONDE_UART_BUFFER_SIZE];
uint16_t read_pos = 0; // Posição onde já lemos os dados
uint8_t payloadBuffer[TELEMETRY_PAYLOAD_SIZE];
int byteCounter = 0;
enum ParserState currentState = AWAITING_SYNC;

// --- Variáveis da Comunicação LoRa ---
LoRaPayload_t latest_telemetry; // Última telemetria válida da radiosonda
volatile bool telemetry_available = false; // Flag indicando se há telemetria válida
volatile bool lora_tx_busy = false; // Flag para controlar transmissão LoRa
volatile bool packet_processed_this_dma_cycle = false;

// Array de lookup para Bandwidth
const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Radio_Init(void);
void ProcessByte(uint8_t receivedByte);
void TransmitTelemetry(void);
uint8_t calculate_checksum(uint8_t* data, int length);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void Start_DMA_Reception_With_Interrupts(void);
void Reset_UART_And_Parser_State(void); // <-- ADD THIS LINE
/* USER CODE END PFP */

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
  MX_USART1_UART_Init();  // UART da Radiosonda
  MX_USART2_UART_Init();  // UART de Debug para o PC
  MX_SUBGHZ_Init();

  /* USER CODE BEGIN 2 */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN); // MUDANÇA 1: Garante que o LED comece apagado.

  printf("\r\n--- LoRa_RS41 (NÓ DE CAMPO) ---\r\n");
  printf("Modo: Teste Definitivo de Low Power\r\n");

  Radio_Init();
  printf("Radio LoRa inicializado.\r\n");

  Start_DMA_Reception_With_Interrupts();
  printf("Aguardando pacotes...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // MUDANÇA 2: O loop principal agora SÓ coloca o MCU para dormir.
    // Nenhum LED pisca aqui.
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

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief Inicia a recepção contínua de dados da UART1 usando DMA e
  * habilita a interrupção de linha ociosa (Idle Line).
  */
void Start_DMA_Reception_With_Interrupts(void)
{
  HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, radiosonde_rx_buffer, RADIOSONDE_UART_BUFFER_SIZE);
  if (status != HAL_OK)
  {
    printf("Erro ao iniciar a recepcao DMA com Idle Line!\r\n");
    Error_Handler();
  }
  // Impede que a interrupção de Half Transfer (HT) nos incomode.
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
}

/**
  * @brief Callback de eventos de recepção da UART (incluindo DMA Idle).
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    // Reset the flag at the beginning of every new data chunk event
    packet_processed_this_dma_cycle = false;

    uint16_t write_pos = RADIOSONDE_UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    // Process all the bytes that just arrived
    while (read_pos != write_pos)
    {
      ProcessByte(radiosonde_rx_buffer[read_pos]);
      read_pos = (read_pos + 1) % RADIOSONDE_UART_BUFFER_SIZE;
    }

    // --- NEW RECOVERY LOGIC ---
    // If we processed the whole chunk and didn't find a single valid packet,
    // it's highly likely the buffer contains garbage from a device reset.
    if (!packet_processed_this_dma_cycle && currentState == AWAITING_SYNC)
    {
        printf("WARN: Invalid data chunk detected. Resetting parser and DMA...\r\n");
        // Use the same reset function we created before
        Reset_UART_And_Parser_State();
    }
  }
}

/**
  * @brief Processa cada byte individual da radiosonda usando uma máquina de estados.
  */
void ProcessByte(uint8_t receivedByte)
{
  switch (currentState)
  {
    case AWAITING_SYNC:
      if (receivedByte == SYNC_WORD) {
        byteCounter = 0;
        currentState = RECEIVING_PAYLOAD;
      }
      break;

    case RECEIVING_PAYLOAD:
      if (byteCounter < TELEMETRY_PAYLOAD_SIZE) {
        payloadBuffer[byteCounter++] = receivedByte;
      }
      if (byteCounter >= TELEMETRY_PAYLOAD_SIZE) {
        currentState = AWAITING_CHECKSUM;
      }
      break;

    case AWAITING_CHECKSUM:
      {
        uint8_t receivedChecksum = receivedByte;
        uint8_t calculatedChecksum = calculate_checksum(payloadBuffer, TELEMETRY_PAYLOAD_SIZE);

        if (receivedChecksum == calculatedChecksum) {
          memcpy(&latest_telemetry, payloadBuffer, TELEMETRY_PAYLOAD_SIZE);
          telemetry_available = true;
          packet_processed_this_dma_cycle = true;

          printf("Pacote da radiosonda validado. Transmitindo via LoRa...\r\n");
          BSP_LED_On(LED_GREEN);
          TransmitTelemetry();

        } else {
          printf("Falha no Checksum da radiosonda! Esperado: 0x%02X, Recebido: 0x%02X\r\n",
                 calculatedChecksum, receivedChecksum);
        }
        currentState = AWAITING_SYNC;
        BSP_LED_Off(LED_GREEN);
      }
      break;
  }
}

/**
  * @brief Prepara e envia o pacote de telemetria mais recente via LoRa.
  */
void TransmitTelemetry(void)
{
    if (lora_tx_busy) {
        printf("WARN: Radio ocupado, transmissao descartada.\r\n");
        return;
    }
    if (!telemetry_available) {
        printf("WARN: Nenhuma telemetria disponível para envio.\r\n");
        return;
    }

    lora_tx_busy = true;
    printf("Transmitindo pacote LoRa ID: %lu\r\n", latest_telemetry.packet_id);

    uint8_t tx_buffer[TELEMETRY_PAYLOAD_SIZE];
    memcpy(tx_buffer, &latest_telemetry, TELEMETRY_PAYLOAD_SIZE);

    PacketParams_t packetParams;
    packetParams.PacketType = PACKET_TYPE_LORA;
    packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
    packetParams.Params.LoRa.HeaderType = LORA_PACKET_FIXED_LENGTH;
    packetParams.Params.LoRa.PayloadLength = TELEMETRY_PAYLOAD_SIZE;
    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
    SUBGRF_SetPacketParams(&packetParams);

    SUBGRF_SendPayload(tx_buffer, TELEMETRY_PAYLOAD_SIZE, 0);
}

/**
  * @brief Inicializa o rádio Sub-GHz com os parâmetros LoRa definidos.
  */
void Radio_Init(void)
{
    SUBGRF_Init(RadioOnDioIrq);

    SUBGRF_SetStandby(STDBY_RC);
    SUBGRF_SetPacketType(PACKET_TYPE_LORA);
    SUBGRF_SetRfFrequency(RF_FREQUENCY);
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);

    ModulationParams_t modulationParams;
    modulationParams.PacketType = PACKET_TYPE_LORA;
    modulationParams.Params.LoRa.SpreadingFactor = LORA_SPREADING_FACTOR;
    modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
    modulationParams.Params.LoRa.CodingRate = LORA_CODINGRATE;
    modulationParams.Params.LoRa.LowDatarateOptimize = LORA_LOWDR_OPT;
    SUBGRF_SetModulationParams(&modulationParams);

    // Habilita apenas a interrupção de transmissão concluída e timeout
    SUBGRF_SetDioIrqParams(IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
                           IRQ_RADIO_NONE, IRQ_RADIO_NONE);
}

/**
  * @brief Callback de interrupção do rádio. Agora trata apenas eventos de transmissão.
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    switch (radioIrq)
    {
        case IRQ_TX_DONE:
            printf("LoRa TX Done.\r\n");
            lora_tx_busy = false;

            // Transmissão concluída. Coloca o rádio em standby para economizar energia.
            SUBGRF_SetStandby(STDBY_RC);

            // MODIFICATION: Reset the system to await the next packet
            Reset_UART_And_Parser_State();
            break;

        case IRQ_RX_TX_TIMEOUT:
            printf("WARN: LoRa TX Timeout.\r\n");
            lora_tx_busy = false;
            SUBGRF_SetStandby(STDBY_RC);

            // Also reset on timeout to be safe
            Reset_UART_And_Parser_State();
            break;

        default:
            // Outras interrupções (como RX_DONE) são ignoradas
            break;
    }
}

void Reset_UART_And_Parser_State(void)
{
  // 1. Stop the current DMA reception to safely modify buffers
  HAL_UART_DMAStop(&huart1);

  // 2. Clear out the buffers to prevent processing old/partial data
  memset(radiosonde_rx_buffer, 0, RADIOSONDE_UART_BUFFER_SIZE);
  memset(payloadBuffer, 0, TELEMETRY_PAYLOAD_SIZE);

  // 3. Reset the software parser state variables
  read_pos = 0;
  byteCounter = 0;
  currentState = AWAITING_SYNC;

  // 4. Restart DMA reception with Idle Line interrupt for the next packet
  Start_DMA_Reception_With_Interrupts();

  printf("SYSTEM RESET: Ready for next radiosonde packet.\r\n");
}

/**
  * @brief Calcula o checksum (XOR) de um buffer de dados.
  */
uint8_t calculate_checksum(uint8_t* data, int length)
{
    uint8_t checksum = 0;
    for (int i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
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
