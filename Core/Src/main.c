/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "subghz.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*typedef enum
{
  STATE_NULL,
  STATE_MASTER,
  STATE_SLAVE
} state_t;

typedef enum
{
  SSTATE_NULL,
  SSTATE_RX,
  SSTATE_TX
} substate_t;

typedef struct
{
  state_t state;
  substate_t subState;
  uint32_t rxTimeout;
  uint32_t rxMargin;
  //uint32_t randomDelay;
  char rxBuffer[RX_BUFFER_SIZE];
  uint8_t rxSize;
} pingPongFSM_t;*/

int rxMargin = 1000;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RF_FREQUENCY                                868000000 /* Hz */
#define TX_OUTPUT_POWER                             1        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
#define LORA_PA_OUTPUT								RFO_LP // OR RFO_HP if > 15 dBm
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;  // TODO: this is lazy

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void radioInit(void);
void RadioOnDioIrq(RadioIrqMasks_t radioIrq);
void eventTxDone(pingPongFSM_t *const fsm);
void eventRxDone(pingPongFSM_t *const fsm);
void eventTxTimeout(pingPongFSM_t *const fsm);
void eventRxTimeout(pingPongFSM_t *const fsm);
void eventRxError(pingPongFSM_t *const fsm);
void enterMasterRx(pingPongFSM_t *const fsm);
void enterSlaveRx(pingPongFSM_t *const fsm);
void enterMasterTx(pingPongFSM_t *const fsm);
void enterSlaveTx(pingPongFSM_t *const fsm);
void transitionRxDone(pingPongFSM_t *const fsm);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


/*void HAL_Delay(uint32_t milliseconds) {
	//https://community.st.com/s/feed/0D50X00009XkW2MSAV
	milliseconds = milliseconds/48000000;

   (void) SysTick->CTRL;

   while (milliseconds != 0) {

      milliseconds -= (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> SysTick_CTRL_COUNTFLAG_Pos;

   }
}*/

uint8_t UART2_rxBuffer[50] = {0};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	 //pingPongFSM_t fsm;
	 char uartBuff[200];

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SUBGHZ_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);

  //GPIO 0 ORANGE MILIEU
  //GPIO 1 Rouge
  //GPio 8 vert


  //HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 5);
	HAL_UART_Receive_DMA(&huart2, UART2_rxBuffer, 5);


  radioInit();
  HAL_UART_Transmit(&huart2, (uint8_t *)"Radio Init\r\n", 12, HAL_MAX_DELAY);

//  printf("Radio init\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


int32_t rnd = 0;
//10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
rnd = SUBGRF_GetRandom();

fsm.state = STATE_NULL;
fsm.subState = SSTATE_NULL;
fsm.rxTimeout = 3000; // 3000 ms
//fsm.rxMargin = 3000000;   // 200 ms
//fsm.randomDelay = rnd >> 22; // [0, 1023] ms
//sprintf(uartBuff, "rand=%u\r\n", fsm.randomDelay);
//HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff, strlen(uartBuff), HAL_MAX_DELAY);

//HAL_Delay(fsm.randomDelay);
SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
						IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR,
						IRQ_RADIO_NONE, IRQ_RADIO_NONE );
//SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
SUBGRF_SetRx(fsm.rxTimeout << 6);
//fsm.state = STATE_MASTER;
//fsm.subState = SSTATE_RX;



//radio_TX("salut",5);

//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
//SUBGRF_SetTxInfinitePreamble();
//SUBGRF_SetTxContinuousWave();





  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  eventReceptor = NULL;
	  while (eventReceptor == NULL);
	  eventReceptor(&fsm);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
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
  * @brief  Initialize the Sub-GHz radio and dependent hardware.
  * @retval None
  */
void radioInit(void)
{
  // Initialize the hardware (SPI bus, TCXO control, RF switch)
  SUBGRF_Init(RadioOnDioIrq);

  // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
  // "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
  SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
  SUBGRF_SetRegulatorMode();
  //WORK TODO desactivate the PA clamp
  //SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x0E)); //default 0100 0x04 can be set to 1111

  // Use the whole 256-byte buffer for both TX and RX
  //1, Define the location of the transmit payload data in the data buffer, with Set_BufferBaseAddress().
  SUBGRF_SetBufferBaseAddress(0x00, 0x00);
  //6. Define the RF frequency with Set_RfFrequency().
  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  //7. Define the PA configuration with Set_PaConfig().
  //8. Define the PA output power and ramping with Set_TxParams().
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  SUBGRF_SetStopRxTimerOnPreambleDetect(false);
  //3. Select the packet type (generic or LoRa) with Set_PacketType().
  SUBGRF_SetPacketType(PACKET_TYPE_LORA);
  //5. Define synchronization word in the associated packet type SUBGHZ_xSYNCR(n) with Write_Register().
  SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
  SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

  ModulationParams_t modulationParams;
  modulationParams.PacketType = PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
  modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
  modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
  modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
  //9. Define the modulation parameters with Set_ModulationParams().
  SUBGRF_SetModulationParams(&modulationParams);

  packetParams.PacketType = PACKET_TYPE_LORA;
  packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;//LORA_IQ_INVERTED
  // 4. Define the frame format with Set_PacketParams().
  SUBGRF_SetPacketParams(&packetParams);

  //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
  // RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}


/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
  switch (radioIrq)
  {
    case IRQ_TX_DONE:
      eventReceptor = eventTxDone;
      break;
    case IRQ_RX_DONE:
      eventReceptor = eventRxDone;
      break;
    case IRQ_RX_TX_TIMEOUT:
      if (SUBGRF_GetOperatingMode() == MODE_TX)
      {
        eventReceptor = eventTxTimeout;
      }
      else if (SUBGRF_GetOperatingMode() == MODE_RX)
      {
        eventReceptor = eventRxTimeout;
      }
      break;
    case IRQ_CRC_ERROR:
      eventReceptor = eventRxError;
      break;
    default:
      break;
  }
}


/**
  * @brief  Process the TX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventTxDone(pingPongFSM_t *const fsm)
{
   HAL_UART_Transmit(&huart2, (uint8_t *)"Event TX Done\r\n", 15, HAL_MAX_DELAY);
   enterMasterRx(fsm);
   fsm->subState = SSTATE_RX;
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); //vert
}


/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxDone(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Done\r\n", 15, HAL_MAX_DELAY);
	transitionRxDone(fsm);
	//BSP_LED_Off(LED_GREEN);
	//BSP_LED_Toggle(LED_RED);
	enterMasterTx(fsm);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // orange
}


/**
  * @brief  Process the TX Timeout event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventTxTimeout(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event TX Timeout\r\n", 18, HAL_MAX_DELAY);
    enterMasterRx(fsm);

}


/**
  * @brief  Process the RX Timeout event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxTimeout(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Timeout\r\n", 18, HAL_MAX_DELAY);
    enterMasterTx(fsm);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // orange
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // rouge

}


/**
  * @brief  Process the RX Error event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxError(pingPongFSM_t *const fsm)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event Rx Error\r\n", 16, HAL_MAX_DELAY);
    enterMasterTx(fsm);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // orange
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // rouge

}


/**
  * @brief  Entry actions for the RX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterMasterRx(pingPongFSM_t *const fsm)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // orange
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // rouge

  HAL_UART_Transmit(&huart2, (uint8_t *)"Master Rx start\r\n", 17, HAL_MAX_DELAY);
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams);
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}



/**
  * @brief  Entry actions for the TX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterMasterTx(pingPongFSM_t *const fsm)
{
	unsigned char* strg = "Miaou\n";
	unsigned char msg_len = 10;


	HAL_Delay(rxMargin);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); //vert


	//HAL_UART_Transmit(&huart2, "...PING\r\n", 9, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, "Master Tx start\r\n", 17, HAL_MAX_DELAY);
	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
						  IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(LORA_PA_OUTPUT, RFSWITCH_TX);
	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	//https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf p.103
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04)); //default 0100 0x04 can be set to 1111
	packetParams.Params.LoRa.PayloadLength = (uint8_t)msg_len;//0x4;
	SUBGRF_SetPacketParams(&packetParams);
	// 2. Write the payload data to the transmit data buffer with Write_Buffer().
	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After
	//the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	SUBGRF_SendPayload(strg/*&UART2_rxBuffer*/, msg_len, 0);//Timout
	HAL_UART_Transmit(&huart2, (uint8_t *)"MSG TX:\r\n", 9, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, &UART2_rxBuffer, 20, HAL_MAX_DELAY);
	//SUBGRF_SendPayload(strg, msg_len, 0);

}




/**
  * @brief  Transition actions executed on every RX Done event (helper function)
  * @param  fsm pointer to FSM context
  * @retval None
  */
void transitionRxDone(pingPongFSM_t *const fsm)
{
  PacketStatus_t packetStatus;
  int32_t cfo;
  char uartBuff[50];

  // Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
  SUBGRF_WriteRegister(0x0920, 0x00);
  SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

  SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
  SUBGRF_GetPacketStatus(&packetStatus);
  HAL_UART_Transmit(&huart2, fsm->rxBuffer, fsm->rxSize, HAL_MAX_DELAY);
  sprintf(uartBuff, "RssiValue=%d dBm, SnrValue=%d Hz\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
  HAL_UART_Transmit(&huart2, uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  HAL_UART_Transmit(&huart2, &UART2_rxBuffer, 10, 500);
  //HAL_UART_Receive_IT(&huart2, &UART2_rxBuffer, 5);
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // rouge
  //HAL_UART_Transmit(&huart2, (uint8_t)"Miaou", 5, 500);
  HAL_UART_Receive_DMA(&huart2, &UART2_rxBuffer, 10);
}



/*
int __io_putchar(int ch) {
    ITM_SendChar(ch);
    return ch;
}
*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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

