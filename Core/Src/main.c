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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "radio_driver.h"
#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "comunicator.h"
#include "util.h"
#include "utility.h" //to share FSM with Interrupts
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

int rxMargin = 10000; //commented DO NOT USE USE

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RF_FREQUENCY                                867000000 /* Hz */ // AV UPLINK
//#define RF_FREQUENCY                                869000000 /* Hz */   // AV DOWNLINK
#define TX_OUTPUT_POWER                             16        /* dBm */ //Do not exceed 10 dBm !!!  (max >15  <22 dBm for HPmode)  (LP  0 < x < 10)
#define LORA_BANDWIDTH                              0        /* Hz */ // 125kHz <=> 0
#define LORA_SPREADING_FACTOR                       8 //10
#define LORA_CODINGRATE                             3
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */
// HIGH POWER max 16 dBm
// The STM32WL low power is amplified by AMP and thus really HIGH POWER!! => Highest power output
// 2 antennas always, just power splitter as 2 antennas in Rocket!
// So for the tests use mode HP (which is the lowest power in MiaouV2)
#define LORA_PA_OUTPUT								RFO_HP // OR RFO_HP if > 15 dBm   RFO_HP = HIGH POWER (Without AmpliOp) max 16 dBm


#define RX_TIMOUT									4294967290 //3000//4294967290 //ms
#define VERBOSE 0  //  DO not put verbose if want to communicate with AV, as only 1 uart now
#define TX_ENABLED 1

//pingPongFSM_t fsm;


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void (*volatile eventReceptor)(pingPongFSM_t *const fsm);
PacketParams_t packetParams;  // TODO: this is lazy

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

uint8_t capsule_buf[256] = {0};

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
//void enterMasterTx(pingPongFSM_t *const fsm, unsigned char* msg_to_send); IN MAIN.h !!!!!!!!!!!!!!!!!!!
void enterMasterTxLen(pingPongFSM_t *const fsm, uint8_t* msg_to_send, uint16_t len);
void enterSlaveTx(pingPongFSM_t *const fsm);
void transitionRxDone(pingPongFSM_t *const fsm);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void UART2_RxCallback(uint8_t opcode, uint16_t len, uint8_t* data);




static char* msg_to_send = "Miaoux";
/*void HAL_Delay(uint32_t milliseconds) {
	//https://community.st.com/s/feed/0D50X00009XkW2MSAV
	milliseconds = milliseconds/48000000;

   (void) SysTick->CTRL;

   while (milliseconds != 0) {

      milliseconds -= (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) >> SysTick_CTRL_COUNTFLAG_Pos;

   }
}*/

uint8_t UART2_rxFragment;
uint8_t UART2_rxBufferData[1024];
util_buffer_u8_t UART2_rxBuffer;

comunicator_t com;

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
	 //char uartBuff[200];

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);//GPS NReset

  //GPIO 0 ORANGE MILIEU
  //GPIO 1 Rouge
  //GPio 8 vert


  //HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, 5);

  util_buffer_u8_init(&UART2_rxBuffer, UART2_rxBufferData, 1024);
  comunicator_init(&com, &huart2, UART2_RxCallback);
  HAL_UART_Receive_IT(&huart2, &UART2_rxFragment, 1);

  //comunicator_send(&com, opcode, length, data);
  radioInit();


 #if VERBOSE == 1
  HAL_UART_Transmit(&huart2, (uint8_t *)"Radio Init\r\n", 12, HAL_MAX_DELAY);
 #endif
  //printf("Radio init\n");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


//int32_t rnd = 0;
//10. Enable TxDone and timeout interrupts by configuring IRQ with Cfg_DioIrq().
SUBGRF_SetDioIrqParams(IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE, IRQ_RADIO_NONE);
//rnd = SUBGRF_GetRandom();

fsm.state = STATE_NULL;
fsm.subState = SSTATE_NULL;
fsm.rxTimeout = RX_TIMOUT; // 3000 ms rxTimeout
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



HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);// power supplies
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);// RESET GPS
//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);



TIM2->ARR = 32e6/3000 -1;
TIM2->CCR1 = 0.5*32e6/3000 -1 ;
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
HAL_Delay(300);
HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);





  while (1)
  {




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  eventReceptor = NULL;
	  while (eventReceptor == NULL) {
		  while(!util_buffer_u8_isempty(&UART2_rxBuffer)) {
			  comunicator_recv(&com, util_buffer_u8_get(&UART2_rxBuffer));
		  }
	  }
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




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if(htim == &htim16)
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // bip buzzer
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
// Here useful
//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


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
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_INVERTED;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
 // packetParams.Params.LoRa.InvertIQ = //LORA_IQ_NORMAL;//LORA_IQ_INVERTED
  // 4. Define the frame format with Set_PacketParams().
  SUBGRF_SetPacketParams(&packetParams);

  //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
  // RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );




  //ADDED BA MARTIN TODO MAYBE DOES NOT WORK !!! TODO
  //SUBGRF_SetRxBoosted(3000);
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
	#if VERBOSE == 1
   HAL_UART_Transmit(&huart2, (uint8_t *)"Event TX Done\r\n", 15, HAL_MAX_DELAY);
   //printf("Tx 1 packet!");
#endif
   enterMasterRx(fsm);
   fsm->subState = SSTATE_RX;
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); //vert
}


/**
  * @brief  Process the RX Done event
  * @param  fsm pointer to FSM context
  * @retval None
  */
// callback finish to receive
void eventRxDone(pingPongFSM_t *const fsm)
{
	#if VERBOSE == 1
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Done\r\n", 15, HAL_MAX_DELAY);
	#endif
	transitionRxDone(fsm);
	//BSP_LED_Off(LED_GREEN);
	//BSP_LED_Toggle(LED_RED);
	#if TX_ENABLED == 1
	enterMasterTx(fsm, (uint8_t *) msg_to_send);
	#else
	enterMasterRx(fsm);
	#endif


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // orange
}


/**
  * @brief  Process the TX Timeout event
  * @param  fsm pointer to FSM context
  * @retval None
  */
// Should not arrive, SF too big and set timout too small
void eventTxTimeout(pingPongFSM_t *const fsm)
{
	#if VERBOSE == 1
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event TX Timeout\r\n", 18, HAL_MAX_DELAY);
	#endif
	enterMasterRx(fsm);

}


/**
  * @brief  Process the RX Timeout event
  * @param  fsm pointer to FSM context
  * @retval None
  */
void eventRxTimeout(pingPongFSM_t *const fsm)
{
	#if VERBOSE == 1
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event RX Timeout\r\n", 18, HAL_MAX_DELAY);
	#endif
	#if TX_ENABLED == 1
	enterMasterTx(fsm,msg_to_send);
	#else
	enterMasterRx(fsm);
	#endif

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
	#if VERBOSE == 1
	HAL_UART_Transmit(&huart2, (uint8_t *)"Event Rx Error\r\n", 16, HAL_MAX_DELAY);
	#endif
	#if TX_ENABLED == 1
	enterMasterTx(fsm, (uint8_t *)msg_to_send);
	#else
	enterMasterRx(fsm);
	#endif
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // orange
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // rouge

}


/**
  * @brief  Entry actions for the RX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Important
void enterMasterRx(pingPongFSM_t *const fsm)
{


#if LORA_PA_OUTPUT == RFO_LP
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);// Disable power supplies

#endif
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // orange
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // rouge
	#if VERBOSE == 1
  HAL_UART_Transmit(&huart2, (uint8_t *)"Master Rx start\r\n", 17, HAL_MAX_DELAY);
#endif
  SUBGRF_SetDioIrqParams( IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR | IRQ_HEADER_ERROR,
                          IRQ_RADIO_NONE,
                          IRQ_RADIO_NONE );
  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  SUBGRF_SetPacketParams(&packetParams); // todo maybe remove
  SUBGRF_SetRx(fsm->rxTimeout << 6);
}



/**
  * @brief  Entry actions for the TX sub-state of the Master state
  * @param  fsm pointer to FSM context
  * @retval None
  */
void enterMasterTx(pingPongFSM_t *const fsm, unsigned char* msg_to_send)
{
	//if(globalTXcounter == 70){
	//	globalTXcounter = 54;
	//}
	//globalTXcounter +=1;
	unsigned int const msg_len  = strlen(msg_to_send);
	//unsigned int const sz2  = strlen(globalTXcounter);
	//char charValue[1];
	//sprintf(charValue, "%c", globalTXcounter);


	//char *concat            = (char*)malloc(7);
	//memcpy(concat         , msg_to_send  , 5 );
	//memcpy(concat + 5     ,charValue , 1 );
	//concat[5+1] = '\0';
    //msg_to_send = concat;

	//unsigned char msg_len = 7;//strlen(msg_to_send);

	//HAL_Delay(rxMargin);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); //vert

#if LORA_PA_OUTPUT == RFO_LP
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);// enable power supplies
	HAL_Delay(12);
#endif

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //bip
	HAL_TIM_Base_Start_IT(&htim16);


	#if VERBOSE == 1
	HAL_UART_Transmit(&huart2, "Master Tx start\r\n", 17, HAL_MAX_DELAY);
	#endif
	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
						  IRQ_RADIO_NONE ); // TODO irq to check if noise on channel for future projects




	SUBGRF_SetSwitch(LORA_PA_OUTPUT, RFSWITCH_TX);

	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	//https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf p.103
	// très bresom
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04)); //default 0100 0x04 can be set to 1111
	packetParams.Params.LoRa.PayloadLength = (uint8_t)msg_len;//0x4;
	SUBGRF_SetPacketParams(&packetParams);
	// 2. Write the payload data to the transmit data buffer with Write_Buffer().
	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After
	//the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	SUBGRF_SendPayload(msg_to_send/*&UART2_rxBuffer*/, msg_len, 0);//Timout
	//HAL_UART_Transmit(&huart2, &UART2_rxBuffer, 20, HAL_MAX_DELAY);
	//SUBGRF_SendPayload(strg, msg_len, 0);



}


void enterMasterTxLen(pingPongFSM_t *const fsm, uint8_t* msg_to_send, uint16_t len)
{
	//HAL_Delay(rxMargin);

#if LORA_PA_OUTPUT == RFO_LP
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);// enable power supplies
	HAL_Delay(10);
#endif
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); //vert

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim16);

	#if VERBOSE == 1
	HAL_UART_Transmit(&huart2, "Master Tx start\r\n", 17, HAL_MAX_DELAY);
	#endif
	SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
						  IRQ_RADIO_NONE,
						  IRQ_RADIO_NONE );
	SUBGRF_SetSwitch(LORA_PA_OUTPUT, RFSWITCH_TX);

	// Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	//https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf p.103
	SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04)); //default 0100 0x04 can be set to 1111
	packetParams.Params.LoRa.PayloadLength = (uint8_t)len+5;//0x4;
	SUBGRF_SetPacketParams(&packetParams);
	// 2. Write the payload data to the transmit data buffer with Write_Buffer().
	//11. Start the transmission by setting the sub-GHz radio in TX mode with Set_Tx(). After
	//the transmission is finished, the sub-GHz radio enters automatically the Standby mode.
	// TODO Capsule GS protocol
	//  | 0xFF | 0xFA | packetID | size | ...data... | CRC |
	capsule_buf[0] = 0xFF;
	capsule_buf[1] = 0xFA;
	capsule_buf[2] = 8; // = AV_TELEMETRY (CAPSULE_ID)
	capsule_buf[3] = len;
	memcpy(capsule_buf+4, msg_to_send, len); // not really optimal...
	// compute CRC
	uint8_t crc = 0;
	for (uint8_t i = 0; i < len; i++) crc += msg_to_send[i];
	capsule_buf[4+len] = crc;

	//SUBGRF_SendPayload(msg_to_send/*&UART2_rxBuffer*/, len, 0);//Timout  // Martin' base
	SUBGRF_SendPayload(capsule_buf/*&UART2_rxBuffer*/, len+5, 0);//Timout

	//HAL_UART_Transmit(&huart2, &UART2_rxBuffer, 20, HAL_MAX_DELAY);
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
  char uartBuff[100];

  // Workaround 15.3 in DS.SX1261-2.W.APP (because following RX w/ timeout sequence)
  SUBGRF_WriteRegister(0x0920, 0x00);
  SUBGRF_WriteRegister(0x0944, (SUBGRF_ReadRegister(0x0944) | 0x02));

  SUBGRF_GetPayload((uint8_t *)fsm->rxBuffer, &fsm->rxSize, 0xFF);
  SUBGRF_GetPacketStatus(&packetStatus);

  // ######################### AV-GS interface #######################
  // Transfer packet to AV board if follows Capsule protocol
  if (fsm->rxBuffer[0] == 0xFF && fsm->rxBuffer[1] == 0xFA && fsm->rxBuffer[3] == fsm->rxSize-5) { // delimiter + size check
	  uint8_t crc = 0;
	  for (uint8_t i = 4; i < fsm->rxSize-1; i++) crc += fsm->rxBuffer[i];
	  if (crc == fsm->rxBuffer[fsm->rxSize-1]) { // check CRC
		  // TODO check packetID ?
		  //if (fsm->rxBuffer[2] == 0x??)

		  // Technique du sale à cause de MSV2 du sale
		  uint16_t sizex = fsm->rxSize-5;
		  if (sizex % 2 != 0) sizex++;
		  // opcode RF is 0x65
		  comunicator_send(&com, 0x65, (uint16_t) sizex, (uint8_t*) fsm->rxBuffer+4); // remove | 0xFF | 0xFA | packetID | size |
		  // comunicator_send(&com, 0xFF, (uint16_t) fsm->rxSize-5, (uint8_t*) fsm->rxBuffer+4); // Normal technique
	  }
  }
  //HAL_UART_Transmit(&huart2, fsm->rxBuffer+4, fsm->rxSize-5, HAL_MAX_DELAY);
  //HAL_UART_Transmit(&huart2, fsm->rxBuffer, fsm->rxSize, HAL_MAX_DELAY);

  #if VERBOSE == 1
  sprintf(uartBuff, "\n\rRssiValue=%d dBm, SnrValue=%d Hz\r\n", packetStatus.Params.LoRa.RssiPkt, packetStatus.Params.LoRa.SnrPkt);
  //HAL_UART_Transmit(&huart2, uartBuff, strlen(uartBuff), HAL_MAX_DELAY);
 #endif
}

void UART2_RxCallback(uint8_t opcode, uint16_t len, uint8_t* data){
	enterMasterTxLen(&fsm,data,len);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart2) {
		util_buffer_u8_add(&UART2_rxBuffer, UART2_rxFragment);
		HAL_UART_Receive_IT(&huart2, &UART2_rxFragment, 1);
	}
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

