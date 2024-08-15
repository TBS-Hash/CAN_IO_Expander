/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYS_FREQ 48000000

typedef enum __AlternateModeTypeDef {
  UART_ALT_MODE,
  TIM_ALT_MODE
} AlternateModeTypeDef;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef CAN_TxHeader; //This will create the Transceiver Header which contains Header information
uint8_t CAN_TxData[8];            //This stores the data which is going to be transmitted over CAN
uint32_t CAN_TxMailbox;           
uint8_t CAN_response_data[250];
int16_t CAN_response_length = 0;

CAN_RxHeaderTypeDef CAN_RxHeader; //This will create the Receiver Header which contains Header information
uint8_t CAN_RxData[8];            //Stores the data which is going to be received         
uint32_t CAN_RxFifo;              
uint8_t UART1_RxData[200];
uint8_t UART2_RxData[200];

TIM_HandleTypeDef* activeTimer;
uint16_t timerChannel = 0;
uint16_t timerPrescaler = 0;
uint16_t timerFreq = 10000;
uint16_t timerPeriod = 65535;
uint16_t channelPulse = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */
GPIO_TypeDef* getGPIOPort(uint8_t CAN_pin);
uint16_t getGPIOPin(uint8_t CAN_pin);
void pinMode(GPIO_TypeDef* port, uint16_t pin, uint8_t dir, uint8_t pull);
TIM_HandleTypeDef* getTimer(uint8_t timer);
void sendCANResponse(CAN_HandleTypeDef* hcan, uint8_t command, uint8_t* CAN_response, int16_t CAN_response_length, uint32_t Response_ID);
void setChannelPulse(TIM_HandleTypeDef* timer, uint8_t channel, uint16_t pulse);
void setPinAlternateMode(uint16_t input_pin, uint16_t pin_function);
void SetFrequencyPWM(uint8_t timer, uint16_t frequency);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t EXTI_callback_pin = 0;
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
  {
    GPIO_Pin = EXTI_callback_pin;
  }


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  
  uint16_t EXTI_last_cycle = 0;
  uint8_t EXTI_timer = 1; //A
  uint8_t EXTI_channel = 1; //B
  uint8_t EXTI_duty_cycle = 100; //C
  uint16_t EXTI_Pins[] = {GPIO_PIN_1, GPIO_PIN_15, GPIO_PIN_0}; //D
  uint8_t EXTI_sensor_function[] = {1,2,3};  //E
  uint8_t EXTI_enable_disable = 1; // F0


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
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  //CAN Filter
  CAN_FilterTypeDef CAN_FilterType;
  uint8_t REMOTE_FRAME = 0;        //If = 1 the frame should be a remote frame. If = 0 the frame will be either remote or data frame
  uint8_t EXTID = 1;               //If = 0 the frame should be a frame with standard ID. If = 1 the frame should be a frame with extended ID
  uint32_t Input_ID = 0x700;
  uint32_t Response_ID = Input_ID;
  uint32_t MASK = 0x1FFFFFFF;   //The Mask defines which Bits are to be compared
  uint32_t Filter_ID = ((Input_ID << 3) | (REMOTE_FRAME << 1) | (EXTID << 2));   //Shifted by 3 because the RTR and IDE still needs to fit
  uint32_t Filter_Mask = ((MASK << 3) | (REMOTE_FRAME << 1) | (EXTID << 2));
  CAN_FilterType.FilterActivation = ENABLE;             //Enables the Filter                 
  CAN_FilterType.FilterBank = 0;                        //Specifies the used Filter Bank
  CAN_FilterType.FilterFIFOAssignment = CAN_RX_FIFO0;   //FIFO0 will be used for receiving Data 
  CAN_FilterType.FilterIdHigh = (Filter_ID >> 16);      //The filter ID is shifted because the Bits from [31:16] are required
  CAN_FilterType.FilterIdLow =  Filter_ID;              //Lower 18 Bits of the Filter ID
  CAN_FilterType.FilterMaskIdHigh = (Filter_Mask >> 16);//Higher 11 Bits of the Mask ID. Same as FilterIdHigh
  CAN_FilterType.FilterMaskIdLow = Filter_Mask;         //Lower 18 Bits of the Mask ID
  CAN_FilterType.FilterMode = CAN_FILTERMODE_IDMASK;    //Sets the Filters into Mask mode    
  CAN_FilterType.FilterScale = CAN_FILTERSCALE_32BIT;   //Sets the Filter Register to 32 Bits
  CAN_FilterType.SlaveStartFilterBank = 14;
  
  uint16_t count = 0;
  CAN_TxHeader.IDE = CAN_ID_EXT;    //Defines the ID as Extended
  CAN_TxHeader.ExtId = 0x700;       //Identifier // lowest numerical value have the highest priority 
  CAN_TxHeader.RTR = CAN_RTR_DATA;  //Indicates Data frame
  CAN_TxHeader.DLC = 0x08;          //The length of of the Data Field
  
  HAL_CAN_Init(&hcan);
  HAL_CAN_ConfigFilter(&hcan, &CAN_FilterType);
  HAL_CAN_Start(&hcan);

  //UART - CAN
  HAL_UART_Init(&huart1);   
  HAL_UART_Receive_DMA(&huart1, UART1_RxData, 200);

  //UART - Scanner
  HAL_UART_Init(&huart2);
  HAL_UART_Receive_DMA(&huart2, UART2_RxData, 200);

  //PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  setChannelPulse(&htim1,1,0);
  setChannelPulse(&htim1,2,0);
  setChannelPulse(&htim1,3,0);
  setChannelPulse(&htim3,1,0);
  setChannelPulse(&htim3,2,0);
  setChannelPulse(&htim3,3,0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
////////////////// EXTI CODE ///////////////////////////////
  if (EXTI_enable_disable == 1){ //Check if the EXTI system is enabled

  uint8_t EXTI_channel_pulse = (EXTI_duty_cycle * timerPeriod)/100; // Calculate the duty cycle
  activeTimer = getTimer(EXTI_timer);  // Get the correct timer
  CAN_TxData[1] = 0x6B;     // Set the response verification
  CAN_TxHeader.DLC = 0x2;   // Set the Data field length to 2 bytes

    for (uint8_t i = 0; i < 3; i++) // Loop through the EXTI_Pins
    {
      if((EXTI_Pins[i] == EXTI_callback_pin) && (EXTI_callback_pin != EXTI_last_cycle)) // If the pin from last cycle is not the pin and the set pin
      {
        SetFrequencyPWM(EXTI_timer, timerFreq); // Set frequency to standard 10 kHz
        EXTI_last_cycle = EXTI_callback_pin;

        switch (EXTI_sensor_function[i])
        {
        case 1: // the input sensor
          setChannelPulse(activeTimer, EXTI_channel, EXTI_channel_pulse); // set to specified duty cycle
          break;

        case 2: // the middle sensor
          setChannelPulse(activeTimer, EXTI_channel, 0); // pause the belt

          while(1){ // wait for message back
            HAL_CAN_GetRxMessage(&hcan, CAN_RxFifo, &CAN_RxHeader, CAN_RxData);

            if(CAN_RxData[0] == 0xF2){
              SetFrequencyPWM(EXTI_timer, timerFreq);
              setChannelPulse(activeTimer, EXTI_channel, EXTI_channel_pulse); // turn on belt motors 
              CAN_TxData[0] = 0xF2;
              break;
            }
          }
          HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, CAN_TxData, &CAN_TxMailbox); // Return for acknowledgment
          break;

        case 3: // the output sensor
          while(1){ // wait for message
            HAL_CAN_GetRxMessage(&hcan, CAN_RxFifo, &CAN_RxHeader, CAN_RxData);

            if(CAN_RxData[0] == 0xF3){
              setChannelPulse(activeTimer, EXTI_channel , 0); // turn the belt motors of
              CAN_TxData[0] = 0xF3;
              break;
            }
          }
          HAL_CAN_AddTxMessage(&hcan, &CAN_TxHeader, CAN_TxData, &CAN_TxMailbox); // Return for acknowledgment
          break;
        }
      }
    }
  }
////////////////////////////////////////////////////////////////////////////////////

    if (!HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RxFifo)) //Checks if there are any messages being or about to be sent
      continue;
    HAL_CAN_GetRxMessage(&hcan, CAN_RxFifo, &CAN_RxHeader, CAN_RxData); //Retrieving message

    switch (CAN_RxData[0]) {
      case 0xAA:    //set pin direction
        pinMode(getGPIOPort(CAN_RxData[1]), getGPIOPin(CAN_RxData[1]), CAN_RxData[2], 0);
        CAN_response_length = 0;  
      break;

      case 0xAB:    //set pin output
        HAL_GPIO_WritePin(getGPIOPort(CAN_RxData[1]), getGPIOPin(CAN_RxData[1]), CAN_RxData[2]);
        CAN_response_length = 0;
      break;

      case 0xAC:    //read pin
        CAN_response_data[0] = HAL_GPIO_ReadPin(getGPIOPort(CAN_RxData[1]), getGPIOPin(CAN_RxData[1]));
        CAN_response_length = 1;
      break;

      case 0xAD:    //set pin alternate function
        setPinAlternateMode(CAN_RxData[1], CAN_RxData[2]);
        CAN_response_length = 0;
      break;

      case 0xBA:    //Timer set frequency for PWM
        activeTimer = getTimer(CAN_RxData[1]);  
        timerFreq = CAN_RxData[2] << 16 | CAN_RxData[3] << 8 | CAN_RxData[4];
        if (timerFreq > 10000)
          timerPrescaler = 0;
        else if (timerFreq > 1000)
          timerPrescaler = 10;
        else 
          timerPrescaler = 100;
        // timerPrescaler = (int)(SYS_FREQ / timerFreq / 1000) - 1;
        timerPeriod = (int)(SYS_FREQ / timerFreq / (timerPrescaler + 1)) - 1;
        activeTimer->Instance->CCR1 = 0;
        activeTimer->Instance->CCR2 = 0;
        activeTimer->Instance->CCR3 = 0;
        activeTimer->Instance->CCR4 = 0;
        activeTimer->Instance->ARR = timerPeriod;
        activeTimer->Instance->PSC = timerPrescaler;
        activeTimer->Instance->CCR1 = timerPeriod / 2;
        activeTimer->Instance->CCR2 = timerPeriod / 2;
        activeTimer->Instance->CCR3 = timerPeriod / 2;
        activeTimer->Instance->CCR4 = timerPeriod / 2;
        CAN_response_length = 0;
      break;

      case 0xBB:    //Timer set duty cycle
        activeTimer = getTimer(CAN_RxData[1]);
        timerChannel = CAN_RxData[2];
        channelPulse = (CAN_RxData[3] * timerPeriod)/100; //Duty%
        setChannelPulse(activeTimer, timerChannel, channelPulse);
        CAN_response_length = 0;
      break;
      
      case 0xBC:    //Timer set on time
        activeTimer = getTimer(CAN_RxData[1]);
        timerChannel = CAN_RxData[2];
        channelPulse = ((CAN_RxData[3] << 8) | (CAN_RxData[4])); //Shift by 8 to allow 2 bytes
        float setTimerCalc =((channelPulse + 0.7)/0.23);       //f(x) = 0.23x-0.07 //x=(y+0.7)/0.23
        setChannelPulse(activeTimer, timerChannel, setTimerCalc);
        CAN_response_length = 0;
      break;

      case 0xCA:    //read UART buffer
        uint8_t uart_num = CAN_RxData[1];
        UART_HandleTypeDef* uart_port;
        uint8_t* uart_buffer;
        if (uart_num == 1) {
          uart_port = &huart1;
          uart_buffer = UART1_RxData;
        }
        else if (uart_num == 2) {
          uart_port = &huart2;
          uart_buffer = UART2_RxData;
        }
        HAL_UART_DMAStop(uart_port);
        memcpy(CAN_response_data, uart_buffer, 200);
        memset(uart_buffer, 0, 200);
        HAL_UART_Receive_DMA(uart_port, uart_buffer, 200);
        CAN_response_length = strlen((char *)CAN_response_data);
      break;
      
      case 0xDA:   //Change ID
        uint32_t Changed_ID = ((CAN_RxData[1] << 24) | (CAN_RxData[2] << 16) | (CAN_RxData[3] << 8) | CAN_RxData[4]); //The ID is contained in the Data Arrays from 1 to 4 
        uint32_t Changed_Filter_ID = ((Changed_ID << 3) | (REMOTE_FRAME << 1) | (EXTID << 2));  //Shifted by 3 because the RTR and IDE still needs to fit
        CAN_FilterType.FilterIdHigh = (Changed_Filter_ID >> 16); 
        CAN_FilterType.FilterIdLow =  Changed_Filter_ID;
        CAN_TxHeader.ExtId = Changed_ID;
        Response_ID = Changed_ID; 
        HAL_CAN_ConfigFilter(&hcan, &CAN_FilterType);
        CAN_response_length = 0;
      break;

      case 0xE0: //Disable usb
        HAL_PCD_DeInit(&hpcd_USB_FS);
        CAN_response_length = 0;
      break;

      case 0xE1:  //Initialize USB
        MX_USB_PCD_Init();
        CAN_response_length = 0;
      break;

      case 0xF0: // EXTI_enable_disable
        EXTI_enable_disable = CAN_RxData[1];
        break;

      case 0xFA: // EXTI_timer
        EXTI_timer = CAN_RxData[1];
        break;

      case 0xFB: // EXTI_Channel
        EXTI_channel = CAN_RxData[1];
        break;
      
      case 0xFC: // EXTI_duty_cycle
        EXTI_duty_cycle = CAN_RxData[1];
        break;

      case 0xFD: // EXTI_sensor_function
        EXTI_sensor_function[0] = CAN_RxData[1];
        EXTI_sensor_function[1] = CAN_RxData[2];
        EXTI_sensor_function[2] = CAN_RxData[3];
        break;

      case 0xFE: // EXTI_Pins
        EXTI_Pins[0] = CAN_RxData[1] << 8 | CAN_RxData[2];
        EXTI_Pins[1] = CAN_RxData[3] << 8 | CAN_RxData[4];
        EXTI_Pins[2] = CAN_RxData[5] << 8 | CAN_RxData[6];
        break;

      case 0xFF:
        EXTI_timer = CAN_RxData[1];
        EXTI_channel = CAN_RxData[2];
        EXTI_duty_cycle = CAN_RxData[3];
        EXTI_sensor_function[0] = (CAN_RxData[4] & 0xF0);
        EXTI_sensor_function[1] = (CAN_RxData[4] << 4);
        EXTI_sensor_function[2] = (CAN_RxData[5] & 0xF0);
        EXTI_Pins[0] = (1 << (CAN_RxData[5] & 0x0F));
        EXTI_Pins[1] = (1 << (CAN_RxData[6] & 0xF0));
        EXTI_Pins[2] = (1 << (CAN_RxData[6] & 0x0F));
        break;

    }
    //CAN_response_data[0] = CAN_RxData[0];
    sendCANResponse(&hcan, CAN_RxData[0], CAN_response_data, CAN_response_length, Response_ID);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPO_Direction_M1_Pin|GPO_Direction_M2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GPO_Direction_M1_Pin GPO_Direction_M2_Pin */
  GPIO_InitStruct.Pin = GPO_Direction_M1_Pin|GPO_Direction_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPI_Sensor_3_Pin */
  GPIO_InitStruct.Pin = GPI_Sensor_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPI_Sensor_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPI_Sensor_2_Pin GPI_Sensor_s1_Pin */
  GPIO_InitStruct.Pin = GPI_Sensor_2_Pin|GPI_Sensor_s1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SM_GPIO25_Pin SM_GPIO25B13_Pin SM_GPIO27_Pin SM_GPIO28_Pin
                           SM_GPIO29_Pin */
  GPIO_InitStruct.Pin = SM_GPIO25_Pin|SM_GPIO25B13_Pin|SM_GPIO27_Pin|SM_GPIO28_Pin
                          |SM_GPIO29_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SM_GPIO26_Pin */
  GPIO_InitStruct.Pin = SM_GPIO26_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SM_GPIO26_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
GPIO_TypeDef* getGPIOPort(uint8_t CAN_pin) {
  if ((CAN_pin >> 4) == 0x0A)       //This gets shifted so the second HEX is ignored
    return GPIOA;
  else if ((CAN_pin >> 4) == 0x0B) 
    return GPIOB;
  else if ((CAN_pin >> 4) == 0x0C) 
    return GPIOC;
  return 0;
}

uint16_t getGPIOPin(uint8_t CAN_pin) {
  return 1 << (CAN_pin & 0x0F); //Because the Pin Hex constants are a power of 1 this is shifted by the second number 
}

void pinMode(GPIO_TypeDef* port, uint16_t pin, uint8_t dir, uint8_t pull) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  if (dir) {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    pull = 0;
  }
  uint8_t GPIO_Modes[] = {0x0, 0x1, 0x2, 0x3}; //The Analouge wont work probably, the number would usally be the same as input 
  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_Modes[dir];
  GPIO_InitStruct.Pull = pull;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/**
 *@brief Change the mode of a specified pin
 *@param input_pin The pin which is to be modified. Pins must be looked up in a Datasheet
 *@param pin_function What the pins function should be. This can be B1-7 (Timer), C1-4 (USART), D0 (CAN)
 */
void setPinAlternateMode(uint16_t input_pin, uint16_t pin_function) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  uint8_t alternate;
  GPIO_TypeDef* port = getGPIOPort(input_pin);
  uint16_t pin = getGPIOPin(input_pin);

  switch (pin_function){
    case 0xB1: alternate = GPIO_AF2_TIM1; break;  //Timer 1
    case 0xB2: alternate = GPIO_AF2_TIM2; break;  //Timer 2
    case 0xB3: alternate = GPIO_AF1_TIM3; break;  //Timer 3
    case 0xB4: alternate = GPIO_AF4_TIM14; break; //Timer 14
    case 0xB5: alternate = (port == GPIOA) ? GPIO_AF0_TIM15 : GPIO_AF1_TIM15; break;    //Timer 15
    case 0xB6: alternate = (port == GPIOA) ? GPIO_AF5_TIM16 : GPIO_AF2_TIM16; break;    //Timer 16
    case 0xB7: alternate = (port == GPIOA) ? GPIO_AF5_TIM17 : GPIO_AF2_TIM17; break;    //Time 17
    case 0xC1: alternate = (port == GPIOA) ? GPIO_AF1_USART1 : GPIO_AF0_USART1; break;  //USART 1
    case 0xC2: alternate = GPIO_AF1_USART2; break;   //USART 2
    case 0xC3: alternate = GPIO_AF4_USART3; break;   //USART 3
    case 0xC4: alternate = GPIO_AF4_USART4; break;   //USART 4
    case 0xD0: alternate = GPIO_AF4_CAN; break;      //CAN
  }

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //Alternate Function Push Pull Mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;     //No Pull-up or Pull-down activation
  GPIO_InitStruct.Speed = ((pin_function & 0xF0) == 0xB) ? GPIO_SPEED_FREQ_LOW : GPIO_SPEED_FREQ_HIGH;  //The Timer requiere a lower frequenzy
  GPIO_InitStruct.Alternate = alternate;  //Alternate Function mapping
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

/**
 *@brief This Function outputs a CAN response
 *@param hcan The Response Header
 *@param command This is usally the first byte of the Data Field
 *@param CAN_response The Data which is to be Transmitted
 *@param CAN_response_length Length of the Response
 *@return Sends a CAN Response
 */
void sendCANResponse(CAN_HandleTypeDef* hcan, uint8_t command, uint8_t * CAN_response, int16_t CAN_response_length, uint32_t Response_ID) {
  CAN_TxHeaderTypeDef CAN_TxHeader;
  uint8_t CAN_TxData[8];            //Array for Data
  uint32_t CAN_TxMailbox;
  uint8_t packet_end = 0;           
  CAN_TxHeader.IDE = CAN_ID_EXT;    //Extended ID
  CAN_TxHeader.RTR = CAN_RTR_DATA;  //Remote Transmission Request set to 
  for (uint8_t packet = 0; packet < 30; packet++) {
    memcpy(&CAN_TxData[1], &CAN_response[packet * 7], 7); //Copies the data from CAN_response to CAN_Data[1]
    uint8_t DLC = 8;                //Length of the code
    CAN_TxData[0] = command;        
    if (CAN_response_length <= 6) {
      DLC = CAN_response_length + 2;  
      CAN_TxData[DLC - 1] = 0x6B;   
      packet_end = 1;
    }
    CAN_response_length -= 7;
    CAN_TxHeader.DLC = DLC;
    CAN_TxHeader.ExtId = Response_ID | packet;
    HAL_CAN_AddTxMessage(hcan, &CAN_TxHeader, CAN_TxData, &CAN_TxMailbox); //Takes the header of the system, adds the Data and Mailbox
    HAL_Delay(1);
    if (packet_end) {
      memset(CAN_response, 0, 250);
      return;
    }
  }
}

TIM_HandleTypeDef* getTimer(uint8_t timer) {
  if (timer == 1)
    return &htim1;
  else if (timer == 2)
    return &htim2;
  else
    return &htim3;
}

void setChannelPulse(TIM_HandleTypeDef* timer, uint8_t channel, uint16_t pulse) {
  //pulse *= 10;
  if (channel == 1)
    timer->Instance->CCR1 = pulse; 
  else if (channel == 2)
    timer->Instance->CCR2 = pulse;
  else if (channel == 3)
    timer->Instance->CCR3 = pulse;
  else if (channel == 4)
    timer->Instance->CCR4 = pulse;
}

void SetFrequencyPWM(uint8_t timer, uint16_t frequency){
  activeTimer = getTimer(timer);  
  if (frequency > 10000)
    timerPrescaler = 0;
  else if (frequency > 1000)
    timerPrescaler = 10;
  else 
    timerPrescaler = 100;
  // timerPrescaler = (int)(SYS_FREQ / frequency / 1000) - 1;
  timerPeriod = (int)(SYS_FREQ / frequency / (timerPrescaler + 1)) - 1;
  activeTimer->Instance->ARR = timerPeriod;
  activeTimer->Instance->PSC = timerPrescaler;
}

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
