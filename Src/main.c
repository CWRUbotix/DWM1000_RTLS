/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFFER_SIZE 		1024

/* Index to access to sequence number of the blink frame in the tx_msg array. */
#define BLINK_FRAME_SN_IDX 	1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 		1000

#define FRAME_LEN_MAX 		127

#define NUMBER_OF_ANCHORS 	3

#define DELAY_BEFORE_TX 	0

#define SAMPLES_PER_POINT 	3
/* Change to match the device you're programming */
#define XTAL_TRIM 			14

/* Change to match which device you're programming */
#define TX_ANT_DLY 			16442
#define RX_ANT_DLY 			16442

#define SELF_CAN_ID 		0x01

#ifdef ANCHOR
/* ID of this device, change to be unique in the system */
#define SELF_ID 			0x33
#endif

#ifdef BEACON

/* ID of this device will be the CAN bus id, because why not */
#define SELF_ID 			SELF_CAN_ID
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
static dwt_config_t config = {
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,   /* Preamble length. Used in TX only. */
    DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    0,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64)    	 /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

DistanceFrame anchors[NUMBER_OF_ANCHORS];

//uint8_t tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};

int size = 0;
uint8_t uart_buf[1024] = {};
HAL_StatusTypeDef UART_status;
HAL_StatusTypeDef CAN_status;

RangingStatus 	ranging_status;
_Bool 			CAN_Rx_OK 					= 0; //
_Bool 			do_ranging 					= 0;
_Bool 			dwm1000_setup_success 		= 0;
AnchorTimeStamps anchor_stamps;
BeaconTimeStamps beacon_stamps;
uint8 			sequence_num 				= 0;
unsigned long long t_sp, t_rr, t_sf, t_rp, t_sr, t_rf;
///* Buffer to store received frame. See NOTE 1 below. */
static uint8 tx_buffer[FRAME_LEN_MAX];
static uint8 rx_buffer[FRAME_LEN_MAX];
static uint8 uCurrentTrim_val;

#ifdef ANCHOR
uint16 src_address;
uint16 src_panid;
#endif


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

#ifdef BEACON
  /* Enable CAN interrupt */
//    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  CAN_FilterTypeDef  can_filter;
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;
  can_filter.FilterBank = 0;
  can_filter.FilterActivation = CAN_FILTER_ENABLE;

  HAL_CAN_ConfigFilter(&hcan, &can_filter);
  /* Start the CAN Module */
    HAL_CAN_Start(&hcan);

    for (int i = 0; i< NUMBER_OF_ANCHORS; i++) {
    	anchors[i].anchor_id = ((i+1) << 4) | (i+1);
    	anchors[i].distance = -1.0;
    }
#endif

	/* Reset and initialise DW1000. See NOTE 2 below.
  	 * For initialisation, DW1000 clocks must be temporarily set to crystal speed. After initialisation SPI rate can be increased for optimum
  	 * performance. */
  	reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
  	port_set_dw1000_slowrate();
  	if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  	{

  		debug(&huart1, "INIT FAILED :(\r");
  		while (1)
  		{ };
  	}else{
  		debug(&huart1, "INIT SUCCESS!!\r");
  	}
  	port_set_dw1000_fastrate();

  	/* Configure DW1000. See NOTE 3 below. */
  	dwt_configure(&config);

  	/* XTAL Trim */
  	dwt_setxtaltrim(XTAL_TRIM);
  	uCurrentTrim_val = dwt_getxtaltrim();

  	size = sprintf((char*)uart_buf, "XTAL trim val: %d\r\n", uCurrentTrim_val);
  	UART_status = HAL_UART_Transmit(&huart1, uart_buf, size, 500);


    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);


    self_pan_id 	= SELF_ID;
    self_address 	= SELF_ID;
    dwt_setpanid(self_pan_id);
    dwt_setaddress16(self_address);	// why not just have ADDRESS == PAN_ID ?

    dwt_enableframefilter(DWT_FF_DATA_EN); // permit DATA frame types
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  	while (1)
  	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef BEACON
  		uint32_t can_frame_available = HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);

  		if(can_frame_available > 0){
//  			debug(&huart1, "CAN Frame received\r");
  			CAN_RxHeaderTypeDef hddr;
  			uint8_t data[8];
  			CAN_status = HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &hddr, data);

  			if(CAN_status == HAL_OK && hddr.RTR == CAN_RTR_REMOTE && hddr.StdId == SELF_CAN_ID){
  				do_ranging = 1;
  				debug(&huart1, "do_ranging = 1\r");
  			}

  		}

//  		do_ranging = 1;

  		if(do_ranging){
  			HAL_GPIO_WritePin(GPIOB, CAN_OK_LED_Pin, GPIO_PIN_SET);

  			for (int i = 0; i < NUMBER_OF_ANCHORS; i++){
  				DistanceFrame* anchor = &(anchors[i]);
  				int tofs[SAMPLES_PER_POINT];
  				_Bool ranging_ok = 1;

  				for(int j = 0; j < SAMPLES_PER_POINT; j++){
  					ranging_status = range_with_anchor(anchor->anchor_id, &anchor_stamps, &beacon_stamps, &sequence_num);
  					if(ranging_status != RANGING_SUCCESS){
  						ranging_ok = 0;
  						break;
  					}
  					tofs[j] = get_tof(&anchor_stamps, &beacon_stamps);
  					HAL_Delay(10);
  				}

  				if(ranging_ok){
  					int sum = 0;
  					for(int i = 0; i < SAMPLES_PER_POINT; i++){
  						sum += tofs[i];
  					}
  					int tof 	= sum / SAMPLES_PER_POINT;

  					float dist 	= 299792458.0 * (1.0*tof / (128.0*499200000.0));

  					size = sprintf((char*)uart_buf, "Anchor %x,\t", anchor->anchor_id);
  					UART_status = HAL_UART_Transmit(&huart1, uart_buf, size, 500);

  					size = sprintf((char*)uart_buf, "ToF Counts: %d\t", tof);
  					UART_status = HAL_UART_Transmit(&huart1, uart_buf, size, 500);

  					int dist_mm = (int)1000.0*dist;
  					size = sprintf((char*)uart_buf, "DISTANCE (mm): %d\r\n", dist_mm);
  					UART_status = HAL_UART_Transmit(&huart1, uart_buf, size, 500);

  					anchor->type = DISTANCE_FRAME_TYPE;
  					anchor->distance = dist;

  					_Bool can_success = send_CAN_update(&hcan, anchor);
  					if(can_success){
//  						debug(&huart1, "CAN Tx SUCCEEDED\r");
  					}else{
//  						debug(&huart1, "CAN Tx FAILED :(\r");
  					}

  				}else{
  					debug(&huart1, "RANGING FAILED :(\r");
  					anchor->type = DISTANCE_FRAME_TYPE;
  					anchor->distance = -1.0;
  					_Bool can_success = send_CAN_update(&hcan, anchor);
  		  			HAL_Delay(250);
  				}
  			}
  			do_ranging = 0;
  			HAL_GPIO_WritePin(GPIOB, CAN_OK_LED_Pin, GPIO_PIN_RESET);
  		}

  		/* Execute a delay between transmissions. */
//  		HAL_Delay(TX_DELAY_MS);

#endif // BEACON

#ifdef ANCHOR

  		 int rcv_len = receive_frame(rx_buffer, FRAME_LEN_MAX, 500);
  		 if(rcv_len > 0){
  			 switch(rx_buffer[MAC_SIZE_EXPECTED]){
  			 case POLL:{
  				 anchor_stamps.t_rp = get_rx_timestamp();

  				 src_address 	= get_src_addr(rx_buffer);
  				 src_panid 		= get_src_panid(rx_buffer);
  				 sequence_num 	= get_seq_number(rx_buffer);
  				 uint8 data_len = make_mac_header(tx_buffer, src_address, src_panid, ++sequence_num);
  				 tx_buffer[data_len++] = RESPONSE_INIT;
  				 HAL_Delay(DELAY_BEFORE_TX);
  				 transmit_frame(tx_buffer, data_len + 2, 1); // +2 to account for checksum

  				 anchor_stamps.t_sr = get_tx_timestamp();
  				 //  					 dwt_rxreset();
  				 break;}
  			 case SEND_FINAL:{
  				 anchor_stamps.t_rf = get_rx_timestamp();

  				 uint16 new_src_addr 	= get_src_addr(rx_buffer);
  				 uint16 new_src_panid 	= get_src_panid(rx_buffer);
  				 uint8 new_seq_num 		= get_seq_number(rx_buffer);
  				 if(new_src_addr == src_address && new_src_panid == src_panid){
  					 sequence_num = new_seq_num;
  					 uint8 data_len = make_mac_header(tx_buffer, src_address, src_panid, ++sequence_num);
  					 tx_buffer[data_len++] = RESPONSE_DATA;
  					 memcpy(tx_buffer+data_len, &anchor_stamps, sizeof(anchor_stamps));

  	  				 HAL_Delay(DELAY_BEFORE_TX);
  	  				 transmit_frame(tx_buffer, data_len + sizeof(anchor_stamps) + 2, 1);
  	  				 anchor_stamps.t_rp = 0;
  	  				 anchor_stamps.t_sr = 0;
  	  				 anchor_stamps.t_rf = 0;
  				 }else{
  					 debug(&huart1, "Received from unexpected source\r");
  				 }

  				 break;}
  			 default:{
  				 debug(&huart1, "Unrecognized type received\r");
  			 	 break;}
  			 }
  		 }else{
  			 debug(&huart1, "Timed out\r");
  		 }

#endif // ANCHOR

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DW_RESET_Pin|CAN_OK_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TXLED_Pin RXLED_Pin SFDLED_Pin RXOKLED_Pin */
  GPIO_InitStruct.Pin = TXLED_Pin|RXLED_Pin|SFDLED_Pin|RXOKLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DW_NSS_Pin CAN_OK_LED_Pin */
  GPIO_InitStruct.Pin = DW_NSS_Pin|CAN_OK_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DW_RESET_Pin */
  GPIO_InitStruct.Pin = DW_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin DW_IRQn_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|DW_IRQn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void debug(UART_HandleTypeDef* huart, char* text){
	char c;
	int len = 0;
	while(c != '\r'){
		c = text[len++];
	}
	HAL_UART_Transmit(huart, (uint8_t*)text, len, 500);
	char newline[] = {'\n'};
	HAL_UART_Transmit(huart, newline, 1, 500);
}

RangingStatus range_with_anchor(uint8_t anchor_id, AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps, uint8* seq_num){
	uint8 tx_buf[FRAME_LEN_MAX];
	uint8 rx_buf[FRAME_LEN_MAX];
	uint8 data_len;
	uint16 new_src_addr;
	uint16 new_src_panid;
	uint16 new_seq_num;
	TxStatus tx_status;
	int rcv_len = -1;

	// ----- SEND POLL -----
	data_len = make_mac_header(tx_buf, anchor_id, anchor_id, *seq_num);
	tx_buf[data_len++] = POLL;
	HAL_Delay(DELAY_BEFORE_TX);

	tx_status = transmit_frame(tx_buf, data_len + 2, 1); 	// data_len + 2 to account for checksum

	if(!(tx_status == TX_SUCCESS)){
		return SEND_POLL_FAILED;
	}
	b_stamps->t_sp = get_tx_timestamp();

	// ----- RECEIVE RESPONSE -----
	rcv_len = receive_frame(rx_buf, FRAME_LEN_MAX, 1000);
	if(rcv_len <= 0){
		debug(&huart1, "No frame received\r");
		return RECEIVE_RESPONSE_FAILED;
	}
	new_src_addr 	= get_src_addr(rx_buf);
	new_src_panid 	= get_src_panid(rx_buf);
	new_seq_num 	= get_seq_number(rx_buf);
	if(new_src_addr != anchor_id || new_src_panid != anchor_id){
		debug(&huart1, "Received from unexpected source\r");
		return RECEIVE_RESPONSE_FAILED;
	}
	if(rx_buf[MAC_SIZE_EXPECTED] != RESPONSE_INIT){
		debug(&huart1, "Frame didn't contain the right numbers\r");
		return RECEIVE_RESPONSE_FAILED;
	}

	*seq_num = get_seq_number(rx_buf);
	b_stamps->t_rr = get_rx_timestamp();

	// ----- SEND FINAL -----
	data_len = make_mac_header(tx_buf, anchor_id, anchor_id, ++(*seq_num));
	tx_buf[data_len++] = SEND_FINAL;
	HAL_Delay(DELAY_BEFORE_TX);

	tx_status = transmit_frame(tx_buf, data_len + 2, 1);

	if(!(tx_status == TX_SUCCESS)){
			return SEND_FINAL_FAILED;
	}

	b_stamps->t_sf = get_tx_timestamp();

	// ----- RECEIVE TIMESTAMPS -----
	rcv_len = receive_frame(rx_buf, FRAME_LEN_MAX, 1000);
	if(rcv_len <= 0){
		debug(&huart1, "No timestamps frame received\r");
		return RECEIVE_TIMESTAMPS_FAILED;
	}
	new_src_addr 	= get_src_addr(rx_buf);
	new_src_panid 	= get_src_panid(rx_buf);
	new_seq_num 	= get_seq_number(rx_buf);
	if(new_src_addr != anchor_id || new_src_panid != anchor_id){
		debug(&huart1, "Received timestamps from unexpected source\r");
		return RECEIVE_TIMESTAMPS_FAILED;
	}
	*seq_num = get_seq_number(rx_buf);
	b_stamps->t_ff = get_rx_timestamp();
	if(rx_buf[MAC_SIZE_EXPECTED] != RESPONSE_DATA){
		debug(&huart1, "Timestamps frame didn't contain the right numbers\r");
		return RECEIVE_TIMESTAMPS_FAILED;
	}
	memcpy(a_stamps, rx_buf+MAC_SIZE_EXPECTED+1, sizeof(*a_stamps));

	return RANGING_SUCCESS;
}

int get_tof(AnchorTimeStamps* a_stamps, BeaconTimeStamps* b_stamps){
	int round1 	= (b_stamps->t_rr - b_stamps->t_sp);
	int reply1 	= (a_stamps->t_sr - a_stamps->t_rp);
	int round2 	= (a_stamps->t_rf - a_stamps->t_sr);
	int reply2 	= (b_stamps->t_sf - b_stamps->t_rr);

	int tof 	= ((round1 - reply1) + (round2 - reply2)) / 4;
	return tof;
}

int receive_frame(uint8* buffer, int max_len, int timeout){
	/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
	uint32 status_reg = 0;

	/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
	uint16 frame_len = 0;

	int count = 0;
	timeout = timeout * 5; // convert milliseconds to 200's of uSeconds

	dwt_rxenable(DWT_START_RX_IMMEDIATE); /* Activate reception immediately. See NOTE 3 below. */

	/* Poll until a frame is properly received or an error/timeout occurs. See NOTE 4 below.
	 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
	 * function to access it. */
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)) && !(count >= timeout))
	{
//		HAL_Delay(1);
		usleep(200);
		count++;
	};

	if (status_reg & SYS_STATUS_RXFCG)
	{
		/* A frame has been received, copy it to our local buffer. */
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= max_len)
		{
			dwt_readrxdata(buffer, frame_len, 0);
		}

		/* Clear good RX frame event in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

		return frame_len;
	}
	else if(count >= timeout)
	{
		dwt_forcetrxoff(); // guarantee that the receiver is off
		return -1;
	}else{
		/* Clear RX error events in the DW1000 status register. */
		dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

		/* Reset RX to properly reinitialise LDE operation. */
		dwt_rxreset();
		return -1;
	}
}

TxStatus transmit_frame(uint8* frame, int f_len, _Bool ranging){
	/* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
	dwt_writetxdata(f_len, frame, 0); 			/* Zero offset in TX buffer. */
	dwt_writetxfctrl(f_len, 0, ranging); 		/* Zero offset in TX buffer */
	dwt_starttx(DWT_START_TX_IMMEDIATE); 	// Start transmission

	/* Poll DW1000 until TX frame sent event set.
	 * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
	 * function to access it.*/
	while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)){ };
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); /* Clear TX frame sent event. */

	return TX_SUCCESS;
}

unsigned long long get_tx_timestamp(void){
	uint8 data[5];
	dwt_readtxtimestamp(data);
	unsigned long long retval;

    for (int i = 4; i >= 0; i--)
    {
    	retval <<= 8;
    	retval |= data[i];
    }
    return retval;
}

unsigned long long get_rx_timestamp(void){
	uint8 data[5];
	dwt_readrxtimestamp(data);
	unsigned long long retval;

    for (int i = 4; i >= 0; i--)
    {
    	retval <<= 8;
    	retval |= data[i];
    }
    return retval;
}


_Bool send_CAN_update(CAN_HandleTypeDef *hcan, DistanceFrame* frame){
	CAN_TxHeaderTypeDef hddr;
	HAL_StatusTypeDef retval;
	uint32_t mailbox;
	uint8_t data[8];

	hddr.StdId 	= SELF_CAN_ID;
	hddr.IDE 	= CAN_ID_STD;
	hddr.RTR 	= CAN_RTR_DATA;
	hddr.DLC 	= 8;
	data[0] = frame->type;
	data[1] = frame->anchor_id;
	memcpy(data+2, &(frame->distance), 4);
	memcpy(data+6, &(frame->confidence), 2);

	retval = HAL_CAN_AddTxMessage(hcan, &hddr, data, &mailbox);

	return retval == HAL_OK;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
