/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdlib.h>
#include "math.h"
#include "task.h"
#include "queue.h"
#include "serial_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void task_UARTman( void *pvParameters );
void hedgehog_send_write_answer_success(void);
void hedgehog_set_crc16(uint8_t *buf, uint8_t size);
QueueHandle_t xQueue_coord;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// variáveis do magnetômetro, bússula, módulo HMC5883L
uint8_t adjst[2] = {0x01, 0x1D};
uint8_t readMag[6];
int16_t Mx, My, Mz;
float bussMagRead, bussula;
#define pi 3.14159

// variáveis do acelerômetro e giroscópio, módulo MPU6050

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;


//  MARVELMIND HEDGEHOG RELATED PART BEGIN
//#define READY_RECEIVE_PATH_PIN 3

//////////////////

#define PACKET_TYPE_STREAM_FROM_HEDGE 0x47
#define PACKET_TYPE_REPLY_TO_STREAM 0x48
#define PACKET_TYPE_READ_FROM_DEVICE 0x49
#define PACKET_TYPE_WRITE_TO_DEVICE 0x4a

#define HEDGEHOG_POS_PACKET_ID 0x0001
#define READY_CONFIRM_PACKET_ID 0x0100
#define MOVEMENT_PATH_PACKET_ID 0x0201

#define DATA_OFS 5

int16_t hedgehog_x, hedgehog_y;// coordinates of hedgehog (X,Y), cm
int16_t hedgehog_z;// height of hedgehog, cm
uint8_t hedgehog_flags= 0;
uint8_t hedgehog_address= 0;
int16_t hedgehog_pos_updated = 0;// flag of new data from hedgehog received

#define HEDGEHOG_BUF_SIZE 64
uint8_t hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
uint8_t hedgehog_serial_buf_ofs = 0;
uint8_t hedgehog_packet_size;

uint8_t hedgehog_packet_type;
int16_t hedgehog_packet_id = 0;

HAL_UART_StateTypeDef UART_status2; // configuração da UART
uint8_t UART_data_rx, UART_data_tx; // configuração da UART
uint32_t TimeoutUART = 1; // tempo

typedef union {uint8_t b[2]; uint16_t w;} uni_8x2_16;

typedef struct {
  uint8_t moveType;
  int16_t param1;
  int16_t param2;
} MoveItem;
#define MAX_MOVE_ITEMS 64
MoveItem moveItems[MAX_MOVE_ITEMS];
uint8_t moveItemsNum= 0;

uint32_t prevMoveItemShowTime= 0;
uint8_t moveItemShowIndex= 0;

/////////////////////////////////////////////////////////////////////////////////////////////////
void hedgehog_send_packet(uint8_t address, uint8_t packet_type, uint16_t id, uint8_t data_size)
{
   uint8_t frameSizeBeforeCRC;

   hedgehog_serial_buf[0]= address;

   hedgehog_serial_buf[1]= packet_type;

   hedgehog_serial_buf[2]= id&0xff;
   hedgehog_serial_buf[3]= (id>>8)&0xff;

   if (data_size != 0)
   {
     hedgehog_serial_buf[4]= data_size;
     frameSizeBeforeCRC= data_size+5;
   }
   else
   {
     frameSizeBeforeCRC= 4;
   }

   hedgehog_set_crc16(&hedgehog_serial_buf[0], frameSizeBeforeCRC);
   Serial_Write(hedgehog_serial_buf, frameSizeBeforeCRC+2);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

// Sends confirmation packet of readyness state to transmit/receive data
void hedgehog_send_ready_confirm()
{
  uint8_t status= 0;

  // estado de operação da UART
  if( HAL_GPIO_ReadPin(READY_RECEIVE_PATH_PIN_GPIO_Port, READY_RECEIVE_PATH_PIN_Pin) == GPIO_PIN_RESET)
  {
	  status|= (1<<0);// ready to receive data
  }
  hedgehog_serial_buf[DATA_OFS + 0]= status;
  hedgehog_serial_buf[DATA_OFS + 1]= 0;
  hedgehog_serial_buf[DATA_OFS + 2]= 0;
  hedgehog_serial_buf[DATA_OFS + 3]= 0;

  hedgehog_send_packet(hedgehog_address, PACKET_TYPE_REPLY_TO_STREAM, READY_CONFIRM_PACKET_ID, 4);
}

////////////////////////////////////////////////////////////////////////////////////////////////////


void restart_packet_receive()
{
   hedgehog_serial_buf_ofs= 0;// restart bufer fill
   hedgehog_packet_id= 0;
}

int16_t check_packet_data_size(uint8_t packet_type, uint16_t packet_id, uint8_t size, uint8_t wanted_size)
{
   if (hedgehog_packet_type == packet_type)
     if (hedgehog_packet_id == packet_id)
       if (size != wanted_size)
         {
           restart_packet_receive();
           return 0;
         }
   return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


// Sends answer on write request
void hedgehog_send_write_answer_success(void)
{
   hedgehog_send_packet(hedgehog_address, hedgehog_packet_type, hedgehog_packet_id, 0);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

void process_stream_packet()
{
   if (hedgehog_packet_id == HEDGEHOG_POS_PACKET_ID)
      {// packet of hedgehog position
         hedgehog_x= (int16_t)hedgehog_serial_buf[9] + (int16_t)((hedgehog_serial_buf[10])<<8);
         hedgehog_y= (int16_t)hedgehog_serial_buf[11] + (int16_t)((hedgehog_serial_buf[12])<<8);// coordinates of hedgehog (X,Y), cm
         hedgehog_z= (int16_t)hedgehog_serial_buf[13] + (int16_t)((hedgehog_serial_buf[14])<<8);// height of hedgehog, cm (FW V3.97+)
         hedgehog_flags= hedgehog_serial_buf[15];
         hedgehog_address= hedgehog_serial_buf[16];
         hedgehog_pos_updated= 1;// flag of new data from hedgehog received

		 xQueueSend(xQueue_coord, &hedgehog_x, pdMS_TO_TICKS(1)); // deixa os dados de posição na fila
		 xQueueSend(xQueue_coord, &hedgehog_y, pdMS_TO_TICKS(1));
		 xQueueSend(xQueue_coord, &hedgehog_z, pdMS_TO_TICKS(1));

         if (hedgehog_flags&0x08)
          {// request for writing data
            hedgehog_send_ready_confirm();
          }
      }
}

//////////////////////////////////////////////////////////////////////////////////////////

void process_write_packet()
{
  if (hedgehog_packet_id == MOVEMENT_PATH_PACKET_ID)
     {// movement path packet
	    uint8_t indcur= hedgehog_serial_buf[5+1];
        moveItemsNum= hedgehog_serial_buf[5+2];

        if (moveItemsNum>=MAX_MOVE_ITEMS)
          moveItemsNum= MAX_MOVE_ITEMS;
        if (indcur<moveItemsNum)
          {
            moveItems[indcur].moveType= hedgehog_serial_buf[5+0];
            moveItems[indcur].param1= hedgehog_serial_buf[5+3] | (int16_t)((hedgehog_serial_buf[5+4])<<8);
            moveItems[indcur].param2= hedgehog_serial_buf[5+5] | (int16_t)((hedgehog_serial_buf[5+6])<<8);

          }

        // send answer packet
        hedgehog_send_write_answer_success();
     }
}

//////////////////////////////////////////////////////////////////////////////////

// Marvelmind hedgehog service loop
//void loop_hedgehog()
void task_UARTman( void *pvParameters ) // TASK DO SENSOR GPS
{int16_t incoming_byte;
 int16_t total_received_in_loop;
 int16_t packet_received;
 //int packet_id;
 //int i,n,ofs;

  while(1)
  {
	  total_received_in_loop= 0;
	  packet_received= 0;
	  while(Serial_Available() > 0) //Verificando se tem byter recebidos
	  {
		  if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE)
			{
			  hedgehog_serial_buf_ofs= 0;
			  break;// buffer overflow
			}
		  total_received_in_loop++;
		  if (total_received_in_loop>100) break;// too much data without required header

		  //incoming_byte = Serial.read();
		  incoming_byte = Serial_Read(); //Lendo o byte
		  if (hedgehog_serial_buf_ofs==0)
			{// check first bytes for constant value
			  if (incoming_byte != 0xff)
				{
				  hedgehog_serial_buf_ofs= 0;// restart bufer fill
				  hedgehog_packet_id= 0;
				  continue;
				}
			}
		  else if (hedgehog_serial_buf_ofs==1)
			{// check packet type
			  if ( (incoming_byte == PACKET_TYPE_STREAM_FROM_HEDGE) ||
				   (incoming_byte == PACKET_TYPE_READ_FROM_DEVICE) ||
				   (incoming_byte == PACKET_TYPE_WRITE_TO_DEVICE)
				 )
				{// correct packet type - save
				  hedgehog_packet_type= incoming_byte;
				}
			   else
				{// incorrect packet type - skip packet
				  restart_packet_receive();
				  continue;
				}
			}
		  else if (hedgehog_serial_buf_ofs==3)
			{// Check two-bytes packet ID
			  hedgehog_packet_id= hedgehog_serial_buf[2] + incoming_byte*256;
			  switch(hedgehog_packet_type)
			  {
				case PACKET_TYPE_STREAM_FROM_HEDGE:
				  {
					switch(hedgehog_packet_id)
					  {
						case HEDGEHOG_POS_PACKET_ID:
						  {
							break;
						  }

						default:
						  {// incorrect packet ID - skip packet
							restart_packet_receive();
							continue;
						  }
					  }
					break;
				  }

				  case PACKET_TYPE_WRITE_TO_DEVICE:
				  {
					switch(hedgehog_packet_id)
					  {
						case MOVEMENT_PATH_PACKET_ID:
						  {
							break;
						  }

						default:
						  {// incorrect packet ID - skip packet
							restart_packet_receive();
							continue;
						  }
					  }
					break;
				  }
			  }
			}
		else if (hedgehog_serial_buf_ofs==4)
		  {// data field size
			if (check_packet_data_size(PACKET_TYPE_STREAM_FROM_HEDGE, HEDGEHOG_POS_PACKET_ID, incoming_byte, 0x10) == 0)
			   continue;// incorrect hedgehog coordinates data size

			if (check_packet_data_size(PACKET_TYPE_WRITE_TO_DEVICE, MOVEMENT_PATH_PACKET_ID, incoming_byte, 0x0c) == 0)
			   continue;// incorrect movement path packet data size

			 // save required packet size
			 hedgehog_packet_size= incoming_byte + 7;
		  }

		  hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte;
		  if (hedgehog_serial_buf_ofs>5)
		   if (hedgehog_serial_buf_ofs == hedgehog_packet_size)
			{// received packet with required header
			  packet_received= 1;
			  hedgehog_serial_buf_ofs= 0;// restart bufer fill
			  break;
			}
		}


	  if (packet_received)
		{
		  hedgehog_set_crc16(&hedgehog_serial_buf[0], hedgehog_packet_size);// calculate CRC checksum of packet
		  if ((hedgehog_serial_buf[hedgehog_packet_size] == 0)&&(hedgehog_serial_buf[hedgehog_packet_size+1] == 0))
			{// checksum success
			  switch(hedgehog_packet_type)
			  {
				case PACKET_TYPE_STREAM_FROM_HEDGE:
				 {
				   process_stream_packet();
				   break;
				 }

				case PACKET_TYPE_WRITE_TO_DEVICE:
				 {
				   process_write_packet();
				   break;
				 }
			  }//switch
			}
		}// if (packet_received)

	  vTaskDelay(pdMS_TO_TICKS(10));
	} //while
}// loop_hedgehog

//////////////////////////////////////////////////////////////////////////////////////


// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(uint8_t *buf, uint8_t size)
{uni_8x2_16 sum;
 uint8_t shift_cnt;
 uint8_t byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// hedgehog_set_crc16

////////////////////////////////////////////////  END OF MARVELMIND HEDGEHOG RELATED PART

/// Funções do módulo  MPU6050 /////////////////////////////////////////////////////

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (�/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}

/////////////// FUNÇÕES DO MÓDULO HMC5883L ///////////////////////////////////////

void HMC5883L_Init (void)
{
	HAL_I2C_Mem_Write(&hi2c3, 0x1A, 0x0B, 1, &adjst[0], 1, 100); //escreve nessas posições
	HAL_I2C_Mem_Write(&hi2c3, 0x1A, 0x09, 1, &adjst[1], 1, 100);
}

void HMC5883L_Read_Mag (void)
{
	HAL_I2C_Mem_Read(&hi2c3, 0x1A, 0x06, 1, readMag, 1, 100);
	if(readMag[0]&0x01)
	{
		HAL_I2C_Mem_Read(&hi2c3, 0x1A, 0x00, 1, readMag, 6, 100);
		Mx = (readMag[1]<<8)|readMag[0];
		My = (readMag[3]<<8)|readMag[2];
		Mz = (readMag[5]<<8)|readMag[4];
		bussula = atan2f(My, Mx)*180/pi;
		if(bussula > 0)
		{
			bussMagRead = bussula;
		}
		else
		{
			bussMagRead = 360 + bussula;
		}
	}
}
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
  Serial_begin(&huart2);  //Configurando para transmitir e recebervia uart
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  xQueue_coord = xQueueCreate( 3, sizeof( int16_t ) );

  xTaskCreate(task_UARTman, "readwriteUART", 256, NULL, 1, NULL);

  MPU6050_Init();
  HMC5883L_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  vTaskStartScheduler();
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	MPU6050_Read_Accel();
	MPU6050_Read_Gyro();
	HMC5883L_Read_Mag ();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  huart2.Init.BaudRate = 500000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  NVIC_SetPriority(USART2_IRQn,
    		NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : READY_RECEIVE_PATH_PIN_Pin */
  GPIO_InitStruct.Pin = READY_RECEIVE_PATH_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(READY_RECEIVE_PATH_PIN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
