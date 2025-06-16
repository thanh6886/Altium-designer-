//==============================================================================
#include "main.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "stdio.h"

//==============================================================================
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

//==============================================================================
#define TRUE														1
#define FALSE														0

#define AFS_2G  													0x00
#define AFS_4G  													0x02
#define AFS_8G  													0x03
#define AFS_16G 													0x01

#define GFS_250DPS  												0x00
#define GFS_500DPS  												0x01
#define GFS_1000DPS 												0x02
#define GFS_2000DPS 												0x03

#define AODR_12_5Hz  												0x01
#define AODR_26Hz    												0x02
#define AODR_52Hz    												0x03
#define AODR_104Hz   												0x04
#define AODR_208Hz   												0x05
#define AODR_417Hz   												0x06
#define AODR_833Hz   												0x07
#define AODR_1667Hz  												0x08
#define AODR_3333Hz  												0x09
#define AODR_6667Hz  												0x0A

#define GODR_12_5Hz  												0x01   
#define GODR_26Hz    												0x02
#define GODR_52Hz    												0x03
#define GODR_104Hz   												0x04
#define GODR_208Hz   												0x05
#define GODR_417Hz   												0x06
#define GODR_833Hz   												0x07
#define GODR_1667Hz  												0x08
#define GODR_3333Hz  												0x09
#define GODR_6667Hz  												0x0A

#define TIME_READ_DATA											2
#define TIME_LOG_DATA												100
char buff[500];
//==============================================================================


float dt ; 
uint32_t lastTick = 0;


float aRes,
	  gRes,
	  accelBias[3] = {0, 0, 0},
	  gyroBias[3] = {0, 0, 0},
	  ax, ay, az, gx, gy, gz,gx_rad,gy_rad,gz_rad,
	  Gtemperature,
	  fG[3],
	  PI = 3.141592653589793238462643383279502884f,
	  fRoll = 0,
	  fPitch = 0;

short int iASM330LHHXLData[3],
		  iASM330LHHGData[4];

unsigned short int uiTimeLed = 0,
					 uiTimeLogData = 0,
				   uiTimeReadData = 0;

unsigned char ucDataASM330[14],
			  ucAscale = AFS_2G,
			  ucGscale = GFS_250DPS,
			  ucAODR = AODR_208Hz,
			  ucGODR = GODR_208Hz,
			  ucBufferTemp[100];

//==============================================================================
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
float GetAres(unsigned char ucAscale_);
float GetGres(unsigned char ucGscale_);
void SelfTest(void);
void Reset(void);
void Init(unsigned char ucAscale_,
		  unsigned char ucGscale_,
		  unsigned char ucAODR_,
		  unsigned char ucGODR_);
void OffsetBias(float * dest1, float * dest2);
void ReadData(short int *iDestination);
void ReadXLData(short int *iDestination);
void ReadGData(short int *iDestination);
void SendDataToPC(unsigned char ucCode_,
				  unsigned char *ucData,
				  unsigned short int uiLength);
void SendByteToUSART1(unsigned char ucByte);
	
					
//==============================================================================
void SysTick_Handler(void)
{
	HAL_IncTick();
	
	if(uiTimeLed != 0)
	{
		uiTimeLed--;
	}
	if(uiTimeReadData != 0)
	{
		uiTimeReadData--;
	}
	if(uiTimeLogData != 0){
		uiTimeLogData--;
	}
}

//==============================================================================
void USART1_IRQHandler(void)
{
	unsigned char ucData;

	if(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE) != RESET)
	{
		ucData = (uint8_t)(USART1 -> RDR & (uint8_t)0xFFU);
	}
}


float ALPHA = 0.95;


//==============================================================================
int main(void)
{
	unsigned short int i, j;
	unsigned char ucTemp[4];
	
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	
	aRes = GetAres(ucAscale);
	gRes = GetGres(ucGscale);
	SelfTest();
	Reset();
	Init(ucAscale, ucGscale, ucAODR, ucGODR);
	OffsetBias(gyroBias, accelBias);

	while(1)
	{
		if(uiTimeLed == 0)
		{
			uiTimeLed = 1000;
		}
		else if(uiTimeLed <= 150)
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		}
		if(uiTimeReadData == 0)
		{
			uint32_t nowTick = HAL_GetTick();
      dt = (nowTick - lastTick) / 1000.0f; // d?i sang giây
      lastTick = nowTick;
			
			ReadXLData(iASM330LHHXLData);
			ax = (float)iASM330LHHXLData[0]*aRes - accelBias[0];
			ay = (float)iASM330LHHXLData[1]*aRes - accelBias[1];   
			az = (float)iASM330LHHXLData[2]*aRes - accelBias[2];
			ReadGData(iASM330LHHGData);
			gx = (float)iASM330LHHGData[1]*gRes - gyroBias[0];
			gy = (float)iASM330LHHGData[2]*gRes - gyroBias[1];  
			gz = (float)iASM330LHHGData[3]*gRes - gyroBias[2];
			
//			gx_rad = gx * (PI / 180.0f); // DPS -> rad/s
//      gy_rad = gy * (PI / 180.0f);
//      gz_rad = gz * (PI / 180.0f);
			
			fG[0] = (ax / 32767.0) * 2.0;
			fG[1] = (ay / 32767.0) * 2.0;
			fG[2] = (az / 32767.0) * 2.0;
			
			
			
			
			float    accRoll  = atan2(fG[1], sqrt(pow(fG[0], 2) + pow(fG[2], 2))) * (180 / PI);  
			float    accPitch = atan2(fG[0], sqrt(pow(fG[1], 2) + pow(fG[2], 2))) * (180 / PI);	
			
			
			fRoll  += gx * dt;
	    fPitch += gy * dt;
			
			fRoll  = ALPHA * fRoll  + (1.0f - ALPHA) * accRoll;
	    fPitch = ALPHA * fPitch + (1.0f - ALPHA) * accPitch;
		
			uiTimeReadData = TIME_READ_DATA;
			
			}
			

			if(uiTimeLogData == 0){
      j = 0;
			ucBufferTemp[j] = (unsigned char)(uiTimeLogData >> 8);
			j++;
			ucBufferTemp[j] = (unsigned char)uiTimeLogData;
			j++;
			*((float *)ucTemp) = ax;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = ay;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = az;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = gx;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = gy;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = gz;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = fRoll;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			*((float *)ucTemp) = fPitch;
			for(i=0;i<4;i++)
			{
				ucBufferTemp[j] = ucTemp[i];
				j++;
			}
			SendDataToPC(254, ucBufferTemp, j);
			 uiTimeLogData = TimeLogData;
			}
		}
	}

// ================================================================================









//==============================================================================



void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
								  | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

//==============================================================================
static void MX_I2C1_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00201D2B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if(HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
}

//==============================================================================
static void MX_USART1_UART_Init(void)
{
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
	if(HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

//==============================================================================
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(RS485_DIR_GPIO_Port, RS485_DIR_Pin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = LED1_Pin | RS485_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(INT1_GPIO_Port, &GPIO_InitStruct);
}

//==============================================================================
float GetAres(unsigned char ucAscale_)
{
	float fTemp;

	switch(ucAscale_)
	{
		case AFS_2G:
			 fTemp = 2.0f / 32768.0f;
			 return fTemp;
			 break;
		case AFS_4G:
			 fTemp = 4.0f / 32768.0f;
			 return fTemp;
			 break;
		case AFS_8G:
			 fTemp = 8.0f / 32768.0f;
			 return fTemp;
			 break;
		case AFS_16G:
			 fTemp = 16.0f / 32768.0f;
			 return fTemp;
			 break;
	}
}

//==============================================================================
float GetGres(unsigned char ucGscale_)
{
	float fTemp;

	switch(ucGscale_)
	{
		case GFS_250DPS:
			  fTemp = 250.0f / 32768.0f;
			  return fTemp;
			  break;
		case GFS_500DPS:
			  fTemp = 500.0f / 32768.0f;
			  return fTemp;
			  break;
		case GFS_1000DPS:
			 fTemp = 1000.0f / 32768.0f;
			 return fTemp;
			 break;
		case GFS_2000DPS:
			  fTemp = 2000.0f / 32768.0f;
			 return fTemp;
			 break;
	}
}

//==============================================================================
void SelfTest(void)
{
	short int temp[7] = {0, 0, 0, 0, 0, 0, 0};
	short int accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
	short int accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};
	unsigned char status = FALSE;

	status = 0x38;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x10, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	status = 0x44;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x12, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100);

	status = FALSE;
	while(status == FALSE)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x1E, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		status = status & 0x01;
	}
	ReadData(temp); // read and discard data

	for (uint8_t i = 0; i < 5; i++){
	ReadData(temp);
	accelNom[0] += temp[4]; // read data five times
	accelNom[1] += temp[5];
	accelNom[2] += temp[6];
	}
	accelNom[0] /= 5.0f; // average data
	accelNom[1] /= 5.0f;
	accelNom[2] /= 5.0f;

	status = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x14, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100); // let accel respond

	status = FALSE;
	while(status == FALSE)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x1E, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		status = status & 0x01;
	}
	ReadData(temp); // read and discard data

	for (uint8_t i = 0; i < 5; i++){
	ReadData(temp);
	accelPTest[0] += temp[4];
	accelPTest[1] += temp[5];
	accelPTest[2] += temp[6];
	}
	accelPTest[0] /= 5.0f;
	accelPTest[1] /= 5.0f;
	accelPTest[2] /= 5.0f;

	status = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x14, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100); // let accel respond

	status = FALSE;
	while(status == FALSE)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x1E, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		status = status & 0x01;
	}
	ReadData(temp); // read and discard data

	for (uint8_t i = 0; i < 5; i++){
	ReadData(temp);
	accelNTest[0] += temp[4];
	accelNTest[1] += temp[5];
	accelNTest[2] += temp[6];
	}
	accelNTest[0] /= 5.0f;
	accelNTest[1] /= 5.0f;
	accelNTest[2] /= 5.0f;


	// gyro self test
	status = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x10, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	status = 0x5C;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x11, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	status = 0x44;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x12, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100);

	status = FALSE;
	while(status == FALSE)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x1E, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		status = status & 0x02;
	}
	ReadData(temp); // read and discard data

	for (uint8_t i = 0; i < 5; i++){
	ReadData(temp);
	gyroNom[0] += temp[1]; // read data five times
	gyroNom[1] += temp[2];
	gyroNom[2] += temp[3];
	}
	gyroNom[0] /= 5.0f; // average data
	gyroNom[1] /= 5.0f;
	gyroNom[2] /= 5.0f;

	status = 0x04;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x14, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100); // let gyro respond

	status = FALSE;
	while(status == FALSE)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x1E, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		status = status & 0x02;
	}
	ReadData(temp); // read and discard data

	for (uint8_t i = 0; i < 5; i++){
	ReadData(temp);
	gyroPTest[0] += temp[1];
	gyroPTest[1] += temp[2];
	gyroPTest[2] += temp[3];
	}
	gyroPTest[0] /= 5.0f;
	gyroPTest[1] /= 5.0f;
	gyroPTest[2] /= 5.0f;

	status = 0x0C;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x14, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100); // let gyro respond

	status = FALSE;
	while(status == FALSE)
	{
		HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x1E, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
		status = status & 0x02;
	}
	ReadData(temp); // read and discard data

	for (uint8_t i = 0; i < 5; i++){
	ReadData(temp);
	gyroNTest[0] += temp[1];
	gyroNTest[1] += temp[2];
	gyroNTest[2] += temp[3];
	}
	gyroNTest[0] /= 5.0f;
	gyroNTest[1] /= 5.0f;
	gyroNTest[2] /= 5.0f;

	status = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x14, I2C_MEMADD_SIZE_8BIT, &status, 1, 100);
	HAL_Delay(100);
}

//==============================================================================
void Reset(void)
{
	HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x12, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = ucDataASM330[0] | 0x01;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x12, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	HAL_Delay(100);
}

//==============================================================================
void Init(unsigned char ucAscale_,
		  unsigned char ucGscale_,
		  unsigned char ucAODR_,
		  unsigned char ucGODR_)
{
	ucDataASM330[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x0B, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = ucAODR_ << 4 | ucAscale_ << 2 | 0x02;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x10, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = ucGODR_ << 4 | ucGscale_ << 2;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x11, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x12, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = ucDataASM330[0] | 0x40 | 0x04;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x12, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = 0x08;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x13, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = 0x01 << 5;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x17, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x0D, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
	ucDataASM330[0] = 0x02;
	HAL_I2C_Mem_Write(&hi2c1, 0xD4, 0x0E, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 1, 100);
}

//==============================================================================
void OffsetBias(float * dest1, float * dest2)
{
	short int temp[7] = {0, 0, 0, 0, 0, 0, 0};
	int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

	HAL_Delay(4000);

	for (int ii = 0; ii < 128; ii++)
	{
		ReadData(temp);
		sum[1] += temp[1];
		sum[2] += temp[2];
		sum[3] += temp[3];
		sum[4] += temp[4];
		sum[5] += temp[5];
		sum[6] += temp[6];
		HAL_Delay(50);
	}

	dest1[0] = sum[1]*gRes/128.0f;
	dest1[1] = sum[2]*gRes/128.0f;
	dest1[2] = sum[3]*gRes/128.0f;
	dest2[0] = sum[4]*aRes/128.0f;
	dest2[1] = sum[5]*aRes/128.0f;
	dest2[2] = sum[6]*aRes/128.0f;

	if(dest2[0] > 0.8f)  {dest2[0] -= 1.0f;}
	if(dest2[0] < -0.8f) {dest2[0] += 1.0f;}
	if(dest2[1] > 0.8f)  {dest2[1] -= 1.0f;}
	if(dest2[1] < -0.8f) {dest2[1] += 1.0f;}
	if(dest2[2] > 0.8f)  {dest2[2] -= 1.0f;}
	if(dest2[2] < -0.8f) {dest2[2] += 1.0f;}
}

//==============================================================================
void ReadData(short int *iDestination)
{
	HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x20, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 14, 100);
	
	iDestination[0] = (((short int)ucDataASM330[1]) << 8) | ((short int)ucDataASM330[0]);
	iDestination[1] = (((short int)ucDataASM330[3]) << 8) | ((short int)ucDataASM330[2]);
	iDestination[2] = (((short int)ucDataASM330[5]) << 8) | ((short int)ucDataASM330[4]);
	iDestination[3] = (((short int)ucDataASM330[7]) << 8) | ((short int)ucDataASM330[6]);
	iDestination[4] = (((short int)ucDataASM330[9]) << 8) | ((short int)ucDataASM330[8]);
	iDestination[5] = (((short int)ucDataASM330[11]) << 8) | ((short int)ucDataASM330[10]);
	iDestination[6] = (((short int)ucDataASM330[13]) << 8) | ((short int)ucDataASM330[12]);
}

//==============================================================================
void ReadXLData(short int *iDestination)
{
	HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x28, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 6, 100);
	
	iDestination[0] = (((short int)ucDataASM330[1]) << 8) | ((short int)ucDataASM330[0]);
	iDestination[1] = (((short int)ucDataASM330[3]) << 8) | ((short int)ucDataASM330[2]);
	iDestination[2] = (((short int)ucDataASM330[5]) << 8) | ((short int)ucDataASM330[4]);
}

//==============================================================================
void ReadGData(short int *iDestination)
{
	HAL_I2C_Mem_Read(&hi2c1, 0xD4, 0x20, I2C_MEMADD_SIZE_8BIT, ucDataASM330, 8, 100);
	
	iDestination[0] = (((short int)ucDataASM330[1]) << 8) | ((short int)ucDataASM330[0]);
	iDestination[1] = (((short int)ucDataASM330[3]) << 8) | ((short int)ucDataASM330[2]);
	iDestination[2] = (((short int)ucDataASM330[5]) << 8) | ((short int)ucDataASM330[4]);
	iDestination[3] = (((short int)ucDataASM330[7]) << 8) | ((short int)ucDataASM330[6]);
}

//==============================================================================
void SendDataToPC(unsigned char ucCode_,
				  unsigned char *ucData,
				  unsigned short int uiLength)
{
	unsigned short int i;
	unsigned char ucChecksum;

	SendByteToUSART1('R');
	SendByteToUSART1('A');
	SendByteToUSART1('K');
	SendByteToUSART1(ucCode_);
	ucChecksum = ucCode_;
	SendByteToUSART1((unsigned char)uiLength);
	ucChecksum = ucChecksum ^ ((unsigned char)uiLength);
	SendByteToUSART1((unsigned char)(uiLength >> 8));
	ucChecksum = ucChecksum ^ ((unsigned char)(uiLength >> 8));
	for(i=0;i<uiLength;i++)
	{
		SendByteToUSART1(ucData[i]);
		ucChecksum = ucChecksum ^ ucData[i];
	}
	SendByteToUSART1(ucChecksum);
}

//==============================================================================
void SendByteToUSART1(unsigned char ucByte)
{
	while(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) == RESET);
	USART1 -> TDR = ucByte;
}

//==============================================================================
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
//==============================================================================
