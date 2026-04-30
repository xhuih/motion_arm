/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "Emm_V5.h"
#include "ZDT.h"
#include "uart.h"

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

/* USER CODE BEGIN PV */
//ni
float a,b,pi=3.14;
float c ;
//目标坐标
//与MATLAB仿真放置角度差异
//qian_mian_keil,hou_mian_mat
//X:Z,Y:Y,Z:X
float x_t,y_t,z_t;
//目标点到2轴参数
float t_xy,t_xz,t_L,t_z;
//杆长
float L1,L2;
//角度
float xita_3,xita_2,xita_1;
//三角函数
float c2,s2,c3,s3,c11;
//系数
float k,j,z;

//key
float clk_1,clk_2,clk_3;
uint8_t usdir;
uint8_t key_temp,key1,key2,key3,key4;

//uart
uint8_t rx_buf[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void nijie_proce(void);
void key_proce(void);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart==&huart3)
	{
		write_proce(rx_buf,Size);
		HAL_UARTEx_ReceiveToIdle_IT(&huart3,rx_buf,sizeof(rx_buf));
	}
}
/*
读出的Y对应-X，X对应Y，Z相同（现在摄像头改）
计算时又不一样
计算（前M后K）Z对应X Y对应Y X对应Z（应该没有取反？）
传过来的数据：帧头 X高8X低8 Y高8Y低8 Z高8Z低8 校验
要以计算用的XYZ坐标去接收识别到的XYZ坐标
*/

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart==&huart3)
//	{
//		write_proce(rx_buf,8);
//		//HAL_UARTEx_ReceiveToIdle_IT(&huart3,rx_buf,sizeof(rx_buf));
//		HAL_UART_Receive_IT(&huart3,rx_buf,sizeof(rx_buf));
//	}
//}

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	a = pi/180;         //角度转弧度
	b = (1 / pi) * 180;    //弧度转角度
	c= (81.1827/360)*3200;//clk脉冲数

	//30  30  -60  90  0
	//250.795  -54.204  -72.250

	//t:目标
	//目标坐标
	x_t = 254.2;
	y_t = 34.84;
	z_t = -72.250;

	//两根连杆长
	L1 = 120;
	L2 = 190;

//	HAL_UART_Receive_IT(&huart3,rx_buf,sizeof(rx_buf));
	HAL_UARTEx_ReceiveToIdle_IT(&huart3,rx_buf,sizeof(rx_buf));
	uint8_t tx_data[50];
	uint8_t data_length=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		data_length = get_command(tx_data);
		if(data_length != 0)
		{
//			计算的x_t = 换算臂的z;
//						y_t = x;
//						z_t = -y;
			//应该还要再减去机械臂原点坐标
			HAL_UART_Transmit(&huart3,tx_data,data_length,HAL_MAX_DELAY);
			x_t = (0.00+182.25-170.00)+((tx_data[6]<<8)|tx_data[5]);
			y_t = (170.00+122.50+148.50)-((tx_data[2]<<8)|tx_data[1]);		//到摄像头距离+到摄像头三维原点距离+误差偏移
			z_t = (-80.00+104.50+154.5)-((tx_data[4]<<8)|tx_data[3]);	//多一个基座抬高距离
			nijie_proce();
			
		}
		
		if(tx_data[5]!=0&&tx_data[1]!=0&&tx_data[3]!=0)
		{
			
			if(clk_3<0)
		{
			clk_3 = -clk_3;
			usdir = 1;
		}
		else usdir =0;
		position_control_zdt(1,usdir,3000,clk_3,true,0);	//dir原0
		HAL_Delay(5);
		
		if(clk_2<0)
		{
			clk_2 = -clk_2;
			usdir = 1;
		}
		else usdir =0;
		Emm_V5_Pos_Control(2, usdir, 500, 25, clk_2, true, false);	//dir原1
		HAL_Delay(5);
		
		
		if(clk_1<0)
		{
			clk_1 = -clk_1;
			usdir = 0;
		}
		else usdir =1;
		Emm_V5_Pos_Control(1, usdir, 500, 25, clk_1, true, false);	//dir原1
			
		}
		//HAL_Delay(10000);
		key_proce();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */

void nijie_proce(void)
{
//转到目标点平面的XY轴
//t_x = atan2(y_t,x_t) * b
//t_y = z_t

//目标点xy到2轴距离，跟z轴上的距离组成直角三角形可求得目标点与2轴的距离
t_xy = sqrt(x_t*x_t+y_t*y_t);
//目标点xz到2轴距离
t_xz = sqrt(x_t*x_t+z_t*z_t);
//目标点与2轴在z轴的差距，2轴本身被1轴抬高了37.5
t_z = 37.5 - (z_t);
//目标点到2轴的距离
t_L = sqrt(t_xy*t_xy+t_z*t_z);

//直接用目标点平面和2轴平面算夹角是错的
//acos(t_xy/t_L)*b

//余弦定理求转角
xita_3 = 180 - acos((t_L*t_L-L1*L1-L2*L2)/(-2*L1*L2))*b;
c3 = -(t_L*t_L-L1*L1-L2*L2)/(-2*L1*L2);
s3 = sqrt(1-c3*c3);

s2 = (z_t - 37.5)/(-199*c3 - 120);
c2 = sqrt(1-s2*s2);
xita_2 = asin(s2)*b;

k = 199*c2*c3 +120*c2 ;
j = -199*s3;
//z = (-2*k*y_t + sqrt((4*k*k*y_t*y_t)-(4*(-(j*j+k*k))*(j*j-y_t*y_t))))/(-2*(j*j+k*k));
//c11 = 180 - acos(z)*b;
//xita_1 = c11 -((c11-45.5)*0.14);

z = (2*k*y_t + sqrt((4*k*k*y_t*y_t)-(4*(-(j*j+k*k))*(j*j-y_t*y_t))))/(2*(j*j+k*k));
xita_1 = acos(z)*b;


//x = [0,7,15,22,30,45,55,60,70,75,90];

//一圈3200脉冲，81.1827圈:1

		clk_1 = xita_1*c;
		clk_2 = xita_2*c;
		clk_3 = xita_3*c;
}

void key_proce(void)
{
	key1 = HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin);
		key2 = HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin);
		key3 = HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin);
		key4 = HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin);
		
		if(key1==0)
		{
			HAL_Delay(30);
			key_temp++;
			if(key_temp>2) key_temp=0;
			while(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==0);
		}
		if(key2==0)
		{
			HAL_Delay(30);
			if(key_temp==0) Emm_V5_Pos_Control(1, 0, 500, 25, 3600*5, false, false);
			if(key_temp==1) Emm_V5_Pos_Control(2, 0, 500, 25, 3600*5, false, false);
			if(key_temp==2) position_control_zdt(1,0,500,3600*5,false,0);			
			while(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==0);
		}
		if(key3==0)
		{
			HAL_Delay(30);
			if(key_temp==0) Emm_V5_Pos_Control(1, 1, 500, 25, 3600*5, false, false);
			if(key_temp==1) Emm_V5_Pos_Control(2, 1, 500, 25, 3600*5, false, false);
			if(key_temp==2) position_control_zdt(1,1,500,3600*5,false,0);			

			while(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==0);
		}
		if(key4==0)
		{
			HAL_Delay(30);
			Emm_V5_Pos_Control(1, 0, 500, 25, 0, true, false);
			HAL_Delay(5);
			Emm_V5_Pos_Control(2, 0, 500, 25, 0, true, false);
			HAL_Delay(5);
			position_control_zdt(1,1,2000,0,true,0);

			while(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)==0);
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
