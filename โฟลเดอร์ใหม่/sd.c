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

  * opensource.org/licenses/BSD-3-Clause

  *

  ******************************************************************************

  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "elon.h"
#include "pic_connv.h"
#include "ILI9341_Touchscreen.h"



#include "ILI9341_STM32_Driver.h"

#include "ILI9341_GFX.h"



#include "snow_tiger.h"

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

/* Private variables ---------------------------------------------------------*/

// Global Variables for Colors to be used in helper functions

uint16_t COL_MIX   = 0xC418;

uint16_t COL_LT_RED   = 0xFDB8;

uint16_t COL_LT_GREEN = 0xCFFA;

uint16_t COL_LT_BLUE  = 0xBDF7;



// Variables to store current percentages

uint8_t val_red = 100; // อิงจากหน้าจอที่คุณส่งมา
uint8_t val_green = 50;
uint8_t val_blue = 70;
float h=30.0, t=40.0; // เก็บค่า Humidity และ Temperature
uint8_t dataBuffer[8];
uint8_t cmdBuffer[3];
char str[50];

uint8_t Page = 1;
uint8_t count = 0;
volatile uint16_t adc_values[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* Private function prototypes -----------------------------------------------*/

void Update_Bar(uint16_t y_center, uint16_t color_fill, uint16_t color_bg, uint8_t percentage);
uint16_t CRC16_2(uint8_t *ptr, uint8_t len);
uint16_t Mix_RGB565(uint8_t r_pct, uint8_t g_pct, uint8_t b_pct);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



/*

 * Helper Function: Updates the Progress Bar and Text

 * y_center: The Y coordinate of the circle (row center)

 * color_fill: Active color (RED, GREEN, BLUE)

 * color_bg: Background color (Light versions)

 * percentage: 0-100

 */

void Update_Bar(uint16_t y_center, uint16_t color_fill, uint16_t color_bg, uint8_t percentage)

{

	// Calculate Dimensions

	// Total bar width = 150px (from X=60 to X=210)

	uint16_t total_width = 150;

	uint16_t filled_width = (total_width * percentage) / 100;

	uint16_t empty_width = total_width - filled_width;

	uint16_t bar_y_start = y_center - 15; // Box height 30, so center - 15



	// 1. Draw Filled Part

	if(filled_width > 0) {

		ILI9341_Draw_Rectangle(60, bar_y_start, filled_width, 30, color_fill);

	}



	// 2. Draw Empty Part

	if(empty_width > 0) {

		ILI9341_Draw_Rectangle(60 + filled_width, bar_y_start, empty_width, 30, color_bg);

	}



	// 3. Update Text

	// Clear previous text area first (White rectangle behind text)
	// พิกัด X=220 ปลอดภัยจากอาการ Wrap-around แน่นอน

	ILI9341_Draw_Filled_Rectangle_Coord(220, y_center-10, 319, y_center+15, WHITE);



	char buf[10];

	sprintf(buf, "%d%%", percentage);

	ILI9341_Draw_Text(buf, 220, y_center-2, BLACK, 1.8, WHITE); // ปรับ size เป็น 2 ให้ตรงสเปคไลบรารี่

}

uint16_t Mix_RGB565(uint8_t r_pct, uint8_t g_pct, uint8_t b_pct) {
    // แปลง 0-100 เป็นช่วง 0-31 (สำหรับ Red/Blue) และ 0-63 (สำหรับ Green)
    uint16_t r = (r_pct * 31) / 100;
    uint16_t g = (g_pct * 63) / 100;
    uint16_t b = (b_pct * 31) / 100;

    // รวมค่าเข้าด้วยกันตามโครงสร้าง RGB565: [RRRRRGGGGGGBBBBB]
    return (uint16_t)((r << 11) | (g << 5) | b);
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

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341
  cmdBuffer[0] = 0x03; // Read Function
  cmdBuffer[1] = 0x00; // Start Address
  cmdBuffer[2] = 0x04; // Read 4 Registers (T & H)
  // ตัวแปรที่ใช้รับ, จำนวนข้อมูล
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // ต้องสั่ง Start ครั้งเดียวก่อนเข้า Loop
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)

    {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	  switch (Page){
  	  case 1:
  		  // ---------------------------------------------------------

  		  // INITIAL UI DRAWING (Static Parts)

  		  // ---------------------------------------------------------

  		  ILI9341_Fill_Screen(WHITE);

  		  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);

  		  // --- TOP ROW ---
  		  COL_MIX = Mix_RGB565(val_red, val_green, val_blue);
  		  ILI9341_Draw_Filled_Circle(120, 35, 20, COL_MIX);

  		  // --- ROW 1: RED (Initial) ---

  		  ILI9341_Draw_Filled_Circle(30, 90, 15, RED);

  		  Update_Bar(90, RED, COL_LT_RED, val_red);

  		  // --- ROW 2: GREEN (Initial) ---

  		  ILI9341_Draw_Filled_Circle(30, 145, 15, GREEN);

  		  Update_Bar(145, GREEN, COL_LT_GREEN, val_green);



  		  // --- ROW 3: BLUE (Initial) ---

  		  ILI9341_Draw_Filled_Circle(30, 200, 15, BLUE);

  		  Update_Bar(200, BLUE, COL_LT_BLUE, val_blue);


  		  // ---------------------------------------------------------

  		  // TOUCH LOOP

  		  // ---------------------------------------------------------
  		  uint32_t last_sensor_time = 0;
  		  while(1)

  		  {
  			  if (HAL_GetTick() - last_sensor_time > 3000) {

  					  // --- ส่วนของอาจารย์: อ่านค่าจาก I2C ---
  					  HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200); // Wake up
  					  HAL_I2C_Master_Transmit(&hi2c1, 0x5c<<1, cmdBuffer, 3, 200); // Send command
  					  HAL_Delay(1);
  					  HAL_I2C_Master_Receive(&hi2c1, 0x5c<<1, dataBuffer, 8, 200); // Receive data

  					  uint16_t Rcrc = dataBuffer[7] << 8 | dataBuffer[6];
  					  if (Rcrc == CRC16_2(dataBuffer, 6)) {
  						  // คำนวณค่า Temp
  						  uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
  						  t = temperature / 10.0;
  						  if ((dataBuffer[4] & 0x80) >> 7 == 1) t *= -1;

  						  // คำนวณค่า Humid
  						  uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
  						  h = humidity / 10.0;

  						  // --- ส่วนแสดงผลบนหน้าจอ TFT ---
  						  // 1. เคลียร์ค่าเก่า (วาดสี่เหลี่ยมขาวทับ)
  						  ILI9341_Draw_Filled_Rectangle_Coord(10, 20, 90, 50, WHITE);   // พื้นที่ Temp
  						  ILI9341_Draw_Filled_Rectangle_Coord(160, 20, 310, 50, WHITE);  // พื้นที่ Humid

  						  // 2. พิมพ์ค่าใหม่ลงไป (ใช้ %.1f ตามของเดิมคุณได้เลย)
  						  char t_str[15], h_str[15];
  						  sprintf(t_str, "%.1f C", t);
  						  sprintf(h_str, "%.1f %%RH", h);

  						  ILI9341_Draw_Text(t_str, 10, 25, BLACK, 2, WHITE);
  						  ILI9341_Draw_Text(h_str, 160, 25, BLACK, 2, WHITE);
  					  }
  					  last_sensor_time = HAL_GetTick();
  				  }

  			  // Poll for touch

  			  if(TP_Touchpad_Pressed())

  			  {

  				  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_SET);





  				  uint16_t pos[2];

  				  // Read Coordinates (Assumes Library returns mapped X/Y)

  				  if(TP_Read_Coordinates(pos) == TOUCHPAD_DATA_OK)

  				  {

  					  uint16_t x = pos[1];

  					  uint16_t y = pos[0];



  					  // --- CHECK HITBOXES ---

  					  // Circles are at X=30. We check touch X between 0 and 60.

  					  // Note: Adjust '60' if touch calibration is slightly off.

  					  //debug touch point
//  					  char debug_msg[30];
//  					  sprintf(debug_msg, "X:%dY:%d", x, y);
//  					  ILI9341_Draw_Text(debug_msg, 220, 220, BLUE, 1, WHITE);

  					  if (x < 45) // User touched the Left side (Button area)

  					  {

  						  // 1. RED BUTTON (Center Y=90) -> Hitbox 70 to 110

  						  // --- 1. RED BUTTON ---
  						  if (y > 140 && y < 170) {
  							  val_red += 10;
  							  if(val_red > 100) val_red = 0;
  							  Update_Bar(90, RED, COL_LT_RED, val_red);

  							  // อัปเดตสีม่วง (ตัวผสม)
  							  COL_MIX = Mix_RGB565(val_red, val_green, val_blue);
  							  ILI9341_Draw_Filled_Circle(120, 35, 20, COL_MIX);

  							  HAL_Delay(200);
  						  }

  						  // --- 2. GREEN BUTTON ---
  						  else if (y > 90 && y < 120) {
  							  val_green += 10;
  							  if(val_green > 100) val_green = 0;
  							  Update_Bar(145, GREEN, COL_LT_GREEN, val_green);

  							  // อัปเดตสีม่วง (ตัวผสม)
  							  COL_MIX = Mix_RGB565(val_red, val_green, val_blue);
  							  ILI9341_Draw_Filled_Circle(120, 35, 20, COL_MIX);

  							  HAL_Delay(200);
  						  }

  						  // --- 3. BLUE BUTTON ---
  						  else if (y > 30 && y < 60) {
  							  val_blue += 10;
  							  if(val_blue > 100) val_blue = 0;
  							  Update_Bar(200, BLUE, COL_LT_BLUE, val_blue);

  							  // อัปเดตสีม่วง (ตัวผสม)
  							  COL_MIX = Mix_RGB565(val_red, val_green, val_blue);
  							  ILI9341_Draw_Filled_Circle(120, 35, 20, COL_MIX);

  							  HAL_Delay(200);
  						  }

  					  }
  					  else if (x>=100 && x<=140){
  						  if (y>=190 && y<230){
  							  Page = 2;
  							  HAL_Delay(500);
  							  break;
  						  }
  					  }

  				  }

  			  }

  			  else

  			  {

  				  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  			  }

  			  // Small delay to prevent CPU hogging

  			  HAL_Delay(10);

  		  }
  	  case 2:
  		  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
  		  HAL_TIM_Base_Start_IT(&htim1);
  		  // 1. วาดรูปครั้งเดียวตอนเข้าหน้านี้
  		  ILI9341_Fill_Screen(WHITE);
  		  // วาดรูปของคุณ (ถ้าภาพเป็น 128x128 ฟังก์ชันเดิมอาจจะดึงค่าจนล้น ให้ระวังจุดนี้ด้วยครับ)
  		  ILI9341_Draw_Image((const char*)ME128x128, SCREEN_HORIZONTAL_1,20,50,95,128);
  		  ILI9341_Draw_Text("Group No.53", 130, 70, COL_MIX, 2, WHITE);

  		  ILI9341_Draw_Text("Pathorn", 130, 90, COL_MIX, 2, WHITE);

  		  ILI9341_Draw_Text("Somsaen", 130, 110, COL_MIX, 2, WHITE);

  		  ILI9341_Draw_Text("67015105", 130, 130, COL_MIX, 2, WHITE);
  		  // 2. ค้างไว้อยู่ในหน้านี้จนกว่าจะมีการเปลี่ยน Page
  		  while(Page == 2)
  		  {
  			  if(TP_Touchpad_Pressed())
  			  {
  				  // ถ้ากดหน้าจอ ให้เปลี่ยนกลับไปหน้า 1
  				  Page = 1;
  				  HAL_TIM_Base_Stop_IT(&htim1);
  				  count = 0;
  				  HAL_Delay(500); // หน่วงเวลาเล็กน้อยกันการกดเบิ้ล
  			  }
  			  // ไม่ต้องวาดรูปซ้ำในนี้ หน้าจอจะนิ่งและแสดงรูปค้างไว้
  			  HAL_Delay(50);
  		  }
  		  break;
  	  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CRC16_2(uint8_t *ptr, uint8_t length)
{
    uint16_t crc = 0xFFFF; //
    uint8_t s = 0x00;      //
    while (length--)
    {
        crc ^= *ptr++;     //
        for (s = 0; s < 8; s++)
        {
            if ((crc & 0x01) != 0)
            {
                crc >>= 1;   //
                crc ^= 0xA001; //
            }
            else crc >>= 1;  //
        }
    }
    return crc; //
}
/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // --- ส่วนคุมระบบหน้าจอ (TIM1) ---
    if (htim->Instance == TIM1) {
        // เปลี่ยนจากการใช้ HAL_Delay เป็นการนับรอบแทน
        count++;
        if (count >= 5) { // สมมติ Interrupt มาทุก 10ms, 50 รอบ = 0.5s
            count = 0;
            Page = 1;
            HAL_TIM_Base_Stop_IT(&htim1);
        }
    }

    // --- ส่วนปรับ Backlight เบื้องหลัง (TIM2) ---
//    if (htim->Instance == TIM2) {
//        // อ่านค่า ADC และปรับ PWM ทันที (ทำงานเบื้องหลังทุกหน้า)
//        uint32_t duty_pct = ((adc_values[0] * 80) / 4095) + 20;
//
//        // ตรวจสอบให้แน่ใจว่าใช้ Timer ตัวที่ต่อกับขา Backlight จริงๆ
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_pct * 10);
//    }
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

  while(1)

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

     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
