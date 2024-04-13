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
#include "ssd1306_fonts.h"
#include "ssd1306_tests.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS18B20_PORT GPIOB
#define DS18B20_PIN GPIO_PIN_14
#define ADC_BUF_SIZE 4
#define VREF 3310
#define DELAY_COND 6
#define DELAY_MOIST 1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t i=0;
uint8_t j=0;
uint32_t counter=0;
int32_t PWM_loop=0;
float final_average_cond=0;
float av_cond=0;
float conductivity=0;
float av_moist=0;
float percentage_moist=0;
float percentage_moist2=0;
float Temp=0;
float Temperature=0;
uint8_t Presence=0;
uint8_t Temp_byte1,Temp_byte2;
uint16_t TEMP =0;;
uint16_t TempInt;
char bufferConduct[200];
char bufferMoist[200];
char bufferTemp[200];
char bufferDs18b20[200];
char bufferTempInside[200];
char bufferVref[200];
char bufferDS18B20[200];
char adcbuffer1[20];
char adcbuffer2[20];
uint16_t conductraw=0;
uint16_t tempraw=0;
uint16_t moistureraw=0;
uint32_t value[3]; // adc valuyes
float temp1;//

uint32_t adc_buffer[ADC_BUF_SIZE];
float voltage_buffer[ADC_BUF_SIZE];
uint8_t adc_ready=0;
uint8_t pwmflagfinished=0;
volatile uint8_t counterflagPWM=0;
uint32_t cond;
uint32_t vref_int_mv;
uint32_t moist;
uint32_t inTempC;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    counter++;

    if (counter>1000)
    {
    	counterflagPWM=1;

    }

  }
}


void PWM_COND(){


	for(PWM_loop=0;PWM_loop<20;PWM_loop++){
	      HAL_Delay(DELAY_COND);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
		  HAL_Delay(DELAY_COND);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
		  HAL_Delay(2);

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 1);
		  HAL_Delay(DELAY_COND);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, 0);
		  HAL_Delay(DELAY_COND);


	}

}
void PWM_MOIST(){

	  counterflagPWM=0;
	  counter=0;



	    // Set PB3 as output
	    GPIOB->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3); // Clear bits
	    GPIOB->CRL |= GPIO_CRL_MODE3_0; // Set pin mode to general purpose output push-pull 10MHz
//


		do
		{

			  GPIOB->ODR ^= GPIO_ODR_ODR3;
//			  delay(4);    // 100Khz  0.6V 100%		2.6V 0%
//			  delay(1);    // 230Khz  0.6V 100%		2.6V 0%
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
			  __NOP();
// 325khz


		}while(counterflagPWM==0);
	    // Toggle PB3 using inline assembly
//	    __asm__ volatile (
////	            "ldr r0, =0x40010C0C \n\t" // GPIOB ODR
////	            "ldr r1, =1<<3 \n\t"       // Turn on PB3
////	            "ldr r2, =0 \n\t"          // Turn off PB3
////	            ".loop: \n\t"
////	            "str r1, [r0] \n\t"        // Set PB3
////	            "str r2, [r0] \n\t"        // Clear PB3
////	            "b .loop \n\t"
//
////	        "ldr r0, =0x40010C0C \n\t" // GPIOB ODR
////	        "ldr r1, =1<<3 \n\t"       // Turn on PB3
////
////	        "ldr r2, =0 \n\t"          // Turn off PB3
////	        ".loop: \n\t"
////	        "ldr r3, =counterflagPWM \n\t" // Load the address of check_variable
////	        "ldr r3, [r3] \n\t"            // Load the value of check_variable
////	        "cmp r3, #1 \n\t"              // Compare the value with 1
////	        "beq .endloop \n\t"            // Branch if equal to 1
////	        "str r1, [r0] \n\t"            // Set PB3
////	        "str r2, [r0] \n\t"            // Clear PB3
////	        "b .loop \n\t"
////	        ".endloop: \n\t"
//
//
//	    		"ldr r0, =0x40010C0C \n\t" // GPIOB ODR
//	    		"ldr r1, =1<<3 \n\t"       // Turn on PB3
//	    		"ldr r2, =0 \n\t"          // Turn off PB3
//	    		"ldr r4, =counterflagPWM \n\t" // Load the address of counterflagPWM
//	    		"ldr r4, [r4] \n\t"            // Load the value of counterflagPWM
//	    		"cmp r4, #1 \n\t"              // Compare the value with 1
//	    		"beq .endloop \n\t"            // Branch if equal to 1
//
//	    		// Calculate the number of cycles needed for a 50% duty cycle
//	    		"mov r5, #5000 \n\t"  // 5000 cycles for half period at 1MHz (adjust according to your frequency)
//
//	    		// Loop for 50% duty cycle
//	    		".pwmloop: \n\t"
//	    		"str r1, [r0] \n\t"            // Set PB3 (High)
//	    		"subs r5, #1 \n\t"             // Decrement cycle counter
//	    		"bne .pwmloop \n\t"            // Branch if not equal to zero
//	    		"str r2, [r0] \n\t"            // Clear PB3 (Low)
//
//	    		"b .loop \n\t"  // Branch back to the main loop
//
//	    		".endloop: \n\t"
//	    );





	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
}

float voltage_to_conductivity(float voltage) { //it needs fix
    // Define the calibration points
    float voltage_low = 3.3; // Lowest voltage
    float conductivity_low = 0.0; // Corresponding conductivity

    float voltage_high = 0.6; // Highest voltage
    float conductivity_high = 100.0; // Corresponding conductivity

    // Calculate the slope (m)
    float slope = (conductivity_high - conductivity_low) / (voltage_high - voltage_low);

    // Linear interpolation formula
    return slope * (voltage - voltage_low) + conductivity_low;
}


void SSD1306_INITS()
{
	  ssd1306_Init();
	  ssd1306_UpdateScreen();
	  HAL_Delay(50);
	  ssd1306_Fill(0);
	  ssd1306_UpdateScreen();
	  ssd1306_WriteString("Telectronio",Font_11x18,1);
	  ssd1306_UpdateScreen();
	  HAL_Delay(1000);
	  ssd1306_Fill(0);
	  ssd1306_UpdateScreen();
	  HAL_Delay(50);
	  ssd1306_SetCursor(0, 0);
	  ssd1306_WriteString("Soil", Font_11x18,1);
	  ssd1306_SetCursor(0, 19);
	  ssd1306_WriteString("Meaurement", Font_11x18,1);
	  ssd1306_SetCursor(0, 38);
	  ssd1306_WriteString("Version1", Font_11x18,1);
	  ssd1306_UpdateScreen();
	  HAL_Delay(1000);
	  ssd1306_Fill(0);
	  ssd1306_UpdateScreen();
}

void delay(uint16_t delay){
	__HAL_TIM_SET_COUNTER(&htim1,0);
	while(__HAL_TIM_GET_COUNTER(&htim1)<delay);
}


void Set_Pin_Output(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin=GPIO_Pin;
	GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx,&GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin=GPIO_Pin;
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull=GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx,&GPIO_InitStruct);
}
void Set_Pin_InputPU(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct={0};
	GPIO_InitStruct.Pin=GPIO_Pin;
	GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull=GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx,&GPIO_InitStruct);
}
uint8_t DS18B20_Start (void)
{
	uint8_t Response=0;
	Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);
	HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,0);
	delay(480);
	Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);
	delay(60);

	if(!(HAL_GPIO_ReadPin (DS18B20_PORT,DS18B20_PIN))) Response =1;
	else Response = -1;
	delay(480);
	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);

	for(int i=0; i<8; i++)
	{
		if((data&(1<<i))!=0)
		{
			//write 1
			Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,0);
			delay(1);
			Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);
			delay(60);
		}
		else
		{
			//write 0

			Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);
			HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,0);
			delay(60);
			Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);

		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);

	for(int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT,DS18B20_PIN);
		HAL_GPIO_WritePin(DS18B20_PORT,DS18B20_PIN,0);
		delay(1);
		Set_Pin_Input(DS18B20_PORT,DS18B20_PIN);
		if(HAL_GPIO_ReadPin(DS18B20_PORT,DS18B20_PIN))
		{
			value |= 1<<i; //read=1
		}
		delay(60);
	}
	return value;
}



float DS18B20_GetTemp(void)
{
    Presence = DS18B20_Start();

    HAL_Delay(1);

    DS18B20_Write(0xCC); // SKIP ROM
    DS18B20_Write(0x44); // Convert T

    HAL_Delay(1);

    Presence = DS18B20_Start();

    HAL_Delay(1);

    DS18B20_Write(0xCC);
    DS18B20_Write(0xBE); // Read scratch pad

    Temp_byte1 = DS18B20_Read();
    Temp_byte2 = DS18B20_Read();
    TEMP = (Temp_byte2 << 8) | Temp_byte1;
    Temperature = (float)TEMP / 16;



    HAL_Delay(10);

    return Temperature;
}

void ADC_CH1(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}
void ADC_CH2(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}
void ADC_CH3(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}
float adc_value_to_voltage(uint16_t adc_value) {
    return (adc_value / 4095.0) * VREF; // 4095 for 12-bit resolution
}


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
  MX_I2C1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_INITS();
  HAL_ADC_Init(&hadc2);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim4);

//  PWM_BEGIN_MOIST();
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);  // Set priority and subpriority as needed
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  GPIOB->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
  GPIOB->CRL |= GPIO_CRL_MODE3;  // Output mode, max speed 50 MHz



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //MEASURE SEQUENTIAL
	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)==1)
	  {	  av_cond=0;
	  	  av_moist=0;
		  ssd1306_Fill(0);
		  ssd1306_UpdateScreen();
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString("Conductivity...",Font_7x10,1);
		  ssd1306_UpdateScreen();
		  ssd1306_Fill(0);


		  ADC_CH1();

		  float av_cond_sum = 0;

		  for(int j = 0; j < 15; j++) {
		      PWM_COND();
		      HAL_ADC_Start(&hadc2);
		      float av_cond = 0; // Initialize av_cond for each iteration

		      for(int i = 0; i < 50; i++) {
		          HAL_ADC_PollForConversion(&hadc2, 1);
		          adc_buffer[0] = HAL_ADC_GetValue(&hadc2);
		          voltage_buffer[0] = adc_value_to_voltage(adc_buffer[0]);
		          av_cond += voltage_buffer[0] / 50; // Accumulate the value
		      }

		      HAL_ADC_Stop(&hadc2);
		      HAL_Delay(100);

		      // Add the average of this iteration to av_cond_sum
		      av_cond_sum += av_cond;
		  }

		  // Calculate the final average
		  final_average_cond = av_cond_sum / 15;


		  HAL_Delay(1000);
		  ssd1306_Fill(0);
		  ssd1306_UpdateScreen();
		  ssd1306_SetCursor(0, 0);
		  ssd1306_WriteString("Moisture...",Font_7x10,1);
		  ssd1306_UpdateScreen();
		  ssd1306_Fill(0);
		  PWM_MOIST();
		  ADC_CH2();

		  percentage_moist2 = 0; // Initialize averaged percentage variable
		  percentage_moist=0;
		  for (j = 0; j < 15; j++) {
		      av_moist = 0; // Reset av_moist for each iteration

		      PWM_MOIST();
		      PWM_MOIST();
		      PWM_MOIST();
		      PWM_MOIST();
		      HAL_ADC_Start(&hadc2);
		      for (i = 0; i < 50; i++) {
		          HAL_ADC_PollForConversion(&hadc2, 1);
		          adc_buffer[1] = HAL_ADC_GetValue(&hadc2);
		          voltage_buffer[1] = adc_value_to_voltage(adc_buffer[1]);
		          av_moist += voltage_buffer[1] / 50;
		      }
		      HAL_ADC_Stop(&hadc2);

		      // Calculate percentage_moist for this iteration
		      percentage_moist = (100 - (av_moist / 1900) * 100);

		      // Accumulate the calculated percentage
		      percentage_moist2 += percentage_moist;
		      HAL_Delay(2); // Add delay between measurements if necessary
		  }

		  // Calculate the average of percentage_moist over 5 measurements
		  percentage_moist2 /= 15;
		  // Manipulate the last value of percentage_moist2
		  if (percentage_moist2 > 100) {
		      percentage_moist2 = 100;
		  } else if (percentage_moist2 < 0) {
		      percentage_moist2 = 0;
		  }

	  }




	  Temp=DS18B20_GetTemp();

//	  //CONDUCTIVITY
//	  ADC_CH1();
//	  HAL_ADC_Start(&hadc2);
//	  HAL_Delay(2);
//	  HAL_ADC_PollForConversion(&hadc2, 100);
//	  adc_buffer[0]=HAL_ADC_GetValue(&hadc2);
//	  voltage_buffer[0]=adc_value_to_voltage(adc_buffer[0]);
//	  HAL_ADC_Stop(&hadc2);
//	  HAL_Delay(2);

//	  //MOISTURE
//	  ADC_CH2();
//	  HAL_ADC_Start(&hadc2);
//	  HAL_Delay(2);
//	  HAL_ADC_PollForConversion(&hadc2, 100);
//	  adc_buffer[1]=HAL_ADC_GetValue(&hadc2);
//	  voltage_buffer[1]=adc_value_to_voltage(adc_buffer[1]);
//	  HAL_ADC_Stop(&hadc2);
//	  HAL_Delay(2);

	  //NTC BOARD TEMP
	  ADC_CH3();
	  HAL_ADC_Start(&hadc2);
	  HAL_Delay(2);
	  HAL_ADC_PollForConversion(&hadc2, 100);
	  adc_buffer[2]=HAL_ADC_GetValue(&hadc2);
	  voltage_buffer[2]=adc_value_to_voltage(adc_buffer[2]);
	  HAL_ADC_Stop(&hadc2);
	  HAL_Delay(2);

	  ssd1306_SetCursor(0, 0);
	  sprintf(bufferConduct,"Cond %.2fV %.f",final_average_cond,conductivity);
	  ssd1306_WriteString(bufferConduct,Font_6x8,1);
	  ssd1306_SetCursor(0, 11);
	  sprintf(bufferMoist,"Moist %.1fV %.1f%%",av_moist,percentage_moist2);
	  ssd1306_WriteString(bufferMoist,Font_6x8,1);
	  ssd1306_SetCursor(0, 21);
	  sprintf(bufferTemp,"Temp MCU %.2fV",voltage_buffer[2]);
	  ssd1306_WriteString(bufferTemp,Font_6x8,1);
	  ssd1306_SetCursor(0, 31);
	  sprintf(bufferDs18b20,"ds18b20 %.2fC",Temp);
	  ssd1306_WriteString(bufferDs18b20,Font_6x8,1);
	  ssd1306_UpdateScreen();



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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_2;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 47999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 255;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_Delay(100); // Introduce a delay (adjust as needed)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Alternate function push-pull
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
