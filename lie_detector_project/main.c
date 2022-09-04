/*  ******************************************************************************
  Names: Alfred Moussa --&&-- Jacob Jackson
  Description: Project
  Course: ECE533 - Embedded Systems
  Board: STM32L552ZEQ, NUECLEO L552
  Date: March 24th, 2022
  ******************************************************************************/

#include "ECG_sensor.h"
#include "lcd_init.h"
#include "Timers.h"

#include "main.h"
#include "SPI_Graphical_lcd.h"
#include "Spi_init.h"
#include "MPU9808_I2C.h"
#include "ECG_sensor.h"
#include "Bluetooth_HC05_LPUART.h"
#include <string.h>
#include "algorthim.h"
#include "HC_05_Bluetooth_USART2.h"
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "bitmap.h"
#include <lie_anim.h>

void Ssd1306();
static void MX_I2C1_Init(void);
static void MX_GPIO_Init(void);
I2C_HandleTypeDef hi2c1;
void test_anim_oled();
void test_anim_oled_2();
void temp_data_Recieve();
void temp_compare_data_Recieve();
void Display_Liar();



int main(void){
	button1 = 0;
	__disable_irq();
	setClks();
	LCD_init();
	Config_PB13 ();

	USART2GPIOinit();
	USART2init();
	enableLED();
	Ssd1306();
	//LPUART1init();
	//Tim2_init();
	//ADC1init();  // ADC1-> is the ECG sensor
	//ADC2init_temp();  // ADC2-> is the Temperature sensor
	__enable_irq();

	while(Temp_Baseline==0 || ECG_Baseline ==0){
	temp_data_Recieve(); // Recieve the data and store it to Display on LCD: Baseline Temp = , Baseline ECG
	}
	bitflip(GPIOB->ODR, 7); 						//flip Green LED
	Temperature_LCD1();// Display on LCD: Baseline Temp Average

	delay1_ms(200);
	ECG_LCD2(); //Display on LCD: Baseline ECG Average
	delay1_ms(200);



	while(1){
		//delay1_ms(5000);
		while(Temp_Compare==0 || ECG_Compare ==0){
			temp_compare_data_Recieve(); // Recieve the data and store it to Display on LCD: Baseline Temp = , Baseline ECG
			}
		//button1 = 0; // reset the flag
		if(Temp_Compare > Temp_Baseline+2 || ECG_Compare > (ECG_Baseline * 1.05)){
				SSD1306_Clear(); // liar should be empty
				//test_lcd();// clear the Lcd
				Temperature_LCD3();
				ECG_LCD4();
			//while(~button1){
			for(int i; i<50; i++){
				bitflip(GPIOA->ODR, 9); 						//flip Green LED
				bitflip(GPIOB->ODR, 7); 						//flip Green LED
				bitflip(GPIOC->ODR, 7); 						//flip Green LED
				Display_Liar();
				delay1_ms(200);
				}
		}
			SSD1306_Clear();// clear oled
			button1 = 0; // reset the flag
			test_lcd();
			ECG_new=ECG_Compare;
			Temperature_LCD3();
			delay1_ms(300);
			ECG_LCD4();
			delay1_ms(300);
			button1 = 0; // reset the flag
			Temp_Compare=0;
			ECG_Compare =0;
		}


}



void EXTI13_IRQHandler(){

	button1=1;
	EXTI->FPR1|=(1<<13);//only triggered with negetive edge of the push botton


}



/*

ECG TEST  --> PAssed
int main(){
	__disable_irq();
	setClks();
	LPUART1init();
	Tim2_init();
//	DAC1_Enable();
	ADC1init();
	//ADC2init_temp();
	__enable_irq();
	while(1){

	}
}

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

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0010061A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
void Display_Liar(){

    /// lets print some string

      SSD1306_GotoXY (0,0);
      SSD1306_Puts ("LIAR", &Font_11x18, 1);
      SSD1306_GotoXY (10, 30);
      SSD1306_Puts ("  LIAR :(  :( ", &Font_11x18, 1);
      SSD1306_UpdateScreen(); //display

    //  HAL_Delay (200);
}
void Ssd1306(){
	HAL_Init();

	  /* USER CODE BEGIN Init */

	  /* USER CODE END Init */

	  /* Configure the system clock */

	  /* USER CODE BEGIN SysInit */

	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	  MX_GPIO_Init();
	  MX_I2C1_Init();
	  /* USER CODE BEGIN 2 */
	  SSD1306_Init();  // initialise

	    /// lets print some string

	      SSD1306_GotoXY (0,0);
	      SSD1306_Puts ("LIE", &Font_11x18, 1);
	      SSD1306_GotoXY (10, 30);
	      SSD1306_Puts ("  DETECTOR :) OR :( ", &Font_11x18, 1);
	      SSD1306_UpdateScreen(); //display

	      HAL_Delay (1000);


	      SSD1306_ScrollRight(0,7);  // scroll entire screen
	      HAL_Delay(200);  // 2 sec

	      SSD1306_ScrollLeft(0,7);  // scroll entire screen
	      HAL_Delay(200);  // 2 sec

	      SSD1306_Stopscroll();
	      SSD1306_Clear();

	      SSD1306_DrawBitmap(0,0,logo, 128, 64, 1);

	      SSD1306_UpdateScreen();

	      HAL_Delay(200);
/*
	      SSD1306_ScrollRight(0x00, 0x0f);    // scroll entire screen right
	      SSD1306_UpdateScreen();

	      HAL_Delay (500);

	      SSD1306_ScrollLeft(0x00, 0x0f);  // scroll entire screen left

	      HAL_Delay (500);

	      SSD1306_Scrolldiagright(0x00, 0x0f);  // scroll entire screen diagonal right

	      HAL_Delay (500);

	      SSD1306_Scrolldiagleft(0x00, 0x0f);  // scroll entire screen diagonal left

	      HAL_Delay (500);*/

	     // SSD1306_Stopscroll();   // stop scrolling. If not done, screen will keep on scrolling


	  /*    SSD1306_InvertDisplay(1);   // invert the display

	      HAL_Delay(500);

	      SSD1306_InvertDisplay(0);  // normalize the display
	  //  HAL_Delay(500);
*/

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, UCPD_DBN_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UCPD_FLT_Pin */
  GPIO_InitStruct.Pin = UCPD_FLT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UCPD_DBN_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = UCPD_DBN_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}
void test_anim_oled(){
	 	 	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_1,128,64,1);
	    	  SSD1306_UpdateScreen();

	    	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_2,128,64,1);
	    	  SSD1306_UpdateScreen();

	    	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_3,128,64,1);
	    	  SSD1306_UpdateScreen();

	    	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_4,128,64,1);
	    	  SSD1306_UpdateScreen();
}

void test_anim_oled_2(){
	 	 	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_1,128,32,1);
	    	  SSD1306_UpdateScreen();

	    	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_2,128,32,1);
	    	  SSD1306_UpdateScreen();

	    	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_3,128,32,1);
	    	  SSD1306_UpdateScreen();

	    	  SSD1306_Clear();
	    	  SSD1306_DrawBitmap(0,0,lie_detector_4,128,32,1);
	    	  SSD1306_UpdateScreen();
}
void temp_data_Recieve(){
	while (!((USART2->ISR)&(1<<5)));   // Check if there is a packet received
	character = (USART2->RDR)&0xff;									//Read Transmitted Val
		switch(character){
					case 't': {											//If we receive a t, the next value is a temperature
						while (!((USART2->ISR)&(1<<5)));
						Temp_Baseline = (USART2->RDR)&0xff;	//Read in Temp
						bitflip(GPIOC->ODR, 7); 						//flip Green LED
					}
					break;
					case 'e':{											//Same for e
						while (!((USART2->ISR)&(1<<5)));
						ECG_Baseline = (USART2->RDR)&0xff;
						bitflip(GPIOA->ODR, 9); 						//flip Green LED
					}
					break;
					default:{
						temp_data_Recieve();
						//character = (USART2->RDR)&0xff;
					}
					break;
		}
					}





void temp_compare_data_Recieve(){
	while (!((USART2->ISR)&(1<<5)));   // Check if there is a packet received
	character = (USART2->RDR)&0xff;									//Read Transmitted Val
		switch(character){
					case 't': {											//If we receive a t, the next value is a temperature
						while (!((USART2->ISR)&(1<<5)));
						Temp_Compare = (USART2->RDR)&0xff;	//Read in Temp
						bitflip(GPIOC->ODR, 7); 						//flip Green LED
					}
					break;
					case 'e':{											//Same for e
						while (!((USART2->ISR)&(1<<5)));
						ECG_Compare = (USART2->RDR)&0xff;
						bitflip(GPIOA->ODR, 9); 						//flip Green LED
					}
					break;
					default:{
						temp_compare_data_Recieve();
						//character = (USART2->RDR)&0xff;
					}
					break;
		}
					}
