/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2_i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef float float32_t;

// Struktura do przechowywania parametrów regulatora
typedef struct
{
    float32_t Kp;
    float32_t Ki;
    float32_t Kd;
    float32_t dt;
} pid_parameters_t;

typedef struct
{
    pid_parameters_t p;
    float32_t previous_error, previous_integral;
} pid_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Granice możliwości zadania temperatury
#define minSetTemperature 20.0f
#define maxSetTemperature 50.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float temperature = -1000.0f;
float set_point = minSetTemperature;
char rxBuffer[12];
int16_t pwmValue = 0;
uint8_t rxIndex = 0;
uint8_t impulse = 0;
uint8_t pimpulse = 0;
pid_t pid1 = { .p.Kp = 1.54872967509146f, .p.Ki = 0.0184338237431332f, .p.Kd = 0.0f, .p.dt = 1.0f, .previous_error = 0, .previous_integral = 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  lcd16x2_i2c_init(&hi2c1); // Inicjalizacja wyświetlacza
  lcd16x2_i2c_cursorShow(true); // Włączenie kursora
  HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_4); // Inicjalizacja PWM
  HAL_TIM_Base_Start_IT(&htim7); // Inicjalizacja zegara głównego
  HAL_TIM_Encoder_Start_IT(&htim1, TIM_CHANNEL_ALL); // Inicjalizacja enkodera
  HAL_UART_Receive_IT(&huart3, (uint8_t *)&rxBuffer[rxIndex], 1); // Inicjalizacja odbierania komend przez UART

  // Ustawienie wstępne trybu na zadanie PWM
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  bool raise = true;
  uint16_t adcValue = 0;
  uint16_t setPWM = 0;
  float setTemperature = minSetTemperature;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Po naciśnięciu enkodera:
	  if (HAL_GPIO_ReadPin(SELECT_GPIO_Port, SELECT_Pin) == GPIO_PIN_SET && raise == GPIO_PIN_RESET)
	  {
		  switch(impulse)
		  {
			  case 2:
				  // Wejście w wybór wartości zadanej
				  impulse = 10;
				  break;

			  case 0:
			  case 4:
			  case 6:
				  // Wejście w wybór wartości zadanej
				  impulse = 12;
				  break;

			  case 10:
				  // Powrót do wyświetlania wartości
				  __HAL_TIM_SET_COUNTER(&htim1, 2);
				  impulse = 2;

				  // Obsługa wyświetlacza
				  lcd16x2_i2c_1stLine();
				  lcd16x2_i2c_printf("Wypelnienie PWM ");
				  lcd16x2_i2c_2ndLine();
				  lcd16x2_i2c_printf("wynosi: %4d%%.  ", pwmValue);
				  break;

			  case 11:
				  // Powrót do wyświetlania wartości
				  __HAL_TIM_SET_COUNTER(&htim1, 2);
				  impulse = 2;

				  // Zadanie wartości PWM
				  pwmValue = setPWM;
				  if (HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin) == GPIO_PIN_RESET)
				  {
					  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmValue);
				  }

				  // Zmiana trybu na zadanie PWM
				  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

				  // Obsługa wyświetlacza
				  lcd16x2_i2c_1stLine();
				  lcd16x2_i2c_printf("Wypelnienie PWM ");
				  lcd16x2_i2c_2ndLine();
				  lcd16x2_i2c_printf("wynosi: %4d%%.  ", pwmValue);
				  break;

			  case 12:
				  // Powrót do wyświetlania wartości
				  __HAL_TIM_SET_COUNTER(&htim1, 0);
				  impulse = 0;

				  if (temperature >= 0.0f && temperature <= 100.0f )
				  {
					  // Obsługa wyświetlacza
					  lcd16x2_i2c_1stLine();
					  lcd16x2_i2c_printf("Temp. wynosi:   ");
					  lcd16x2_i2c_2ndLine();
					  lcd16x2_i2c_printf("%6.2f stopni C.", temperature);
				  }
				  else
				  {
					  // Obsługa wyświetlacza
					  lcd16x2_i2c_1stLine();
					  lcd16x2_i2c_printf("Temperatura poza");
					  lcd16x2_i2c_2ndLine();
					  lcd16x2_i2c_printf("zakresem        ");
				  }
				  break;

			  case 13:
				  // Powrót do wyświetlania wartości
				  __HAL_TIM_SET_COUNTER(&htim1, 0);
				  impulse = 0;

				  // Zadanie wartości temperatury
				  set_point = setTemperature;

				  // Zmiana trybu na zadanie temperatury
				  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

				  if (temperature >= 0.0f && temperature <= 100.0f )
				  {
					  // Obsługa wyświetlacza
					  lcd16x2_i2c_1stLine();
					  lcd16x2_i2c_printf("Temp. wynosi:   ");
					  lcd16x2_i2c_2ndLine();
					  lcd16x2_i2c_printf("%6.2f stopni C.", temperature);
				  }
				  else
				  {
					  // Obsługa wyświetlacza
					  lcd16x2_i2c_1stLine();
					  lcd16x2_i2c_printf("Temperatura poza");
					  lcd16x2_i2c_2ndLine();
					  lcd16x2_i2c_printf("zakresem        ");
				  }
				  break;
		  }
	  }

	  switch(impulse)
	  {
	  	  case 10:
	  		  // Sczytanie wartości z potencjometru
	  		  HAL_ADC_Start(&hadc1);
	  		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  		  if (abs(adcValue - HAL_ADC_GetValue(&hadc1)) > 8)
	  		  {
	  			  adcValue = HAL_ADC_GetValue(&hadc1);
	  			  setPWM = adcValue*1000/4095;
	  		  }

	  		  // Obsługa wyświetlacza
	  		  lcd16x2_i2c_1stLine();
	  		  lcd16x2_i2c_printf("Zadac PWM %4d?:", setPWM);
	  		  lcd16x2_i2c_2ndLine();
	  		  lcd16x2_i2c_printf(" Tak  Nie       ");
	  		  lcd16x2_i2c_setCursor(1, 5);
		  	  break;

	  	  case 11:
	  		  // Sczytanie wartości z potencjometru
	  		  HAL_ADC_Start(&hadc1);
	  		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  		  if (abs(adcValue - HAL_ADC_GetValue(&hadc1)) > 8)
	  		  {
	  			  adcValue = HAL_ADC_GetValue(&hadc1);
	  			  setPWM = adcValue*1000/4095;
	  		  }

	  		  // Obsługa wyświetlacza
	  		  lcd16x2_i2c_1stLine();
	  		  lcd16x2_i2c_printf("Zadac PWM %4d?:", setPWM);
	  		  lcd16x2_i2c_2ndLine();
	  		  lcd16x2_i2c_printf(" Tak  Nie       ");
	  		  lcd16x2_i2c_setCursor(1, 0);
	  		  break;

	  	  case 12:
	  		// Sczytanie wartości z potencjometru
	  		  HAL_ADC_Start(&hadc1);
	  		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  		  if (abs(adcValue - HAL_ADC_GetValue(&hadc1)) > 8)
	  		  {
	  			  adcValue = HAL_ADC_GetValue(&hadc1);
	  			  setTemperature = (float)adcValue*(maxSetTemperature - minSetTemperature)/4095.0f + minSetTemperature;
	  		  }

	  		  // Obsługa wyświetlacza
	  		  lcd16x2_i2c_1stLine();
	  		  lcd16x2_i2c_printf("Zadac temp.  Tak");
	  		  lcd16x2_i2c_2ndLine();
	  		  lcd16x2_i2c_printf("%6.2f?:     Nie", setTemperature);
	  		  lcd16x2_i2c_setCursor(1, 12);
		  	  break;

	  	  case 13:
	  		// Sczytanie wartości z potencjometru
	  		  HAL_ADC_Start(&hadc1);
	  		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  		  if (abs(adcValue - HAL_ADC_GetValue(&hadc1)) > 8)
	  		  {
	  			  adcValue = HAL_ADC_GetValue(&hadc1);
	  			  setTemperature = (float)adcValue*(maxSetTemperature - minSetTemperature)/4095.0f + minSetTemperature;
	  		  }

	  		  // Obsługa wyświetlacza
	  		  lcd16x2_i2c_1stLine();
	  		  lcd16x2_i2c_printf("Zadac temp.  Tak");
	  		  lcd16x2_i2c_2ndLine();
	  		  lcd16x2_i2c_printf("%6.2f?:     Nie", setTemperature);
	  		  lcd16x2_i2c_setCursor(0, 12);
		  	  break;
	  }

	  raise = HAL_GPIO_ReadPin(SELECT_GPIO_Port, SELECT_Pin);

	  HAL_Delay(100);
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
// Funkcja do wysyłania danych przez UART
void UART_SendData(float temp)
{
    char buffer[50];
    int len = snprintf(buffer, sizeof(buffer), "temp=%.2f\n", temp);
    HAL_UART_Transmit(&huart3, (uint8_t *)buffer, len, 50);
}

/**
Calculation of discrete PID
* @param [in, out] s A pointer to PID parameters and history
* @param [in] setpoint Input setpoint value
* @param [in] measured Input measured value
* @return PID output value
**/
// Algorytm regulatora
float32_t calculate_discrete_pid(pid_t* pid, float32_t setpoint, float32_t measured) {
    float32_t u = 0.0f, P, I, D, error, integral, derivative;

    error = setpoint - measured;

    // proportional part
    P = pid->p.Kp * error;

    // integral part
    integral = pid->previous_integral + (error + pid->previous_error); // numerical integrator without anti-windup
    pid->previous_integral = integral;
    I = pid->p.Ki * integral * (pid->p.dt / 2.0f);

    // derivative part
    derivative = (error - pid->previous_error) / pid->p.dt; // numerical derivative without filter
    pid->previous_error = error;
    D = pid->p.Kd * derivative;

    // sum of all parts
    u = P + I + D; // without saturation

    return u;
}

// Pomiar temperatury, aktualizowanie danych o pomiar, regulacja PWM
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // Odczyt temperatury
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	uint16_t adcTemperature = HAL_ADC_GetValue(&hadc2);
    temperature = adcTemperature*3.3f/0.01f/4095.0f;

    // Sprawdzanie poprawności odczytu
    if (temperature >= 0.0f && temperature <= 100.0f )
    {
        // Wysyłanie danych przez UART
        UART_SendData(temperature);

        switch (impulse)
        {
        	case 0:
        		// Obsługa wyświetlacza
    	        lcd16x2_i2c_1stLine();
    	        lcd16x2_i2c_printf("Temp. wynosi:   ");
    	        lcd16x2_i2c_2ndLine();
    	        lcd16x2_i2c_printf("%6.2f stopni C.", temperature);
        		break;

        	case 2:
        		// Obsługa wyświetlacza
				lcd16x2_i2c_1stLine();
				lcd16x2_i2c_printf("Wypelnienie PWM ");
				lcd16x2_i2c_2ndLine();
				lcd16x2_i2c_printf("wynosi: %4d%%.  ", pwmValue);
        		break;

        	case 4:
        		// Obsługa wyświetlacza
				lcd16x2_i2c_1stLine();
				lcd16x2_i2c_printf("Uchyb temp.:    ");
				lcd16x2_i2c_2ndLine();
				lcd16x2_i2c_printf("%6.2f stopni C.", set_point - temperature);
        		break;

        	case 6:
        		// Obsługa wyświetlacza
				lcd16x2_i2c_1stLine();
				lcd16x2_i2c_printf("Zadana temp.:   ");
				lcd16x2_i2c_2ndLine();
				lcd16x2_i2c_printf("%6.2f stopni C.", set_point);
        		break;
        }

        if(HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin) == GPIO_PIN_RESET && HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin) == GPIO_PIN_RESET)
        {
			// Calculate controller output
			// Użycie algorytmu regulatora
			float pwm_duty_f = (1000.0f*calculate_discrete_pid(&pid1, set_point, temperature));
			// Nasycenie ograniczające do przedziału <0; 1000>
			if(pwm_duty_f < 0)
			{
				pwmValue = 0;
			}
			else if(pwm_duty_f > 1000.0f)
			{
				pwmValue = 1000;
			}
			else
			{
				pwmValue = (uint16_t)pwm_duty_f;
			}

			// Use control signal
			// użycie wyjścia regulatora, by zmienić impuls PWM (na użądzeniu wykonawczym)
			__HAL_TIM_SET_COMPARE (&htim2, TIM_CHANNEL_4, pwmValue); //PWM f = 1 kHz
        }
    }
    else
    {
        // Obsługa błędu (np. brak czujnika)
    	char error[] = "Temperatura poza zakresem – możliwe niepoprawne podłączenie czujnika.\n";
    	HAL_UART_Transmit(&huart3, (uint8_t *)error, strlen(error), 50); // Wysłanie błędu

    	if (impulse == 0 || impulse == 4 || impulse == 6)
		{
    		// Obsługa wyświetlacza
			lcd16x2_i2c_1stLine();
			lcd16x2_i2c_printf("Temperatura poza");
			lcd16x2_i2c_2ndLine();
			lcd16x2_i2c_printf("zakresem        ");
		}
    	else if(impulse == 2)
		{
    		// Obsługa wyświetlacza
			lcd16x2_i2c_1stLine();
			lcd16x2_i2c_printf("Wypelnienie PWM ");
			lcd16x2_i2c_2ndLine();
			lcd16x2_i2c_printf("wynosi: %4d%%.  ", pwmValue);
		}
    }
    // 4 - Toggle LED1
    HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
}

// Obsługa obrotów enkodera (poruszanie się po interfejsie)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (__HAL_TIM_GET_COUNTER(&htim1)%2 == 0 && __HAL_TIM_GET_COUNTER(&htim1) != pimpulse)
	{
		// Zmiana widocznej wartości
		if (impulse < 10)
		{
    		impulse = __HAL_TIM_GET_COUNTER(&htim1);
    		if (HAL_GPIO_ReadPin(LD2_GPIO_Port, LD2_Pin) == GPIO_PIN_SET)
    		{
        		impulse %= 4;
    		}
		}
	    switch(impulse)
	    {
	    	case 0:
	    		if (temperature >= 0.0f && temperature <= 100.0f )
	    		    {
	    				// Obsługa wyświetlacza
	    		        lcd16x2_i2c_1stLine();
	    		        lcd16x2_i2c_printf("Temp. wynosi:   ");
	    		        lcd16x2_i2c_2ndLine();
	    		        lcd16x2_i2c_printf("%6.2f stopni C.", temperature);
	    		    }
	    		    else
	    		    {
	    		    	// Obsługa wyświetlacza
	    		        lcd16x2_i2c_1stLine();
	    		        lcd16x2_i2c_printf("Temperatura poza");
	    		        lcd16x2_i2c_2ndLine();
	    		        lcd16x2_i2c_printf("zakresem        ");
	    		    }
	    		break;

	    	case 2:
	    			// Obsługa wyświetlacza
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Wypelnienie PWM ");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("wynosi: %4d%%.  ", pwmValue);
	    		break;

	    	case 4:
	    		if (temperature >= 0.0f && temperature <= 100.0f )
	    		{
	    			// Obsługa wyświetlacza
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Uchyb temp.:    ");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("%6.2f stopni C.", set_point - temperature);
	    		}
	    		else
				{
	    			// Obsługa wyświetlacza
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Temperatura poza");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("zakresem        ");
				}
		        break;

	    	case 6:
	    		if (temperature >= 0.0f && temperature <= 100.0f )
	    		{
	    			// Obsługa wyświetlacza
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Zadana temp.:   ");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("%6.2f stopni C.", set_point);
	    		}
	    		else
				{
	    			// Obsługa wyświetlacza
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Temperatura poza");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("zakresem        ");
				}
	    		break;

	    	// Dalsze case’y – zmiana Tak/Nie
	    	case 10:
	    		impulse = 11;
	    		break;

	    	case 11:
	    		impulse = 10;
	    		break;

	    	case 12:
	    		impulse = 13;
	    		break;

	    	case 13:
	    		impulse = 12;
	    		break;
	    }
	}
	pimpulse = __HAL_TIM_GET_COUNTER(&htim1);
}

// Odbieranie wartości zadanych z UART-a
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Sprawdzenie, czy odebrany znak to '\n'
	if (rxBuffer[rxIndex] == ';') {
		rxBuffer[rxIndex] = '\0'; // Zakończenie stringa

		// Sprawdzenie, czy komenda zaczyna się od "PWM1="
		if (strncmp(rxBuffer, "PWM1=", 5u) == 0) {
			int tpwmValue = atoi(&rxBuffer[5]); // Konwersja na liczbę

			if (tpwmValue >= 0 && tpwmValue <= 1000) { // Zakres 0-1000%o
				// Ustawienie wypełnienia PWM (TIM2, kanał 4)
				pwmValue = tpwmValue;
				if (HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin) == GPIO_PIN_RESET)
				{
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmValue);
				}

				// Zmiana trybu na zadanie PWM
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

				// Wyświetlanie PWM, jeśli wyświetlało uchyb lub temperaturę zadaną
				if(impulse == 4 || impulse == 6)
				{
					  __HAL_TIM_SET_COUNTER(&htim1, 2);
					  impulse = 2;
				}

				if (impulse == 2)
				{
					// Obsługa wyświetlacza
					lcd16x2_i2c_1stLine();
					lcd16x2_i2c_printf("Wypelnienie PWM ");
					lcd16x2_i2c_2ndLine();
					lcd16x2_i2c_printf("wynosi: %4d%%.  ", pwmValue);
				}
			}
			else
			{
				char error[] = "Brak efektu. PWM1 musi wynosić między 0 a 1000.\n";
				HAL_UART_Transmit(&huart3, (uint8_t *)error, strlen(error), 50);
			}
		}

		// Sprawdzenie, czy komenda zaczyna się od "PID1="
		else if(strncmp(rxBuffer, "PID1=", 5u) == 0)
		{
			int tset_point = atof(&rxBuffer[5]); // Konwersja na liczbę

			if (tset_point >= minSetTemperature && tset_point <= maxSetTemperature) { // Zakres minSetTemperature-maxSetTemperature
				// Ustawienie wypełnienia PWM (TIM2, kanał 4)
				set_point = tset_point;

				// Zmiana trybu na zadanie temperatury
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

				switch (impulse)
				{
					case 4:
			    		if (temperature >= 0.0f && temperature <= 100.0f )
			    		{
			    			// Obsługa wyświetlacza
							lcd16x2_i2c_1stLine();
							lcd16x2_i2c_printf("Uchyb temp.:    ");
							lcd16x2_i2c_2ndLine();
							lcd16x2_i2c_printf("%6.2f stopni C.", set_point - temperature);
						}
						else
						{
							// Obsługa wyświetlacza
							lcd16x2_i2c_1stLine();
							lcd16x2_i2c_printf("Temperatura poza");
							lcd16x2_i2c_2ndLine();
							lcd16x2_i2c_printf("zakresem        ");
						}
						break;

					case 6:
			    		if (temperature >= 0.0f && temperature <= 100.0f )
			    		{
			    			// Obsługa wyświetlacza
							lcd16x2_i2c_1stLine();
							lcd16x2_i2c_printf("Zadana temp.:   ");
							lcd16x2_i2c_2ndLine();
							lcd16x2_i2c_printf("%6.2f stopni C.", set_point);
						}
						else
						{
							// Obsługa wyświetlacza
							lcd16x2_i2c_1stLine();
							lcd16x2_i2c_printf("Temperatura poza");
							lcd16x2_i2c_2ndLine();
							lcd16x2_i2c_printf("zakresem        ");
						}
						break;
				}
			}
			else
			{
				char error[100];
				sprintf(error, "Brak efektu. PID1 musi wynosić między %.2f a %.2f.\n", minSetTemperature, maxSetTemperature);
				HAL_UART_Transmit(&huart3, (uint8_t *)error, strlen(error), 50);
			}
		}
		else
		{
			char error[] = "Podana składnia nie przynosi efektu.\n";
			HAL_UART_Transmit(&huart3, (uint8_t *)error, strlen(error), 50);
		}
		rxIndex = 0; // Reset indeksu
	} else {
		rxIndex++;
		if (rxIndex >= sizeof(rxBuffer)) {
			rxIndex = 0; // Zabezpieczenie przed przepełnieniem bufora
		}
	}
	// Kontynuacja odbioru kolejnego znaku
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&rxBuffer[rxIndex], 1);
}

// Pauzowanie programu przyciskiem
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin) == GPIO_PIN_SET)
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwmValue);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	}
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
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
