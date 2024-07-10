#include "main.h"
#include <math.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

#define FLASH_USER_START_ADDR 0x08040000U
#define BIAS_THRESHOLD 2670
#define ZERO_CROSSING_THRESHOLD -520
#define NUM_SAMPLES 16000
#define NUM_FRAMES 100
#define NUM_REFERENCES 10
#define NUM_FEATURES 10
#define BUFFER_SIZE (NUM_FEATURES - 1)

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);

void ADC_READ(ADC_HandleTypeDef *hadc, short int *value_read) {
	HAL_ADC_Start(hadc);
	if (HAL_ADC_PollForConversion(hadc, 1) == HAL_OK) {
		*value_read = (HAL_ADC_GetValue(hadc) - BIAS_THRESHOLD);
	}
	HAL_ADC_Stop(hadc);
}

float calculate_distance(int32_t ref[NUM_FEATURES][NUM_FRAMES],
		int32_t test[NUM_FEATURES][NUM_FRAMES]) {
	float total_distance = 0.0;
	float distance;

	for (uint8_t i = 0; i < NUM_FRAMES; i++) {
		distance = 0.0;

		for (uint8_t j = 0; j < NUM_FEATURES; j++)
			distance += pow(ref[j][i] - test[j][i], 2);

		total_distance += sqrt(distance);
	}

	return total_distance;
}

HAL_StatusTypeDef Flash_Write(uint32_t start_address, int32_t *data,
		uint32_t size) {
	HAL_StatusTypeDef status;
	uint32_t Address = start_address;

	// Unlock the Flash
	HAL_FLASH_Unlock();

	// Erase the Flash memory sector(s) if necessary
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError = 0;

	// Determine the sector to start erasing
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FLASH_SECTOR_6; // Adjust this according to the address
	EraseInitStruct.NbSectors = 1;

	status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
	if (status != HAL_OK) {
		HAL_FLASH_Lock();
		return status;
	}

	// Write data to Flash memory
	for (uint32_t i = 0; i < size; i++) {
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data[i]);
		if (status != HAL_OK) {
			HAL_FLASH_Lock();
			return status;
		}
		Address += 4; // Increment address by word size (4 bytes)
	}

	// Lock the Flash
	HAL_FLASH_Lock();

	return HAL_OK;
}

void Flash_Read(uint32_t start_address, int32_t *data, uint32_t size) {
	uint32_t Address = start_address;

	// Read data from Flash memory
	for (uint32_t i = 0; i < size; i++) {
		data[i] = *(__IO int32_t*) Address;
		Address += 4; // Increment address by word size (4 bytes)
	}
}

void flatten_array(int32_t source[NUM_REFERENCES + 1][NUM_FEATURES][NUM_FRAMES],
		int32_t *flattened) {
	for (int i = 0, index = 0; i < NUM_REFERENCES; i++) {
		for (int j = 0; j < NUM_FEATURES; j++) {
			for (int k = 0; k < NUM_FRAMES; k++) {
				flattened[index++] = source[i][j][k];
			}
		}
	}
}

void unflatten_array(int32_t *flattened,
		int32_t destination[NUM_REFERENCES + 1][NUM_FEATURES][NUM_FRAMES]) {
	for (int i = 0, index = 0; i < NUM_REFERENCES; i++) {
		for (int j = 0; j < NUM_FEATURES; j++) {
			for (int k = 0; k < NUM_FRAMES; k++) {
				destination[i][j][k] = flattened[index++];
			}
		}
	}
}

int16_t buffer[BUFFER_SIZE] = { 0 };
uint8_t buffer_index = 0;
int32_t features[NUM_REFERENCES + 1][NUM_FEATURES][NUM_FRAMES];
int32_t flattened_features[(NUM_REFERENCES) * (NUM_FEATURES) * (NUM_FRAMES)];

uint8_t voice = 0;
uint8_t x;
int32_t isFlashed = 0;

int16_t sample;
uint32_t energy = 0;
uint8_t zero_crossed_flag = 0;
uint16_t zero_crossing_count = 0;
int32_t autocorrelation[BUFFER_SIZE - 1] = { 0 };

uint16_t r_motor = 0;
uint16_t l_motor = 0;

float min_distance, dist = 0.0;
uint8_t decision_index = 0;

int main(void) {
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	for (uint8_t i = 0; i < 10; i++) {
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
	}

	// Flash checking, loading features
	Flash_Read(FLASH_USER_START_ADDR, &isFlashed, 1);
	if (isFlashed) {
		voice = NUM_REFERENCES;
		Flash_Read(FLASH_USER_START_ADDR + 4, flattened_features,
		NUM_REFERENCES * NUM_FRAMES * NUM_FEATURES);
		unflatten_array(flattened_features, features);
	}

	while (1) {
		ADC_READ(&hadc1, &sample);
		min_distance = 1000000000000000.0;

		HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);
		if (sample > 0) {
			HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
			for (uint8_t i = 0; i < NUM_FRAMES; i++) {
				// reset all features
				energy = 0;
				zero_crossed_flag = 0;
				zero_crossing_count = 0;
				buffer_index = 0;
				for (uint8_t b = 0; b < BUFFER_SIZE; b++) {
					buffer[b] = 0;
				}
				for (uint16_t j = 0; j < (uint16_t) (NUM_SAMPLES / NUM_FRAMES);
						j++) {

					// sample reading
					ADC_READ(&hadc1, &sample);

					// energy calculation
					energy += abs(sample);
					// zero crossing detection
					x = (uint8_t) (sample > ZERO_CROSSING_THRESHOLD);
					if (x != zero_crossed_flag) {
						zero_crossed_flag = x;
						zero_crossing_count++;
					}

					// auto-correlation calculations
					buffer[buffer_index] = sample;

					for (uint8_t n = 1; n <= NUM_FEATURES - 2; n++) {
						if (n <= buffer_index)
							autocorrelation[n - 1] += buffer[buffer_index]
									* buffer[(buffer_index - n)];
						else
							autocorrelation[n - 1] += buffer[buffer_index]
									* buffer[(buffer_index - n) + BUFFER_SIZE];
					}

					// Update the buffer index
					buffer_index = (buffer_index + 1) % BUFFER_SIZE;
				}
				features[voice][0][i] = (int32_t) (energy
						/ (NUM_SAMPLES / NUM_FRAMES));
				features[voice][1][i] = zero_crossing_count;

				for (uint8_t autocorr = 0; autocorr < BUFFER_SIZE - 1;
						autocorr++) {
					features[voice][autocorr + 2][i] =
							(int32_t) (autocorrelation[autocorr]
									/ (NUM_SAMPLES / NUM_FRAMES));
					autocorrelation[autocorr] = 0;
				}
			}
			if (voice < NUM_REFERENCES) {
				voice++;
				if (voice == NUM_REFERENCES) {
					isFlashed = 1;
					Flash_Write(FLASH_USER_START_ADDR, &isFlashed, 1);

					// save features on the flash
					flatten_array(features, flattened_features);
					Flash_Write(FLASH_USER_START_ADDR + 4, flattened_features,
					NUM_REFERENCES * NUM_FRAMES * NUM_FEATURES);
				}
			} else {
				// decision making
				for (uint8_t ref = 0; ref < NUM_REFERENCES; ref++) {
					dist = calculate_distance(features[ref], features[NUM_REFERENCES]);
					if (dist < min_distance) {
						min_distance = dist;
						decision_index = ref;
					}
				}
				if (decision_index < 2) {
					// Forward
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_R_Pin,
							GPIO_PIN_SET);
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_L_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_R_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_L_Pin, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

					r_motor = 40000;
					l_motor = 40000;

					for (int z = 0; z <= 20; z++) {
						if (TIM1->CCR3 == r_motor && TIM3->CCR2 == l_motor)
							break;
						TIM1->CCR3 = 2000 * z;
						TIM3->CCR2 = 2000 * z;
						HAL_Delay(1);
					}

				} else if (decision_index < 4) {
					// Backward
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_R_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_L_Pin,
							GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, Backward_R_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_L_Pin, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

					TIM1->CCR3 = 0;
					TIM3->CCR2 = 0;

					HAL_Delay(3000);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

					HAL_Delay(100);

					r_motor = 30000;
					l_motor = 30000;

					for (int z = 0; z <= 20; z++) {
						TIM1->CCR3 = 1500 * z;
						TIM3->CCR2 = 1500 * z;
						HAL_Delay(2);
					}

					TIM1->CCR3 = 0;
					TIM3->CCR2 = 0;
					HAL_Delay(3000);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

					decision_index = 9;

				} else if (decision_index < 6) {
					// Right
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_R_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_L_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_R_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, Backward_L_Pin, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

					r_motor = 20000;
					l_motor = 40000;

					for (int z = 0; z <= 20; z++) {
						if (TIM1->CCR3 == r_motor && TIM3->CCR2 == l_motor)
							break;
						TIM1->CCR3 = 1000 * z;
						TIM3->CCR2 = 2000 * z;
						HAL_Delay(1);
					}

				} else if (decision_index < 8) {
					// Left
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_R_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_L_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_R_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_L_Pin, GPIO_PIN_SET);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

					r_motor = 40000;
					l_motor = 20000;

					for (int z = 0; z < 20; z++) {
						if (TIM1->CCR3 == r_motor && TIM3->CCR2 == l_motor)
							break;
						TIM1->CCR3 = 2000 * z;
						TIM3->CCR2 = 1000 * z;
						HAL_Delay(1);
					}

				} else {
					// Stop
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_R_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(Forward_R_GPIO_Port, Forward_L_Pin,
							GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_R_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Backward_L_Pin, GPIO_PIN_RESET);

					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

					TIM1->CCR3 = 0;
					TIM3->CCR2 = 0;

					r_motor = 0;
					l_motor = 0;
				}
			}
			HAL_Delay(500);
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Button_Pin) {
		voice = 0;
		isFlashed = 0;
		Flash_Write(FLASH_USER_START_ADDR, &isFlashed, 1);
	}
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_ADC1_Init(void) {

	ADC_ChannelConfTypeDef sConfig = { 0 };

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_TIM1_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			Forward_R_Pin | Forward_L_Pin | Backward_R_Pin | Backward_L_Pin
					| GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : led_Pin */
	GPIO_InitStruct.Pin = led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Forward_R_Pin Forward_L_Pin Backward_R_Pin Backward_L_Pin
	 PA4 PA5 */
	GPIO_InitStruct.Pin = Forward_R_Pin | Forward_L_Pin | Backward_R_Pin
			| Backward_L_Pin | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Button_Pin */
	GPIO_InitStruct.Pin = Button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {
	}
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif
