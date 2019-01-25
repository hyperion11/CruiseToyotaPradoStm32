/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2\u8g2.h"
#include "stdbool.h"
#include "arm_math.h"
#include "stdlib.h"
#include "ftoa.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EEPROM_BLOCK_SIZE 32
#define Current_Data 0
#define TPS_Data 1
#define Switch_Data 2
#define font_w 9
#define font_h 8
#define LONG_PRESS_DURATION 20 //20x100ms-100мс время опроса рычага. 2секунды на распознание долгого нажания
enum ACTION {
	UP, DOWN
};
enum EDITED_VALUE {
	PID_P, PID_I, PID_D
};
enum DIR {
	CV, CCV, STOP
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
volatile uint8_t EDITED_VALUE_NUM = 0;
volatile uint8_t GUI_counter = 0;
volatile uint16_t ADC_Data[3]; //0-Current Sensor CH7;;;; 1-TPS CH8;;;; 2-Switch;;;;
volatile uint8_t OldKeyValue = 5; // Состояние покоя
volatile uint16_t rpm_ticks = 0, rpm = 0;
volatile uint8_t second;
volatile uint8_t NewKeyValue = 5;
const uint16_t values[5] = { 0, 564, 1215, 2075, 2300 };
const uint8_t error = 65;       // Величина отклонения от значений - погрешность
uint32_t amountsent = 0;
bool EEPROM_RECENT_WRITE = false;
volatile uint32_t IC_Faling_Val; // Direct mode	Faling Edge Detection
volatile uint8_t detectlongpress[4];
volatile float pid_error;
struct EESTR {
	float PID_P;
	float PID_I;
	float PID_D;
////
} EEPROM;

struct CRUISESTR {
	volatile bool ARMED;
	volatile bool THR_MOVING;
	volatile uint32_t MOVE_TIME_END;
	volatile uint8_t DIR;
	volatile int32_t PID_OUT;
	volatile uint8_t tick;
} CRUISE;

volatile struct SPDSTR {
	float CURRENT;
	float TARGET;
	float Pulse_width_avg;
	uint32_t Pulse_width;
	bool lowspeed;
	uint16_t index;
} SPD;
volatile bool STP = false, AT_OD = false, IDLE = true, AT_D = false;
volatile uint8_t screen = 1;
uint8_t activescreen = 0;
const uint16_t devAddr = (0x50 << 1); // HAL expects address to be shifted one bit to the left

static u8g2_t u8g2;
arm_pid_instance_f32 PID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SaveToEEPROM() {
	if (!EEPROM_RECENT_WRITE) {
		EEPROM.PID_P = PID.Kp;
		EEPROM.PID_I = PID.Ki;
		EEPROM.PID_D = PID.Kd;
		uint8_t * p = (uint8_t *) &EEPROM;
		//uint8_t const * p = (uint8_t const *) &EEPROM;
		for (size_t i = 0, len = sizeof(EEPROM); i < sizeof(EEPROM); i +=
		EEPROM_BLOCK_SIZE, len -= EEPROM_BLOCK_SIZE) {
			HAL_I2C_Mem_Write(&hi2c1, devAddr, i, I2C_MEMADD_SIZE_16BIT, p + i,
					(len > EEPROM_BLOCK_SIZE) ? EEPROM_BLOCK_SIZE : len, 1);

		}
		EEPROM_RECENT_WRITE = true;
	}
}

void move_throttle(uint8_t dir) {
	if (dir == STOP) {
		//PWM сигнал в ноль
		TIM2->CCR1 = 0;
		//На управляющие пины драйвера мотора ноли
		HAL_GPIO_WritePin(VNH2_SP30_INB_GPIO_Port, VNH2_SP30_INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VNH2_SP30_INA_GPIO_Port, VNH2_SP30_INB_Pin, GPIO_PIN_RESET);
	} else if (dir == CV) {
		HAL_GPIO_WritePin(VNH2_SP30_INA_GPIO_Port, VNH2_SP30_INA_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(VNH2_SP30_INB_GPIO_Port, VNH2_SP30_INB_Pin, GPIO_PIN_RESET);
		TIM2->CCR1 = 450;
	} else if (dir == CCV) {
		HAL_GPIO_WritePin(VNH2_SP30_INA_GPIO_Port, VNH2_SP30_INA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VNH2_SP30_INB_GPIO_Port, VNH2_SP30_INB_Pin, GPIO_PIN_SET);
		TIM2->CCR1 = 450;
	}
}

void disarm(void) {
	CRUISE.ARMED = false;
	//отключение магнитной муфты
	HAL_GPIO_WritePin(GPIOB, PB10_OUT_SOLENOID_Pin, GPIO_PIN_RESET);
	//отключение индикатора круиза
	HAL_GPIO_WritePin(GPIOB, CRUISE_LAMP_OUT_PB15_Pin, GPIO_PIN_RESET);
	//останов привода круиза
	move_throttle(STOP);
	arm_pid_reset_f32(&PID);

}

void CheckCruiseAndDisarm() {
	//if (CRUISE.ARMED == true)
	//if (STP == true || AT_D == false || IDLE == true	|| rpm > 3500 || SPD.CURRENT > 130.0 || SPD.CURRENT < 40.0)
	//disarm();
}

void arm() {
	SPD.TARGET = (int) SPD.CURRENT + 0.1;
	CRUISE.ARMED = true;
	//включение магнитной муфты
	HAL_GPIO_WritePin(GPIOB, PB10_OUT_SOLENOID_Pin, GPIO_PIN_SET);
	//включение индикатора круиза
	HAL_GPIO_WritePin(GPIOB, CRUISE_LAMP_OUT_PB15_Pin, GPIO_PIN_SET);
	CheckCruiseAndDisarm(STP, AT_D, IDLE, rpm, SPD.CURRENT);
}

uint8_t GetButtonNumberByValue(uint16_t value) { // Новая функция по преобразованию кода нажатой кнопки в её номер
	for (uint8_t i = 0; i <= 4; i++) {
		// Если значение в заданном диапазоне values[i]+/-error - считаем, что кнопка определена
		if (value <= values[i] + error && value >= values[i] - error)
			return i;
	}
	return 5;                 // Значение не принадлежит заданному диапазону
}

void readADC() {
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	ADC_Data[Current_Data] = HAL_ADCEx_InjectedGetValue(&hadc1,
	ADC_INJECTED_RANK_1);
	ADC_Data[TPS_Data] = HAL_ADCEx_InjectedGetValue(&hadc1,
	ADC_INJECTED_RANK_2);
	ADC_Data[Switch_Data] = HAL_ADCEx_InjectedGetValue(&hadc1,
	ADC_INJECTED_RANK_3);
	HAL_ADCEx_InjectedStop(&hadc1);
}

void readGPIO() {
	if (HAL_GPIO_ReadPin(GPIOA, STOP_IN_PA3_Pin) == GPIO_PIN_SET)
		//Обработка нажания на педаль тормоза. Нажата - высокий уровень. Отпущена - низкий
		STP = true;
	else
		STP = false;
	if (HAL_GPIO_ReadPin(GPIOA, D_IN_PA8_Pin) == GPIO_PIN_SET)
		//Обработка селектора АКПП. Селектор в положении D - высокий уровень. В любом другом - низкий
		AT_D = true;
	else
		AT_D = false;
	if (HAL_GPIO_ReadPin(GPIOB, IDLE_IN_PB14_Pin) == GPIO_PIN_SET)
		//Обработка ХХ. Дроссель полностью закрыт - низкий урове. Приоткрыта - высокий
		IDLE = false;
	else
		IDLE = true;

	if (HAL_GPIO_ReadPin(GPIOB, OD_IN_LIGHT_PB12_Pin) == GPIO_PIN_SET)
		//Обработка включенного OD. Нажата кнопка OD - высокий уровень. �?наче низкий
		AT_OD = true;
	else
		AT_OD = false;
}

void AdjustValue(uint8_t value, uint8_t action) {
	float input_value;
	switch (value) {
	case PID_P: {
		input_value = PID.Kp;
		if (action == UP)
			input_value += 1;
		else
			input_value -= 1;
		PID.Kp = input_value;
	}
		break;
	case PID_I: {
		input_value = PID.Ki;
		if (action == UP)
			input_value += 0.01;
		else
			input_value -= 0.01;
		PID.Ki = input_value;
	}
		break;
	case PID_D: {
		input_value = PID.Kd;
		if (action == UP)
			input_value += 0.01;
		else
			input_value -= 0.01;
		PID.Kd = input_value;
	}
		break;
	}
}

void ReadCruiseSwitch() {
	NewKeyValue = GetButtonNumberByValue(ADC_Data[Switch_Data]);
	//OldKeyValue 1
	//NewKeyValue 5
	//detectlongpress[1]=21
	//Поиск долгого нажатия клавиши, больше 2х секунд
	if (OldKeyValue == NewKeyValue && NewKeyValue != 5)
		detectlongpress[NewKeyValue] += 1;
	//Обработка нажатой клавиши в момент ее отпускания. Одновременно с проверкой на долгое нажатие.
	if (OldKeyValue != NewKeyValue && NewKeyValue == 5) {
		switch (OldKeyValue) {
		case 0: {
			//кнопка ON-OFF
			//При нажатии на главном экране включается-отключается круиз
			//При нажатии на экране настроек переключается настраиваемый параметр.
			if (detectlongpress[OldKeyValue] > LONG_PRESS_DURATION) {
				//долгое нажание на ON-OFF
				if (screen == 0) {
					//долгое нажание на экране тюна пидов - сохранение их в еепром единоразово. После этого
					//возможность сохранения блокируется до тех пор пока не перейдешь на главный экран
					//с целью исключения износа EEPROM
					SaveToEEPROM();

				}
				detectlongpress[OldKeyValue] = 0;
				break;
			}
			OldKeyValue = NewKeyValue;
			if (CRUISE.ARMED && screen == 1)
				//отключение круиза
				disarm();
			else if (!CRUISE.ARMED && screen == 1) {
				//включение круиза
				arm();
			} else if (screen == 0) {
				//на экране настроек переключается настраиваемый параметр.
				if (EDITED_VALUE_NUM != PID_D)
					EDITED_VALUE_NUM++;
				else
					EDITED_VALUE_NUM = 0;
			}
		}
			break;
		case 1:       //RES
		{
			if (detectlongpress[OldKeyValue] > LONG_PRESS_DURATION) {
				//длинное нажание на RES
				detectlongpress[OldKeyValue] = 0;
				break;
			}
			OldKeyValue = NewKeyValue;
			if (CRUISE.ARMED && screen == 1)
				SPD.TARGET += 10;
			else if (screen == 0) {
				AdjustValue(EDITED_VALUE_NUM, UP);
			}
		}
			break;
		case 2:       //SET
		{
			if (detectlongpress[OldKeyValue] > LONG_PRESS_DURATION) {
				//длинное нажание на SET
				detectlongpress[OldKeyValue] = 0;
				break;
			}
			OldKeyValue = NewKeyValue;
			//На основном экране уменьшение желаемой скорости
			if (CRUISE.ARMED && screen == 1)
				SPD.TARGET -= 10;
			//на нулевом экране - коррекция текущего параметра вниз
			else if (screen == 0) {
				AdjustValue(EDITED_VALUE_NUM, DOWN);
			}
		}
			break;
		case 3:       //CANCEL
		{
			if (detectlongpress[OldKeyValue] > LONG_PRESS_DURATION) {
				//длинное нажание на cancel
				detectlongpress[OldKeyValue] = 0;
				break;
			}
			OldKeyValue = NewKeyValue;
			screen++;
			if (screen == 2)
				screen = 0;
			HAL_GPIO_TogglePin(PC13_GPIO_Port, PC13_Pin);
		}
			break;
		case 5:       //NO PRESSED
		{
			OldKeyValue = NewKeyValue;
		}
			break;
		}

	} else
		OldKeyValue = NewKeyValue;
}

uint8_t u8x8_gpio_and_delay_mine(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		break;
	case U8X8_MSG_DELAY_NANO:
		break;
	case U8X8_MSG_DELAY_10MICRO:
		break;
	case U8X8_MSG_DELAY_100NANO:
		break;
	case U8X8_MSG_DELAY_MILLI:
		break;
	case U8X8_MSG_DELAY_I2C:
		break;
	case U8X8_MSG_GPIO_I2C_CLOCK:
		break;
	case U8X8_MSG_GPIO_I2C_DATA:
		break;
	default:
		break;
	}
	return 1;
}

uint8_t u8x8_byte_stm32f103_hw_i2c(U8X8_UNUSED u8x8_t *u8x8,
U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
U8X8_UNUSED void *arg_ptr) {
#define MAX_LEN 32
	static uint8_t vals[MAX_LEN];
	static uint8_t length = 0;
	U8X8_UNUSED uint8_t *args = arg_ptr;
	switch (msg) {
	case U8X8_MSG_BYTE_SEND: {
		if ((arg_int + length) <= MAX_LEN)
			for (int i = 0; i < arg_int; i++) {
				vals[length] = args[i];
				length++;
			}
		break;
	}
	case U8X8_MSG_BYTE_INIT:
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER: {
		length = 0;
		break;
	}
	case U8X8_MSG_BYTE_END_TRANSFER: {
		while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) { /* empty */
		}
		const uint8_t addr = 0x78;
		HAL_I2C_Master_Transmit(&hi2c1, addr, vals, length, 10);
		amountsent += length;
		break;
	}
	default:
		return 0;
	}
	return 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)

{
	if (GPIO_Pin == GPIO_PIN_11) { //RPM
		rpm_ticks++;
	} else {
		__NOP();
	}

}
void CalculateSPEED() {
	//Скорость усредняется
	SPD.Pulse_width_avg = SPD.Pulse_width / SPD.index;
	SPD.Pulse_width = 0;
	SPD.index = 0;
	if (SPD.Pulse_width_avg > 300000 || SPD.Pulse_width_avg < 1000) {
		SPD.lowspeed = true;
		SPD.Pulse_width_avg = 0;
		SPD.CURRENT = 0;
	} else {
		SPD.CURRENT = 600.0 / (SPD.Pulse_width_avg / 1000.0);
		SPD.lowspeed = false;
	}

}

void CalculateRPM() {
	second++;
	if (second == 10) {
		rpm = rpm_ticks * 40;
		second = 0;
		rpm_ticks = 0;
	}
}
void PIDLoop() {
	pid_error = SPD.CURRENT - SPD.TARGET;
	CRUISE.PID_OUT = (int) arm_pid_f32(&PID, pid_error);
	//если результат пида отрицательный до включается реверс, а сама переменная становится положительной
	//Управление мотором каждые 500мс
	//длительность импульса максимум 20мс
	if (CRUISE.PID_OUT == 0) {
		CRUISE.DIR = STOP;
	} else if (CRUISE.PID_OUT < 0) {
		if (CRUISE.PID_OUT < -20)
			CRUISE.PID_OUT = 20;
		CRUISE.MOVE_TIME_END = HAL_GetTick() + CRUISE.PID_OUT;
		CRUISE.DIR = CCV;
	} else {
		if (CRUISE.PID_OUT > 20)
			CRUISE.PID_OUT = 20;
		CRUISE.MOVE_TIME_END = HAL_GetTick();
		+CRUISE.PID_OUT;
		CRUISE.DIR = CV;
	}
	CRUISE.THR_MOVING = true;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		//прерывание каждые 100мс
		readGPIO();
		CheckCruiseAndDisarm();
		readADC();
		ReadCruiseSwitch();
		GUI_counter++; //Счетчик для отображения информации на экране
		CRUISE.tick++; //Счетчик цикла PID регуляции todo: 500мс пока что
		if (CRUISE.ARMED && CRUISE.tick == 5){
			CRUISE.tick = 0;
			CalculateSPEED();
			PIDLoop();
		}
		CalculateRPM(); //Вычисление оборотов. Раз в секунду думаю будет достаточно
	}
	if (htim->Instance == TIM3) {
		SPD.lowspeed = true; //если переполнился счетчик таймера спидометра - значит слишком медленная скорость
		SPD.Pulse_width = 0;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // =RISING= EDGE DETECTED
		__HAL_TIM_SET_COUNTER(&htim3, 0x00);
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // =FALLING= EDGE DETECTED
			{
		//Скорость считается по времени между первым и вторым фронтами.
		//Длительность фронтов суммируется и далее усредняется каждые 100ms todo:наверное надо усреднять каждые 500мс, как раз перед вычислением PID
		//длина фронта измеряется в микросекундах с разрешением 10микросекунд.
		//достаточно для измерения скорости от 2-3км\ч. Ниже будет возникать ошибка которая исправляется в обработчике переполнения таймера
		//а так же при вычислении средней скорости.
		IC_Faling_Val = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		SPD.Pulse_width += IC_Faling_Val * 10;
		SPD.index++;
	}
}

void showpage(uint8_t page) {
	char tmp_string[6];
	switch (page) {
	case 0: {
		u8g2_FirstPage(&u8g2);
		do {
			u8g2_SetFont(&u8g2, u8g2_font_artossans8_8u);
			u8g2_DrawStr(&u8g2, 0, font_h, "P");
			u8g2_DrawStr(&u8g2, 0, font_h * 2, "I");
			u8g2_DrawStr(&u8g2, 0, font_h * 3, "D");
			u8g2_DrawStr(&u8g2, font_w, font_h * (EDITED_VALUE_NUM + 1), "->");
			ftoa(PID.Kp, tmp_string, 3);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h, tmp_string);
			ftoa(PID.Ki, tmp_string, 3);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h * 2, tmp_string);
			ftoa(PID.Kd, tmp_string, 3);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h * 3, tmp_string);
		} while (u8g2_NextPage(&u8g2));
	}
		break;
	case 1: {
		EEPROM_RECENT_WRITE = false;
		u8g2_FirstPage(&u8g2);
		do {
			u8g2_SetFont(&u8g2, u8g2_font_artossans8_8u);
			u8g2_DrawStr(&u8g2, 0, font_h, "PID");
			u8g2_DrawStr(&u8g2, 0, font_h * 2, "CUR");
			u8g2_DrawStr(&u8g2, 0, font_h * 3, "TGT");
			u8g2_DrawStr(&u8g2, 0, font_h * 4, "IMP");
			u8g2_DrawStr(&u8g2, 0, font_h * 5, "RPM");
			u8g2_DrawStr(&u8g2, 0, font_h * 6, "STP");
			u8g2_DrawStr(&u8g2, 0, font_h * 7, "IDL");
			u8g2_DrawStr(&u8g2, 0, font_h * 8, "ATD");
			u8g2_DrawStr(&u8g2, font_w * 5, font_h * 6, "ATO");
			itoa(CRUISE.PID_OUT, tmp_string, 10);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h, tmp_string);
			itoa((int) (SPD.CURRENT + 0.1), tmp_string, 10);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h * 2, tmp_string);
			itoa((int) SPD.TARGET, tmp_string, 10);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h * 3, tmp_string);
			itoa(IC_Faling_Val * 10, tmp_string, 10);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h * 4, tmp_string);
			itoa(rpm, tmp_string, 10);
			u8g2_DrawStr(&u8g2, font_w * 3, font_h * 5, tmp_string);
			if (STP == true)
				u8g2_DrawBox(&u8g2, font_w * 3, font_h * 5, font_w, font_h);
			else
				u8g2_DrawFrame(&u8g2, font_w * 3, font_h * 5, font_w,
				font_h);
			if (IDLE == true)
				u8g2_DrawBox(&u8g2, font_w * 3, font_h * 6, font_w, font_h);
			else
				u8g2_DrawFrame(&u8g2, font_w * 3, font_h * 6, font_w,
				font_h);
			if (AT_D == true)
				u8g2_DrawBox(&u8g2, font_w * 3, font_h * 7, font_w, font_h);
			else
				u8g2_DrawFrame(&u8g2, font_w * 3, font_h * 7, font_w,
				font_h);
			if (AT_OD == true)
				u8g2_DrawBox(&u8g2, font_w * 8, font_h * 5, font_w, font_h);
			else
				u8g2_DrawFrame(&u8g2, font_w * 8, font_h * 5, font_w,
				font_h);
		} while (u8g2_NextPage(&u8g2));
	}
		break;
	default:
		break;
	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	CRUISE.ARMED = false;
	CRUISE.THR_MOVING = false;
	CRUISE.tick = 0;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_RTC_Init();
	MX_TIM1_Init();
	MX_CRC_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	u8g2_Setup_ssd1306_i2c_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_stm32f103_hw_i2c, u8x8_gpio_and_delay_mine);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display

	/*
	 EEPROM.PID_P=100;
	 EEPROM.PID_I=0.025;
	 EEPROM.PID_D=10;
	 uint8_t const * p = (uint8_t const *)&EEPROM;
	 for(size_t i = 0, len = sizeof(EEPROM); i < sizeof(EEPROM); i += EEPROM_BLOCK_SIZE, len -= EEPROM_BLOCK_SIZE) {
	 HAL_I2C_Mem_Write(&hi2c1, devAddr, i, I2C_MEMADD_SIZE_16BIT, p + i, (len > EEPROM_BLOCK_SIZE) ? EEPROM_BLOCK_SIZE : len, 1);
	 HAL_Delay(5);
	 }
	 */
	EEPROM.PID_P = 0;
	EEPROM.PID_I = 0;
	EEPROM.PID_D = 0;
	uint8_t * r = (uint8_t*) &EEPROM;
	for (size_t i = 0, len = sizeof(EEPROM); i < sizeof(EEPROM); i +=
	EEPROM_BLOCK_SIZE, len -= EEPROM_BLOCK_SIZE) {
		HAL_I2C_Mem_Read(&hi2c1, devAddr, i, I2C_MEMADD_SIZE_16BIT, r + i,
				(len > EEPROM_BLOCK_SIZE) ? EEPROM_BLOCK_SIZE : len, 1);
	}

	PID.Kp = EEPROM.PID_P; /* Proporcional */
	PID.Ki = EEPROM.PID_I; /* Integral */
	PID.Kd = EEPROM.PID_D; /* Derivative */

	arm_pid_init_f32(&PID, 1);
	for (uint8_t i = 0; i < 4; i++)
		detectlongpress[i] = 0;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		if (CRUISE.ARMED && CRUISE.THR_MOVING) {
			if (HAL_GetTick() <= CRUISE.MOVE_TIME_END) {
				move_throttle(CRUISE.DIR);
			} else {
				move_throttle(STOP);
				CRUISE.THR_MOVING = false;
			}
		}
		if (GUI_counter >= 4) {
			showpage(screen);
			GUI_counter = 0;
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_ADC;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
	/**Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_InjectionConfTypeDef sConfigInjected = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/**Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_7;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedNbrOfConversion = 3;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.InjectedOffset = 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}
	/**Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_8;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}
	/**Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_9;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef DateToUpdate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/**Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */

	/* USER CODE END Check_RTC_BKUP */

	/**Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
	DateToUpdate.Month = RTC_MONTH_JANUARY;
	DateToUpdate.Date = 0x1;
	DateToUpdate.Year = 0x0;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 119;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 59999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 3;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 899;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 719;
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
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(PC13_GPIO_Port, PC13_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			TB6612FNG_STB_Pin | PB10_OUT_SOLENOID_Pin | OD_OUT_BUTTON_PB13_Pin | CRUISE_LAMP_OUT_PB15_Pin
					| VNH2_SP30_INA_Pin | VNH2_SP30_INB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13_Pin */
	GPIO_InitStruct.Pin = PC13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PC13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PA1 PA2 PA4 PA5
	 PA6 PA9 PA10 PA11
	 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_10
			| GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : STOP_IN_PA3_Pin */
	GPIO_InitStruct.Pin = STOP_IN_PA3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(STOP_IN_PA3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TB6612FNG_STB_Pin PB10_OUT_SOLENOID_Pin OD_OUT_BUTTON_PB13_Pin CRUISE_LAMP_OUT_PB15_Pin
	 VNH2_SP30_INA_Pin VNH2_SP30_INB_Pin */
	GPIO_InitStruct.Pin = TB6612FNG_STB_Pin | PB10_OUT_SOLENOID_Pin | OD_OUT_BUTTON_PB13_Pin | CRUISE_LAMP_OUT_PB15_Pin
			| VNH2_SP30_INA_Pin | VNH2_SP30_INB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : RPM_EXTI11_Pin */
	GPIO_InitStruct.Pin = RPM_EXTI11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(RPM_EXTI11_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OD_IN_LIGHT_PB12_Pin */
	GPIO_InitStruct.Pin = OD_IN_LIGHT_PB12_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(OD_IN_LIGHT_PB12_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : IDLE_IN_PB14_Pin */
	GPIO_InitStruct.Pin = IDLE_IN_PB14_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(IDLE_IN_PB14_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : D_IN_PA8_Pin */
	GPIO_InitStruct.Pin = D_IN_PA8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(D_IN_PA8_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
