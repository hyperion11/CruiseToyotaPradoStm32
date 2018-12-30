#define SW_IN PB1 //ADC9
//#define VNH2_SP30_EN PB10 //PB10 T2C3N B10
//#define VNH2_SP30_CS PB1 //ADC9
//#define VNH2_SP30_INA PB8 //PB8 T4C3 B8
//#define VNH2_SP30_INB PB9 //PB9 T4C4 B9
//#define VNH2_SP30_PWM PA0 //PA0 T2C1 A0


uint8_t keyValue = 5; // Состояние покоя
uint8_t newKeyValue = 5;
const uint16_t values[5] = {0, 564, 1215, 2075, 2300};
const uint8_t error     = 15;                     // Величина отклонения от значений - погрешность

HardwareTimer timer_2(2);
HardwareTimer timer_3(3);
#define SWITCH_LOOP_TIME 100000 //100ms 0.1sec
volatile uint16_t ticktock = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(SW_IN, INPUT_ANALOG);
 // pinMode(VNH2_SP30_EN, OUTPUT);
 // pinMode(VNH2_SP30_INA, OUTPUT );
 // pinMode(VNH2_SP30_INB, OUTPUT );
 // pinMode(VNH2_SP30_PWM, PWM);
  
  timer_2.pause();
  timer_3.pause();
  timer_2.setPeriod(SWITCH_LOOP_TIME); // in microseconds
  timer_3.setPeriod(20000);
  timer_2.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer_3.setMode(TIMER_CH2, TIMER_OUTPUT_COMPARE);
  timer_2.setCompare(TIMER_CH1, 1);
  timer_3.setCompare(TIMER_CH2, 1);
  timer_2.attachCompare1Interrupt( ISR2);
  timer_3.attachCompare2Interrupt( ISR3);
  timer_2.refresh();
  timer_3.refresh();
  timer_2.resume();
  timer_3.resume();
  Serial.begin(115200);
}

void loop() {
  if (ticktock == 1)
  {
    ReadCruiseSwitch();
    ticktock = 0;
  }

}
void ISR2() {
  ticktock++;
}

void ISR3() {
  Serial.println(millis());
}
void ReadCruiseSwitch() {
  newKeyValue = GetButtonNumberByValue(analogRead(SW_IN));
  if (keyValue != newKeyValue) {  // Если новое значение не совпадает со старым - реагируем на него
    keyValue = newKeyValue;       // Актуализируем переменную хранения состояния
    switch (keyValue) {
      case 0://ON-OFF
        {
          Serial.println("ON-OFF BUTTON PRESSED" + String(keyValue));
        }
        break;
      case 1://RES
        {
          Serial.println("RES BUTTON PRESSED" + String(keyValue));
        }
        break;
      case 2://SET
        {
          Serial.println("SET BUTTON PRESSED" + String(keyValue));
        }
        break;
      case 3://CANCEL
        {
          Serial.println("CANCEL BUTTON PRESSED" + String(keyValue));
        }
        break;
      case 5://NO PRESSED
        break;
    }
  }
}

uint8_t GetButtonNumberByValue(uint16_t value) {   // Новая функция по преобразованию кода нажатой кнопки в её номер
  for (uint8_t i = 0; i <= 4; i++) {
    // Если значение в заданном диапазоне values[i]+/-error - считаем, что кнопка определена
    if (value <= values[i] + error && value >= values[i] - error) return i;
  }
  return 5;                              // Значение не принадлежит заданному диапазону
}
