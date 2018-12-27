#define SW_IN PB1 //ADC9
uint8_t keyValue = 5; // Состояние покоя
uint8_t newKeyValue = 5;
const uint16_t values[5] = {0, 564, 1215, 2075, 2300};
const uint8_t error     = 15;                     // Величина отклонения от значений - погрешность

HardwareTimer timer(2);
#define SWITCH_LOOP_TIME 100000 //100ms 0.1sec
volatile uint16_t ticktock = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(SW_IN, INPUT);
  timer.pause();
  timer.setPeriod(SWITCH_LOOP_TIME); // in microseconds
  timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);
  timer.attachCompare1Interrupt( ISR1);
  timer.refresh();
  timer.resume();
  Serial.begin(115200);
}

void loop() {
  if (ticktock == 1)
  {
    ReadCruiseSwitch();
    ticktock = 0;
  }

}
void ISR1() {
  ticktock++;
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
