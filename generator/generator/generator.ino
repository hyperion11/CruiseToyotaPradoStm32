
#include <TimerOne.h>
#include <Timer.h>
#include <LcdKeypad.h>
LcdKeypad* myLcdKeypad = 0;
#define pulse_for_1kmh  600 //ms for 1km/h speed pulse duration+
uint8_t speedkmh = 1;
volatile uint16_t button_tick = 0;
const int spd_out = 2;  // D0
const int tach_out = 3; // D0
uint8_t button, spd = 0;
uint16_t rpm = 0;
volatile uint16_t pulse_dur_tach, pulse_dur_spd;
volatile boolean spd_out_State = LOW;
volatile boolean tach_out_State = LOW;
volatile uint32_t spd_out_dur, tach_out_dur;

void mstick()
{
  if (spd_out_dur >= pulse_dur_spd)
    if (spd_out_State == HIGH)
    {
      digitalWrite(spd_out, LOW);
      spd_out_State = LOW;
      spd_out_dur = 0;
    }
    else
    {
      digitalWrite(spd_out, HIGH);
      spd_out_State = HIGH;
      spd_out_dur = 0;
    }
  spd_out_dur++;
  ///////////////////////////////////////////////
  if (tach_out_dur >= pulse_dur_tach)
    if (tach_out_State == HIGH)
    {
      digitalWrite(tach_out, LOW);
      tach_out_State = LOW;
      tach_out_dur = 0;
    }
    else
    {
      digitalWrite(tach_out, HIGH);
      tach_out_State = HIGH;
      tach_out_dur = 0;
    }
  tach_out_dur++;
  button_tick++;
}




class MyLcdKeypadAdapter : public LcdKeypadAdapter
{
  private:
    LcdKeypad* m_lcdKeypad;
    unsigned char m_value;
  public:
    MyLcdKeypadAdapter(LcdKeypad* lcdKeypad)
      : m_lcdKeypad(lcdKeypad)
      , m_value(7)
    { }
    void handleKeyChanged(LcdKeypad::Key newKey)
    {
      if (0 != m_lcdKeypad)
      {
        if (LcdKeypad::UP_KEY == newKey)
        {
          spd++;
          pulse_dur_spd = _pulse_dur_spd(spd);
          Serial.println(pulse_dur_spd);
          myLcdKeypad->setCursor(4, 0);
          myLcdKeypad->print("   ");
          myLcdKeypad->setCursor(4, 0);
          myLcdKeypad->print(spd);
          myLcdKeypad->setCursor(9, 0);
          myLcdKeypad->print("   ");
          myLcdKeypad->setCursor(9, 0);
          myLcdKeypad->print(pulse_dur_spd / 20.0, 2);
        }
        else if (LcdKeypad::DOWN_KEY == newKey)
        {
          spd--;
          pulse_dur_spd = _pulse_dur_spd(spd);
          Serial.println(pulse_dur_spd);
          myLcdKeypad->setCursor(4, 0);
          myLcdKeypad->print("   ");
          myLcdKeypad->setCursor(4, 0);
          myLcdKeypad->print(spd);
          myLcdKeypad->setCursor(9, 0);
          myLcdKeypad->print("   ");
          myLcdKeypad->setCursor(9, 0);
          myLcdKeypad->print(pulse_dur_spd / 20.0, 2);
        }
        else if (LcdKeypad::LEFT_KEY == newKey)
        {
          rpm -= 100;
          pulse_dur_tach = _pulse_dur_tach(rpm);
          Serial.println(pulse_dur_tach);
          myLcdKeypad->setCursor(4, 1);
          myLcdKeypad->print("    ");
          myLcdKeypad->setCursor(4, 1);
          myLcdKeypad->print(rpm);
          myLcdKeypad->setCursor(9, 1);
          myLcdKeypad->print("   ");
          myLcdKeypad->setCursor(9, 1);
          myLcdKeypad->print(pulse_dur_tach / 20.0, 2);
        }
        else if (LcdKeypad::RIGHT_KEY == newKey)
        {
          rpm += 100;
          pulse_dur_tach = _pulse_dur_tach(rpm);
          Serial.println(pulse_dur_tach);
          myLcdKeypad->setCursor(4, 1);
          myLcdKeypad->print("    ");
          myLcdKeypad->setCursor(4, 1);
          myLcdKeypad->print(rpm);
          myLcdKeypad->setCursor(9, 1);
          myLcdKeypad->print("   ");
          myLcdKeypad->setCursor(9, 1);
          myLcdKeypad->print(pulse_dur_tach / 20.0, 2);
        }
      }
    }
};

uint16_t _pulse_dur_spd(uint8_t _speedkmh)//mks
{
  return (float(600) / float(_speedkmh) * 20.0f ); //every 50mks
}


uint16_t _pulse_dur_tach(uint16_t _rpm)
{
  return int(((1 / (float(_rpm) / 20.0f)) * 1000.0f) * 20.0f); //every 50mks
}

void setup() {
  // put your setup code here, to run once:
  myLcdKeypad = new LcdKeypad();  // instantiate an object of the LcdKeypad class, using default parameters
  myLcdKeypad->attachAdapter(new MyLcdKeypadAdapter(myLcdKeypad));
  myLcdKeypad->setBacklight(255);
  pinMode(spd_out, OUTPUT);
  pinMode(tach_out, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  digitalWrite(spd_out, 1);
  digitalWrite(tach_out, 1);
  spd_out_State = true;
  Timer1.initialize(50);
  Timer1.attachInterrupt(mstick); // blinkLED to run every 0.15 seconds
  digitalWrite(LED_BUILTIN, HIGH);
  spd = 60;
  rpm = 3000;
  pulse_dur_spd = _pulse_dur_spd(spd);
  pulse_dur_tach = _pulse_dur_tach(rpm);
  myLcdKeypad->setCursor(0, 0);
  myLcdKeypad->print("SPD");
  myLcdKeypad->setCursor(0, 1);
  myLcdKeypad->print("RPM");
  myLcdKeypad->setCursor(4, 0);
  myLcdKeypad->print(spd);
  myLcdKeypad->setCursor(9, 0);
  myLcdKeypad->print(pulse_dur_spd / 20.0, 2);
  myLcdKeypad->setCursor(4, 1);
  myLcdKeypad->print(rpm);
  myLcdKeypad->setCursor(9, 1);
  myLcdKeypad->print(pulse_dur_tach / 20.0, 2);
}



void loop() {
  if (button_tick >= (500 * 20))
  { //button = detectButton();
    yield();  // Get the timer(s) ticked, in particular the LcdKeypad dirver's keyPollTimer
  }
  // put your main code here, to run repeatedly:

}
