
#include <TimerOne.h>
uint8_t speedkmh = 1;
uint32_t speedms(uint8_t _speedkmh)
{
  return (_speedkmh * 1000 / 3600);
}
#define pulse_for_1kmh  600 //ms for 1km/h speed pulse duration+
const int spd_out = 2;  // D0
const int tach_out = 3; // D0
volatile boolean spd_out_State = LOW;
volatile boolean tach_out_State = LOW;
volatile uint32_t ticktock = 0, spd_out_dur, tach_out_dur;
void mstick()
{
  ticktock++;
  if ((spd_out_dur >= 600) && (spd_out_State == HIGH))
  {
    digitalWrite(spd_out, LOW);
    spd_out_State = LOW;
    spd_out_dur = 0;

  }
  if ((spd_out_dur >= 600) && (spd_out_State == LOW))
  {
    digitalWrite(spd_out, HIGH);
    spd_out_State = HIGH;
    spd_out_dur = 0;
  }
  spd_out_dur++;
  ///////////////////////////////////////////////
  if ((tach_out_dur >= 20) && (tach_out_State == HIGH))
  {
    digitalWrite(tach_out, LOW);
    tach_out_State = LOW;
    tach_out_dur = 0;

  }
  if ((tach_out_dur >= 20) && (tach_out_State == LOW))
  {
    digitalWrite(tach_out, HIGH);
    tach_out_State = HIGH;
    tach_out_dur = 0;
  }
  tach_out_dur++;
}


uint16_t pulse_dur_spd(uint8_t _speedkmh)
{
  return (pulse_for_1kmh / _speedkmh);
}


uint16_t pulse_dur_tach(uint8_t _rpm)
{
  return ((1 / (_rpm / 20)) * 1000);
}

void spd_out_ISR ()
{
  if (spd_out_State == false) {
    spd_out_State = true;
    digitalWrite(spd_out, 1);
    digitalWrite(tach_out, 1);
  } else {
    spd_out_State = false;
    digitalWrite(spd_out, 0);
    digitalWrite(tach_out, 0);
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(spd_out, OUTPUT);
  pinMode(tach_out, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  digitalWrite(spd_out, 1);
  digitalWrite(tach_out, 1);
  spd_out_State = true;
  Timer1.initialize(1000);
  Timer1.attachInterrupt(mstick); // blinkLED to run every 0.15 seconds
  digitalWrite(LED_BUILTIN, HIGH);
}



void loop() {

  // put your main code here, to run repeatedly:

}
