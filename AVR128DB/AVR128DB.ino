#define led       PIN_PA7

#define ON_TIME   4

#define USE_OSC 0

#include <avr/sleep.h>

void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
#if USE_OSC
  RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */
#else
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, 0x3);

  while (RTC.STATUS > 0) ;

  RTC.CLKSEL = RTC_CLKSEL_XOSC32K_gc;   /* 32.768kHz External Oscillator */
#endif

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  // For 2s overflow ticks while sleeping:
  RTC.INTCTRL = RTC_OVF_bm;
  
  RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RUNSTDBY_bm | RTC_RTCEN_bm;
}


volatile uint16_t ticks;

ISR(RTC_PIT_vect)
{
  if (ticks)
    ticks--;
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
}


volatile uint8_t sec2s, hours, minutes; // 2s ticks

ISR(RTC_CNT_vect)
{
  static unsigned int adjust_ticks;

  // Our clock is almost 1 sec too fast every 24 hours:
  // 24 * 3600 / 2 = 43200 for exactly 1 sec too fast
  if (++adjust_ticks == 45000)
    adjust_ticks = 0;
  else {
    if (sec2s < 58)
      sec2s += 2;
    else {
      sec2s = 0;
      if (minutes < 59)
        minutes++;
      else {
        minutes = 0;
        if (hours < 23)
          hours++;
        else
          hours = 0;
      }  
    }
  }

  RTC.INTFLAGS = RTC_OVF_bm;          /* Clear interrupt flag by writing '1' (required) */
}


void sleepDelay(uint16_t n)
{
  uint8_t period;
  uint16_t cticks;
  
  if (n < 100) {
      period = RTC_PERIOD_CYC8_gc; // 1/4 ms
      cticks = (n << 1); // Why NOT (n << 2) !?!?!
  } else if (n < 1000) {
      period = RTC_PERIOD_CYC64_gc; // 2 ms
      cticks = (n >> 1);
  } else if (n < 5000) {
      period = RTC_PERIOD_CYC1024_gc; // 32 ms
      cticks = (n >> 5);
  } else {
      period = RTC_PERIOD_CYC8192_gc; // 256 ms
      cticks = (n >> 8);
  }

  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm)  // Wait for new settings to synchronize
    ;

  RTC.PITCTRLA = period | RTC_PITEN_bm;    // Enable PIT counter: enabled

  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm)  // Wait for new settings to synchronize
    ;
  ticks = cticks;

  while (ticks)
    sleep_cpu();

  RTC.PITCTRLA = 0;    /* Disable PIT counter */
}

#if 1
unsigned int getBandgap ()
{
  analogReference(INTERNAL1V024);
  return 10 * analogRead(ADC_VDDDIV10);
} // end of getBandgap
#endif

#if 1
unsigned int getBattery ()
{
  unsigned int val = analogRead(PIN_PA3);

  // 3.0v regulator with voltage divider (820k/820k):
  const float Vref = 3000,
              R = 820000,
              Rintern = 100000000, // ATtiny ADC internalresistance
              Rpar = (R * Rintern) / (R + Rintern);
  const unsigned long scale = Vref * (R + Rpar) / Rpar;
  unsigned int results = scale * val / 0x400;

  return results;
} // end of getBattery
#endif


void blinkN(uint8_t n, uint8_t l = led)
{
  for (uint8_t i = 0; i < n; i++) {
    digitalWrite(l, HIGH);
    sleepDelay(ON_TIME);
    digitalWrite(l, LOW);
    sleepDelay(700);
  }
}


void blinkDec(uint8_t d)
{
  blinkN(d / 10);
  sleepDelay(1500);
  blinkN(d % 10);
  sleepDelay(3000);
}

void blinkDec3(uint16_t d)
{
  blinkN(d / 100);
  sleepDelay(1500);
  blinkDec(d % 100);
}


// the setup routine runs once when you press reset:
void setup() {
  RTC_init();                           /* Initialize the RTC timer */
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);  /* Set sleep mode to POWER DOWN mode */
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_enable();                       /* Enable sleep mode, but not going to sleep yet */

  pinMode(PIN_PA0, INPUT_PULLUP);
  pinMode(PIN_PA1, INPUT_PULLUP);
  pinMode(PIN_PA2, INPUT_PULLUP);
  pinMode(PIN_PA3, INPUT_PULLUP);
  pinMode(PIN_PA4, INPUT_PULLUP);
  pinMode(PIN_PA5, INPUT_PULLUP);
  pinMode(PIN_PA6, INPUT_PULLUP);
  pinMode(PIN_PA7, INPUT_PULLUP);
  
  pinMode(PIN_PD1, INPUT_PULLUP);
  pinMode(PIN_PD2, INPUT_PULLUP);
  pinMode(PIN_PD3, INPUT_PULLUP);
  pinMode(PIN_PD4, INPUT_PULLUP);
  pinMode(PIN_PD5, INPUT_PULLUP);
  pinMode(PIN_PD6, INPUT_PULLUP);
  pinMode(PIN_PD7, INPUT_PULLUP);
  
  pinMode(PIN_PC0, INPUT_PULLUP);
  pinMode(PIN_PC1, INPUT_PULLUP);
  pinMode(PIN_PC2, INPUT_PULLUP);
  pinMode(PIN_PC3, INPUT_PULLUP);
  
  pinMode(PIN_PF0, INPUT_PULLUP);
  pinMode(PIN_PF1, INPUT_PULLUP);
  
  pinMode(led, OUTPUT);

  for (byte i = 0; i < 3; i++) {
    blinkN(1, led);
  }

  sleepDelay(5000);

  getBandgap(); // drop first
}


unsigned voltage;

volatile unsigned cnt;


// the loop routine runs over and over again forever:
void loop() {

  //unsigned int v = voltage = getBattery();
  unsigned int v = voltage = getBandgap();
  blinkDec((v + 50) / 100);
//  blinkDec3((v + 5) / 10);

  blinkDec(minutes);

//  sleepDelay(5000); return;
  while (sec2s != 0) {
    if (sec2s % 10 == 0 && sec2s <= 55) {
      blinkN(1);
      sleepDelay(9000);
    } else
      sleepDelay(100);
  }

}
