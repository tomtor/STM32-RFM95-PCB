/*
 * Adapted RadioLib example for AVR128DB with rain bucket counter
 *
 * Power usage 2uA in sleep mode. Also enters 2uA (radio) sleep when waiting for LoraWan reply.
 */

#include "config.h"

#include <avr/sleep.h>

int uplinkIntervalSeconds = 1 * 60;    // minutes x seconds

#define LED             PIN_PA7
#define COUNT_PIN       PIN_PA2

//#define TIMING_PIN      PIN_PD7

#define ON_TIME         3 // LED ON blink time in ms

#define RTC_PERIOD	    0x8000 // Overflow at 32768 ticks from RTC
#define RTC_MILLIS	    1000   // That matches 1000ms

#define USE_OSC         0      // 1 if no crystal or 0 when using a 32768 crystal

volatile unsigned counter;  // Counts pin transitions to low

typedef uint8_t u1_t;

void fillDEVEUI (u1_t* buf) { // Fetch AVR128 serial numbers
  // use chip ID:
  u1_t *serial= (u1_t*)&SIGROW.SERNUM0;
  for (byte i= 0; i <= 7; i++)
    buf[i] = serial[i] ^ serial[8+i];
  // Make locally registered:
  buf[0] = (buf[0] & ~0x3) | 0x1;

  Serial.print("DevEui: ");
  for (int i = 7; i >= 0; i--) {
    Serial.printHex((uint8_t)buf[i]);
    Serial.print(' ');
  }
  Serial.println("");
}

void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
    ;                                   /* Wait for all register to be synchronized */
  RTC.PER = RTC_PERIOD - 1;		/* 1 sec overflow */
#if USE_OSC
  RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */
#else
  _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_ENABLE_bm | CLKCTRL_LPMODE_bm);

  while (RTC.STATUS > 0) ;

  RTC.CLKSEL = RTC_CLKSEL_XOSC32K_gc;   /* 32.768kHz External Oscillator */
#endif

  // For overflow ticks while sleeping:
  RTC.INTCTRL = RTC_OVF_bm;

  RTC.CTRLA |= RTC_PRESCALER_DIV1_gc | RTC_RUNSTDBY_bm | RTC_RTCEN_bm;
}

volatile uint8_t sleep_cnt;

volatile uint8_t secs, hours, minutes;
volatile uint16_t time_s;

ISR(RTC_CNT_vect)
{
  if (RTC.INTFLAGS & RTC_CMP_bm) {
    sleep_cnt--;
    RTC.INTFLAGS = RTC_CMP_bm;            /* Clear interrupt flag by writing '1' (required) */
  }

  if (RTC.INTFLAGS & RTC_OVF_bm) {
    time_s++;
    if (secs < 59)
      secs += 1;
    else {
      secs = 0;
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
    RTC.INTFLAGS = RTC_OVF_bm;          /* Clear interrupt flag by writing '1' (required) */
  }
}

bool idle_mode = true;

void sleep_idle() // We do not use real idle mode, but just keep TCB2 running in standby
{
  idle_mode = true;
  TCB2.CTRLA = TCB2.CTRLA | TCB_RUNSTDBY_bm;
}

void sleep_standby()
{
  idle_mode = false;
  TCB2.CTRLA = TCB2.CTRLA & ~TCB_RUNSTDBY_bm;
}

void sleepDelay(uint16_t orgn)
{
#ifdef TIMING_PIN
  digitalWriteFast(TIMING_PIN, HIGH);
#endif
  while (RTC.STATUS /* & RTC_CMPBUSY_bm */)  // Wait for new settings to synchronize
    ;

  uint32_t delay;
  uint16_t cnt = RTC.CNT;
  RTC.CMP = (cnt + (delay = (orgn * 32UL) + uint16_t(orgn / 4 * 3))) & (RTC_PERIOD-1); // With this calculation every multiple of 4ms is exact!

  if (RTC.CMP - cnt <= 1) { // overflow is/was near
    while (RTC.CNT < RTC.CMP) // Wait for it
      ;
    delay -= RTC_PERIOD;
  }

  RTC.INTCTRL |= RTC_CMP_bm; // This might trigger a pending interrupt, so do this before assigning sleep_cnt!
  sleep_cnt = delay / RTC_PERIOD + 1; // Calculate number of wrap arounds (overflows)
  uint64_t start = millis();
  while (sleep_cnt)
    sleep_cpu();
  if (!idle_mode)
    set_millis(start + orgn);

  RTC.INTCTRL &= ~RTC_CMP_bm;
#ifdef TIMING_PIN
  digitalWriteFast(TIMING_PIN, LOW);
#endif
}

void count_rain()
{
    static uint16_t prev;
    if (counter == 0)
      time_s = 0;
    if (time_s - prev > 1 || counter == 0) {
      counter++;
      prev = time_s;
    }
}

#if 1
unsigned int getBandgap ()
{
  analogReference(INTERNAL1V024);
  analogRead(ADC_VDDDIV10); // drop first
  return analogRead(ADC_VDDDIV10);
}
#endif

#if GET_BATTERY
unsigned int getBattery ()
{
  analogReference(VDD);

  analogRead(PIN_PD1);
  unsigned int val = analogRead(PIN_PD1);

  // 3.3v regulator with voltage divider (820k/820k):
  const float Vref = 3300;
  const unsigned long scale = Vref*2;
  unsigned int results = scale * val / 0x400;

  return results;
}
#endif

#if 0
unsigned int getTemp()
{
  analogReference(INTERNAL2V048);
  analogRead(ADC_TEMPERATURE);
  uint16_t adc_reading = analogRead(ADC_TEMPERATURE);
  uint16_t sigrow_offset = SIGROW.TEMPSENSE1; // Read unsigned value from signature row
  uint16_t sigrow_slope = SIGROW.TEMPSENSE0; // Read unsigned value from signature row
  uint32_t temp = sigrow_offset - adc_reading;
  temp *= sigrow_slope; // Result will overflow 16-bit variable
  temp += 0x0800; // Add 4096/2 to get correct rounding on division below
  temp >>= 12; // Round off to nearest degree in Kelvin, by dividing with 2^12 (4096)
  return temp-273;
}
#endif

void blinkN(uint8_t n, uint8_t l = LED)
{
  for (uint8_t i = 0; i < n; i++) {
    digitalWriteFast(l, HIGH);
    sleepDelay(ON_TIME);
    digitalWriteFast(l, LOW);
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

void sleepDelayRadio(uint32_t t)
{
  if (t > 500)
    radio.sleep();
  sleep_standby();
  sleepDelay(t);
  sleep_idle();
}

void sleepDelayLong(uint32_t t)
{
  while (t > 32768) {
    sleepDelay(32768);
    t -= 32768;
  }
  sleepDelay(t);
}


void setup() {
  Serial.begin(38400, SERIAL_TX_ONLY);
  Serial.println("Starting");
  delay(100);

  TCA0.SPLIT.CTRLA = 0;                 // If you aren't using TCA0 for anything

  RTC_init();                           /* Initialize the RTC timer */
  set_sleep_mode(SLEEP_MODE_STANDBY);
  sleep_standby();
  sleep_enable();                       /* Enable sleep mode, but not going to sleep yet */

  pinMode(PIN_PA0, INPUT_PULLUP);
  pinMode(PIN_PA1, INPUT_PULLUP);
  pinMode(PIN_PA2, INPUT_PULLUP);
  pinMode(PIN_PA3, INPUT_PULLUP);
  pinMode(PIN_PA4, INPUT_PULLUP);
  pinMode(PIN_PA5, INPUT_PULLUP);
  pinMode(PIN_PA6, INPUT_PULLUP);
  pinMode(PIN_PA7, INPUT_PULLUP);
  
#if GET_BATTERY
  pinMode(PIN_PD1, INPUT);
#else
  pinMode(PIN_PD1, INPUT_PULLUP);
#endif
  pinMode(PIN_PD2, INPUT_PULLUP);
  pinMode(PIN_PD3, INPUT_PULLUP);
  pinMode(PIN_PD4, INPUT); 		// DIO2
  pinMode(PIN_PD5, INPUT_PULLUP);
  pinMode(PIN_PD6, INPUT_PULLUP);
  pinMode(PIN_PD7, INPUT_PULLUP);
#ifdef TIMING_PIN
  pinMode(TIMING_PIN, OUTPUT);
#endif
  
  //pinMode(PIN_PC0, INPUT_PULLUP);	// Serial out
  pinMode(PIN_PC1, INPUT_PULLUP);
  pinMode(PIN_PC2, INPUT_PULLUP);
  pinMode(PIN_PC3, INPUT_PULLUP);
  
  pinMode(PIN_PF0, INPUT_PULLUP);
  pinMode(PIN_PF1, INPUT_PULLUP);
  
  pinMode(LED, OUTPUT);

  pinConfigure(COUNT_PIN, (PIN_DIR_INPUT | PIN_PULLUP | PIN_ISC_FALL));
  attachInterrupt(digitalPinToInterrupt(COUNT_PIN), count_rain, FALLING);

  for (byte i = 0; i < 3; i++) {
    blinkN(1, LED);
  }

  Serial.flush();
  Serial.println(F("\nSetup ... "));

  Serial.println(F("Initialise the radio"));
  radio.reset();
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

  fillDEVEUI((uint8_t*)&devEUI);
  // Setup the OTAA session information
  //state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  state = node.beginOTAA(joinEUI, devEUI, NULL, appKey);
  debug(state != RADIOLIB_ERR_NONE, F("Initialise node failed"), state, true);

  Serial.println(F("Join ('login') the LoRaWAN Network"));
  state = node.activateOTAA();
  debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, true);

  Serial.println(F("Ready!\n"));
#if !USE_OSC
  Serial.println("Using 32768 crystal!");
  node.setSleepFunction(sleepDelayRadio); // Needs a 32768 crystal
#endif
}

struct {
  unsigned short temp;
  unsigned short rain;
  byte power;
  byte rate2;
} mydata;

#define voltagePin    PA1

void readData()
{
    mydata.power = getBandgap() - 100;
    noInterrupts(); // prevent race condition with rain interrupts
    mydata.rain = counter;
    counter = 0;
    interrupts();
}

void loop() {
  Serial.println(F("Sending uplink"));

  readData();
  
  // Perform an uplink
  uint8_t reply[2];
  size_t replylen;
  int16_t state = node.sendReceive((uint8_t*)&mydata, sizeof(mydata), 1, reply, &replylen);
  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);

  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
  if(state > 0) {
    Serial.println(F("Received a downlink"));
    if (replylen == 1) {
      mydata.rate2= reply[0];
      uplinkIntervalSeconds= (1 << reply[0]);
    }
  } else {
    Serial.println(F("No downlink received"));
  }

  Serial.print(F("Next uplink in "));
  Serial.print(uplinkIntervalSeconds);
  Serial.println(F(" seconds\n"));
  Serial.flush();
 
  radio.sleep();

  // Wait until next uplink - observing legal & TTN FUP constraints
  sleep_standby();
  sleepDelayLong(uplinkIntervalSeconds * 1000UL);  // delay needs milli-seconds
 
  blinkN(1);
  sleep_idle();
  Serial.printf("Up time: %02d:%02d:%02d\n", hours, minutes, secs);
}
