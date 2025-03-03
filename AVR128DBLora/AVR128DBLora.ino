/*
This example AVR128DB script for LORA counts events on a specified pin and sends the count with LoraWAN.

The script is tuned for low power consumption (2 microampere sleep current)

Tom Vijlbrief (C) 2025
*/

#include <arduino_lmic.h>
#include <arduino_lmic_hal_boards.h>
#include <arduino_lmic_hal_configuration.h>
#include <arduino_lmic_lorawan_compliance.h>
#include <arduino_lmic_user_configuration.h>

#include <lmic.h>
#include <hal/hal.h>

#include <avr/sleep.h>

#define LED       	PIN_PA7

#define COUNT_PIN   PIN_PA2

#define GET_BATTERY	0 // Read battery when using a voltage regulator

#define USE_LORA // Activate LORA RFM95 chip

//#define USE_TIMER // Use timer in deep sleep

#undef Serial
#define Serial      Serial1 // I use serial port 1 (C0/C1 = Tx/Rx)
#define SERIAL_OUT  PIN_PC0

unsigned TX_INTERVAL = 300;

#define ON_TIME   3 // LED ON Time in ms

#include "my_lora.h"

//static const u1_t APPEUI[8] = {  }; // reversed 8 bytes of AppEUI registered with ttnctl
void os_getArtEui (u1_t* buf) { memcpy(buf, APPEUI, 8);}

//static const unsigned char APPKEY[16] = {  }; // non-reversed 16 bytes of the APPKEY used when registering a device with ttnctl register DevEUI AppKey
void os_getDevKey (u1_t* buf) {  memcpy(buf, APPKEY, 16);}

static u1_t DEVEUI[8]={ };
void os_getDevEui (u1_t* buf) { memcpy(buf, DEVEUI, 8);}

void fillDEVEUI (u1_t* buf) {
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

struct {
  unsigned short temp;
  unsigned short rain;
  byte power;
  byte rate2;
} mydata;

volatile unsigned counter;  // Counts pin transitions to low

static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = PIN_PD6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PIN_PD5,
    .dio = {PIN_PD2, PIN_PD3, PIN_PD4},
};

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        mydata.rain = counter;
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, (uint8_t*)&mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
        counter = 0;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

bool in_tx;

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            in_tx = false;
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            in_tx = false;
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
              mydata.rate2 = (LMIC.frame + LMIC.dataBeg)[0];
              TX_INTERVAL = (1 << mydata.rate2);
              if (LMIC.dataLen > 1) {
                switch ((LMIC.frame + LMIC.dataBeg)[1]) {
                case 7: LMIC_setDrTxpow(DR_SF7, 14); break;
                case 8: LMIC_setDrTxpow(DR_SF8, 14); break;
                case 9: LMIC_setDrTxpow(DR_SF9, 14); break;
                case 10: LMIC_setDrTxpow(DR_SF10, 14); break;
                case 11: LMIC_setDrTxpow(DR_SF11, 14); break;
                case 12: LMIC_setDrTxpow(DR_SF12, 14); break;
                }
              }
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            in_tx = true;
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


void RTC_init(void)
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_OSC32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */
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

  RTC.INTFLAGS = RTC_OVF_bm;          /* Clear interrupt flag by writing '1' (required) */
}

#ifdef USE_TIMER
bool idle_mode = true;

void sleep_idle()
{
  idle_mode = true;
  set_sleep_mode(SLEEP_MODE_IDLE);
}

void sleep_standby()
{
  idle_mode = false;
  set_sleep_mode(SLEEP_MODE_STANDBY);
}
#endif

void sleepDelay(uint16_t n)
{
  uint8_t period;
  uint16_t cticks;

  int nudge;
  if (n < 100) {
      period = RTC_PERIOD_CYC8_gc; // 1/4 ms
      cticks = (n << 1); // Why NOT (n << 2) !?!?!
      nudge = 0;
  } else if (n < 1000) {
      period = RTC_PERIOD_CYC64_gc; // 2 ms
      cticks = (n >> 1);
      nudge = 2;
  } else if (n < 5000) {
      period = RTC_PERIOD_CYC1024_gc; // 32 ms
      cticks = (n >> 5);
      nudge = 32;
  } else {
      period = RTC_PERIOD_CYC8192_gc; // 256 ms
      cticks = (n >> 8);
      nudge = 256;
  }

  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm)  // Wait for new settings to synchronize
    ;

  RTC.PITCTRLA = period | RTC_PITEN_bm;    // Enable PIT counter: enabled

  while (RTC.PITSTATUS & RTC_CTRLBUSY_bm)  // Wait for new settings to synchronize
    ;
  ticks = cticks;
  pinModeFast(SERIAL_OUT, INPUT_PULLUP);
  uint32_t start = millis();

  while (ticks) {
    sleep_cpu();
#ifndef USE_TIMER
    nudge_millis(nudge);
#else
    if (!idle_mode) nudge_millis(nudge);
#endif
  }
  
  RTC.PITCTRLA = 0;    /* Disable PIT counter */
  
#ifndef USE_TIMER
  set_millis(start + n);
#else
  if (!idle_mode) set_millis(start + n);
#endif

  pinModeFast(SERIAL_OUT, OUTPUT);
}

#if 1
unsigned int getBandgap ()
{
  analogReference(INTERNAL1V024);
  analogRead(ADC_VDDDIV10); // drop first
  return analogRead(ADC_VDDDIV10);
} // end of getBandgap
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
} // end of getBattery
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


// the setup routine runs once when you press reset:
void setup() {

  Serial.begin(38400);
  Serial.println(F("Starting"));
  delay(100);

  RTC_init();                           /* Initialize the RTC timer */
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);  /* Set sleep mode to POWER DOWN mode, disables RTC! */
  #ifndef USE_TIMER
  set_sleep_mode(SLEEP_MODE_STANDBY);
  #else
  sleep_standby();
  #endif
  sleep_enable();                       /* Enable sleep mode, but not going to sleep yet */
  // #ifdef USE_TIMER
  // _PROTECTED_WRITE(CLKCTRL_MCLKCTRLB, ( CLKCTRL_MCLKCTRLB | TCB_RUNSTDBY_bm));
  // #endif

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
  pinMode(PIN_PD4, INPUT_PULLUP);
  pinMode(PIN_PD5, INPUT_PULLUP);
  pinMode(PIN_PD6, INPUT_PULLUP);
  pinMode(PIN_PD7, INPUT_PULLUP);
  
  pinMode(PIN_PC0, INPUT_PULLUP); // Serial out
  pinMode(PIN_PC1, INPUT_PULLUP);
  pinMode(PIN_PC2, INPUT_PULLUP);
  pinMode(PIN_PC3, INPUT_PULLUP);
  
  pinMode(PIN_PF0, INPUT_PULLUP);
  pinMode(PIN_PF1, INPUT_PULLUP);
  
  pinMode(LED, OUTPUT);
  pinConfigure(COUNT_PIN, (PIN_DIR_INPUT | PIN_PULLUP | PIN_ISC_FALL));

  for (byte i = 0; i < 3; i++) {
    blinkN(1, LED);
  }

  fillDEVEUI(DEVEUI);
  Serial.flush();

  // while (1) {
  //   sleepDelay(50);
  //   Serial.print(millis()); Serial.print(' '); Serial.println(micros()); Serial.flush();
  // }

#ifdef USE_LORA
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
#endif
}


unsigned voltage;

ISR(PORTA_PORT_vect)
{
  //check flags, this code is from the DxCore documentation
  byte flags = VPORTA.INTFLAGS; //here we'll use a port example, like I said, I can't think of any other cases where the disable behavior is wacky like this.
  if (flags & (1 << 2)) {           // Check if the flag is set, if it is; since we se flags to determine what code to run, we need o check flags, otherwise we
    PORTA.PIN1CTRL &= ~PORT_ISC_gm; // we want to turn off hat interrupt so we do so here.
    VPORTA.INTFLAGS |= (1 << 2);    // Now clear flag knowing it won' get set again. .
  }
  if (flags & (1 << 2)) {
    // Handle flag 2 interrupt.
    static unsigned long prev;
    if ((unsigned long)(millis() - prev) > 1000) {
      counter++;
      prev = millis();
    }
  }
}


// the loop routine runs over and over again forever:
void loop() {

  static int loopcnt;
  
  loopcnt++;
  os_runloop_once();

  #ifdef USE_TIMER
  if (!os_queryTimeCriticalJobs(ms2osticks(100)))
    sleepDelay(90);
  #endif

  if (in_tx) {
    if ((loopcnt & 0x7FF) == 1) {
      Serial.print('+');
    }
    Serial.flush();
#ifdef USE_TIMER
    digitalWriteFast(LED, (loopcnt & 0x3F) == 0);
#else
    digitalWriteFast(LED, (loopcnt & 0xF) == 0);
#endif
    return;
  }
  digitalWriteFast(LED, LOW);
#ifdef USE_TIMER
  sleep_standby();
#endif
  
  //Serial.print(millis()); Serial.print(':');
  //unsigned int v = voltage = getBattery();
  // unsigned int v = voltage = getBandgap();
  // mydata.power = v - 100;

#if GET_BATTERY
  Serial.print(getBattery());
  Serial.print(' ');
#endif
  // Serial.println(getBandgap());
  // Serial.print(' ');
  // Serial.print(counter);
  // Serial.print(' ');
  // Serial.println(digitalRead(COUNT_PIN));
  // Serial.flush();

  // blinkN(mydata.power/10, led);
 
  if (!os_queryTimeCriticalJobs(ms2osticks(2000))) {
    if ((loopcnt & 0x7) == 0) {
      mydata.power = getBandgap() - 100;
      blinkN(1, LED);
    }
    // Serial.println(counter);
    Serial.print('-'); Serial.flush();
    sleepDelay(2000);
  }
#ifdef USE_TIMER
  sleep_idle();
#endif
}
