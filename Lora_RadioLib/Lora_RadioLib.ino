/*
 * Adapted RadioLib example
 */

#include "config.h"

// STM32 Unique Chip IDs
#define STM32_ID	((uint8_t *) 0x1FFFF7E8)

void os_getDevEui (uint8_t* buf)
{
  // use chip ID:
  memcpy(buf, &STM32_ID[1], 8);
  // Make locally registered:
  buf[0] = buf[0] & ~0x3 | 0x1;

  Serial.print("DevEui: ");
  for (int i = 7; i >= 0; i--) {
    char str[8];
    sprintf(str, "%02x", buf[i]);
    Serial.print(str);
  }
  Serial.println("");
}

#if 1
#include <libmaple/iwdg.h>

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>

#include <RTClock.h>

RTClock rt(RTCSEL_LSI, 39); // 1 milli second alarm

// Define the Base address of the RTC registers (battery backed up CMOS Ram), so we can use them for config of touch screen or whatever.
// See http://stm32duino.com/viewtopic.php?f=15&t=132&hilit=rtc&start=40 for a more details about the RTC NVRam
// 10x 16 bit registers are available on the STM32F103CXXX more on the higher density device.
#define BKP_REG_BASE   ((uint32_t *)(0x40006C00 +0x04))

void storeBR(int i, uint32_t v) {
  BKP_REG_BASE[2 * i] = (v << 16);
  BKP_REG_BASE[2 * i + 1] = (v & 0xFFFF);
}

uint32_t readBR(int i) {
  return ((BKP_REG_BASE[2 * i] & 0xFFFF) >> 16) | (BKP_REG_BASE[2 * i + 1] & 0xFFFF);
}

bool next = false;

void sleepMode(bool deepSleepFlag)
{
  // Clear PDDS and LPDS bits
  PWR_BASE->CR &= PWR_CR_LPDS | PWR_CR_PDDS | PWR_CR_CWUF;

  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  // Enable wakeup pin bit.
  PWR_BASE->CR |=  PWR_CSR_EWUP;

  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // System Control Register Bits. See...
  // http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dui0497a/Cihhjgdh.html
  if (deepSleepFlag) {
    // Set Power down deepsleep bit.
    PWR_BASE->CR |= PWR_CR_PDDS;
    // Unset Low-power deepsleep.
    PWR_BASE->CR &= ~PWR_CR_LPDS;
  } else {
    adc_disable(ADC1);
    adc_disable(ADC2);
#if STM32_HAVE_DAC
    dac_disable_channel(DAC, 1);
    dac_disable_channel(DAC, 2);
#endif
    //  Unset Power down deepsleep bit.
    PWR_BASE->CR &= ~PWR_CR_PDDS;
    // set Low-power deepsleep.
    PWR_BASE->CR |= PWR_CR_LPDS;
  }

  // Now go into stop mode, wake up on interrupt
  asm("    wfi");

  // Clear SLEEPDEEP bit so we can use SLEEP mode
  SCB_BASE->SCR &= ~SCB_SCR_SLEEPDEEP;
}

int sleepTime;

void AlarmFunction () {
  // We always wake up with the 8Mhz HSI clock!
  // So adjust the clock if needed...

#if F_CPU == 8000000UL
  // nothing to do, using about 8 mA
#elif F_CPU == 16000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_2);
#elif F_CPU == 48000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_6);
#elif F_CPU == 72000000UL
  rcc_clk_init(RCC_CLKSRC_HSI, RCC_PLLSRC_HSE , RCC_PLLMUL_9);
#else
#error "Unknown F_CPU!?"
#endif

  extern volatile uint32 systick_uptime_millis;
  noInterrupts();
  systick_uptime_millis += sleepTime;
  interrupts();
}

void mdelay(int n, bool mode = false)
{
  sleepTime = n;
  const int interval = 10000;
  while (n > 0) {
    time_t nextAlarm = rt.getTime() + (n > interval ? interval : n); // Calculate from time now.
    rt.createAlarm(&AlarmFunction, nextAlarm);
    iwdg_feed();
    sleepMode(mode);
    n -= interval;
  }
}

void msleep(uint32_t ms)
{
  uint32_t start = rt.getTime();

  while (rt.getTime() - start < ms) {
    asm("    wfi");
  }
}
#endif

void blinkN(int n, int d = 400, int t = 800)
{
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_BUILTIN, 0);
    mdelay(5);
    digitalWrite(LED_BUILTIN, 1);
    mdelay(d);
  }
  pinMode(LED_BUILTIN, INPUT_ANALOG);
  mdelay(t);
}

void allInput()
{
  adc_disable(ADC1);
  adc_disable(ADC2);

  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA1, INPUT_ANALOG);
  pinMode(PA2, INPUT_ANALOG);
  pinMode(PA3, INPUT_ANALOG);
  pinMode(PA4, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA6, INPUT_ANALOG);
  pinMode(PA7, INPUT_ANALOG);
  pinMode(PA8, INPUT_ANALOG);
  pinMode(PA9, INPUT_ANALOG);
  pinMode(PA10, INPUT_ANALOG);

  pinMode(PA11, INPUT_ANALOG);
  pinMode(PA12, INPUT_ANALOG);
  pinMode(PA13, INPUT_ANALOG);
  pinMode(PA14, INPUT_ANALOG);
  pinMode(PA15, INPUT_ANALOG);

  pinMode(PB0, INPUT_ANALOG);
  pinMode(PB1, INPUT_ANALOG);
  pinMode(PB2, INPUT_ANALOG);
  pinMode(PB3, INPUT_ANALOG);
  pinMode(PB4, INPUT_ANALOG);
  pinMode(PB5, INPUT_ANALOG);
  pinMode(PB6, INPUT_ANALOG);
  pinMode(PB7, INPUT_ANALOG);
  pinMode(PB8, INPUT_ANALOG);
  pinMode(PB9, INPUT_ANALOG);
  pinMode(PB10, INPUT_ANALOG);
  pinMode(PB11, INPUT_ANALOG);
  pinMode(PB12, INPUT_ANALOG);
  pinMode(PB13, INPUT_ANALOG);
  pinMode(PB14, INPUT_ANALOG);
  pinMode(PB15, INPUT_ANALOG);
}

void mdelay_long(long unsigned int m)
{
  if (m <100)
    return;
  mdelay(m - 100);
}

#include <CCS811.h>

#define WAKE_PIN  PB12
#define ADDR      0x5A

CCS811 sensor;

void setup() {
  iwdg_init(IWDG_PRE_256, 4095); // 26s watchdog

  allInput();

  pinMode(LED_BUILTIN, OUTPUT);
  blinkN(3);

  Serial.begin(115200);
  while(!Serial);
  delay(5000);  // Give time to switch to the serial monitor
  Serial.println(F("\nSetup ... "));

  while (1) {
    blinkN(1);
    if (!sensor.begin(uint8_t(ADDR), uint8_t(WAKE_PIN))) {
      Serial.println("Initialization failed.");
      Serial.flush();
      mdelay(2000);
    } else {
      Serial.println("CCS811 OK");
      sensor.sleep();
      break;
    }
  }


  Serial.println(F("Initialise the radio"));
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Initialise radio failed"), state, true);

  os_getDevEui((uint8_t*)&devEUI);
  // Setup the OTAA session information
  //state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  state = node.beginOTAA(joinEUI, devEUI, NULL, appKey);
  debug(state != RADIOLIB_ERR_NONE, F("Initialise node failed"), state, true);

  Serial.println(F("Join ('login') the LoRaWAN Network"));
  state = node.activateOTAA();
  debug(state != RADIOLIB_LORAWAN_NEW_SESSION, F("Join failed"), state, true);

  Serial.println(F("Ready!\n"));
  //node.setSleepFunction(mdelay_long);
}

struct {
  unsigned short co2;
  unsigned short tvoc;
  byte power;
  byte rate2;
  signed short temp;
} mydata;

#define voltagePin    PA1

void readData()
{
    adc_enable(ADC1);

    pinMode(voltagePin, INPUT_ANALOG);
    const int vref = 4095;

    analogRead(voltagePin);
    int v = analogRead(voltagePin);
    //mydata.power = (2 * (VREF * v / vref) / 10) - 200;

    adc_reg_map *regs = ADC1->regs;
    regs->CR2 |= ADC_CR2_TSVREFE; // enable VREFINT and temp sensor
    regs->SMPR1 = (ADC_SMPR1_SMP17 | ADC_SMPR1_SMP16); // sample rate for VREFINT/TMP ADC channel

    int vrefi = 1210 * 4096 / adc_read(ADC1, 17); // ADC sample to millivolts
    mydata.power = (2 * (vrefi * v / vref) / 10) - 200;
}

void loop() {
  Serial.println(F("Sending uplink"));

  readData();
  
  // Perform an uplink
  int16_t state = node.sendReceive((uint8_t*)&mydata, sizeof(mydata));    
  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);

  // Check if a downlink was received 
  // (state 0 = no downlink, state 1/2 = downlink in window Rx1/Rx2)
  if(state > 0) {
    Serial.println(F("Received a downlink"));
  } else {
    Serial.println(F("No downlink received"));
  }

  Serial.print(F("Next uplink in "));
  Serial.print(uplinkIntervalSeconds);
  Serial.println(F(" seconds\n"));
  Serial.flush();
  
  SPI.end();
#if 1 // Do we need all this?
  digitalWrite(SCK_PIN, LOW);
  pinMode(SCK_PIN, OUTPUT);

  digitalWrite(MOSI_PIN, LOW);
  pinMode(MOSI_PIN, OUTPUT);

  pinMode(MISO_PIN, INPUT_ANALOG);

  digitalWrite(CS_PIN, LOW); // NSS
  pinMode(CS_PIN, OUTPUT);
#endif

  // DIO Inputs
  pinMode(DIO1_PIN, INPUT_ANALOG);
  pinMode(DIO0_PIN, INPUT_ANALOG);

  //pinMode(RESET_PIN, INPUT_ANALOG);

  // Serial
  pinMode(PA9, INPUT_ANALOG);
  pinMode(PA10, INPUT_ANALOG);

  // Wait until next uplink - observing legal & TTN FUP constraints
  mdelay(uplinkIntervalSeconds * 1000UL);  // delay needs milli-seconds

  //digitalWrite(RESET_PIN, 1); // prevent reset
  pinMode(DIO1_PIN, INPUT);
  pinMode(DIO0_PIN, INPUT);

  SPI.begin();
}
