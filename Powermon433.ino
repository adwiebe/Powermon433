/*

 Powermon433
 
 Monitoring 433MHz power monitor products (both the same internally)
 -- Black and Decker EM100B 
 -- BlueLine PowerCost Monitor
 
 Protocol decoding by "redgreen" 
 https://github.com/merbanan/rtl_433
 
 via comments on Stewart C. Russell's blog 
 http://scruss.com/blog/2013/12/03/blueline-black-decker-power-monitor-rf-packets/
 
 Additional work (aka "most of it" - SCR) 
 and porting to ATmega by Bryan Mayland
 https://github.com/CapnBry/Powermon433
 
 Original OOK interrupt and decoder based on jeelib 
 "ookRelay2" project https://github.com/jcw/jeelib
 
 General messing about and reformatting 
 by Stewart C. Russell, scruss.com
 https://github.com/scruss/Powermon433

 Dec 2016 - added the following features
 -- store TX_ID in EPROM (prevents need to reset after every power failure)
 -- mechanical lock out switch to replace TX_ID_LOCK
 -- use status LED (pin 13) for
 ---- heartbeat - blink every timeout
 ---- looking ID - double blink
 ---- found ID - tripple blink
 
 */

#include <util/atomic.h>
#include <EEPROM.h>
// Remove rf69_ook.h - not using that 
// #include "rf69_ook.h"
#include "temp_lerp.h"

#define POWERFACTOR         7.2

// the pin connected to the receiver output
#define DPIN_OOK_RX         8

// Status LEDs and colours
#define DPIN_STATUS         13
#define DPIN_GRN_LED        11
#define DPIN_RED_LED        12
#define STATUS_OFF          0
#define STATUS_RED          1
#define STATUS_GREEN        2
#define STATUS_ORANGE       3

#define STATE_GOOD          1
#define STATE_CRCERR        2
#define STATE_MISSED        3
#define STATE_ID_SYNC_ON    4
#define STATE_ID_RECD       5

/*
 The default ID of the transmitter to decode from
 is whatever random bytes are in EEPROM addr 0&1
 - it is likely that yours is different.
 Jumper this PIN to HIGH, then press RESET on the
 transmitter. Once Status LED is single blinking 
 and/or colour LED is Green, jumper can be removed.
 */
#define DPIN_SYNC           5

/*
 TX_ID_LOCK - 
 Uncomment this #define if you want to lock the decoder ID.
 This will prevent nearby stations hijacking your power log
 */
//#define TX_ID_LOCK

/*
 TEMPERATURE_F - 
 Uncomment this #define if you believe in the power of
 D. G. Farenheit's armpit, and want your temperature in °F.
 Otherwise, you get °C, like you should.
 [I'd include an option for K, but it doesn't fit into a byte.]
 */
//#define TEMPERATURE_F

static uint16_t g_TxId;

static struct tagWireVal
{
  uint8_t hdr;
  union {
    uint8_t raw[2];
    uint16_t val16;
  } 
  data;
  uint8_t crc;
} 
wireval;

static struct tagDecoder
{
  uint8_t state;
  uint8_t pos;
  uint8_t bit;
  uint8_t data[4];
} 
decoder;

static int8_t g_RxTemperature;
static uint8_t g_RxFlags;
static uint16_t g_RxWatts;
static uint32_t g_RxWattHours;

static unsigned long g_TotalRxWattHours;
static uint32_t g_PrevRxWattHours;

// better stats on time between reports
// delta is long as packets are appx ½ range of uint16_t
// so we roll if we miss >2
static unsigned long g_PrintTime_ms;
static unsigned long g_PrevPrintTime_ms;
static unsigned long g_PrintTimeDelta_ms;

static bool g_RxDirty;
static uint32_t g_RxLast;

#if DPIN_OOK_RX >= 14
#define VECT PCINT1_vect
#elif DPIN_OOK_RX >= 8
#define VECT PCINT0_vect
#else
#define VECT PCINT2_vect
#endif

void BlinkStatusLED(int cnt, int clr=STATUS_OFF)
{
  digitalWrite(DPIN_GRN_LED, clr & STATUS_GREEN);
  digitalWrite(DPIN_RED_LED, clr & STATUS_RED);
  // Orange is both Green & Red on together
  
  for (int x=0; x<cnt; x++)
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(100);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(250);  
  }

  delay(2500); // Wait 2.5 seconds before allowing next status to be displayed.
}


volatile uint16_t pulse_433;

static void pinChange(void)
{
  static uint16_t last_433;

  uint16_t now = micros();
  uint16_t cnt = now - last_433;
  if (cnt > 10)
    pulse_433 = cnt;

  last_433 = now;
}

ISR(VECT) {
  pinChange();
}

static void setupPinChangeInterrupt ()
{
  pinMode(DPIN_OOK_RX, INPUT);
#if DPIN_OOK_RX >= 14
  bitSet(PCMSK1, DPIN_OOK_RX - 14);
  bitSet(PCICR, PCIE1);
#elif DPIN_OOK_RX >= 8
  bitSet(PCMSK0, DPIN_OOK_RX - 8);
  bitSet(PCICR, PCIE0);
#else
  PCMSK2 = bit(DPIN_OOK_RX);
  bitSet(PCICR, PCIE2);
#endif
}

// Short burst in uSec
#define OOK_TX_SHORT 500
// Long burst in uSec
#define OOK_TX_LONG  1000
// Inter-packet delay in msec
#define OOK_TX_DELAY 65
// Inter-ID-packet delay in msec
#define OOK_ID_DELAY 225

#define OOK_PACKET_INSTANT 1
#define OOK_PACKET_TEMP    2
#define OOK_PACKET_TOTAL   3

/* crc8 from chromimum project */
__attribute__((noinline)) uint8_t crc8(uint8_t const *data, uint8_t len)
{
  uint16_t crc = 0;
  for (uint8_t j=0; j<len; ++j)
  {
    crc ^= (data[j] << 8);
    for (uint8_t i=8; i>0; --i)
    {
      if (crc & 0x8000)
        crc ^= (0x1070 << 3);
      crc <<= 1;
    }
  }
  return crc >> 8;
}

static void resetDecoder(void)
{
  decoder.pos = 0;
  decoder.bit = 0;
  decoder.state = 0;
}

static void decoderAddBit(uint8_t bit)
{
  decoder.data[decoder.pos] = (decoder.data[decoder.pos] << 1) | bit;
  if (++decoder.bit > 7)
  {
    decoder.bit = 0;
    if (++decoder.pos >= sizeof(decoder.data))
      resetDecoder();
  }
}

static bool decodeRxPulse(uint16_t width)
{
  // 500,1000,1500 usec pulses with 25% tolerance
  if (width > 375 && width < 1875)
  {
    // The only "extra long" long signals the end of the preamble
    if (width > 1200)
    {
      resetDecoder();
      return false;
    }

    bool isShort = width < 750;
    if (decoder.state == 0)
    {
      // expecting a short to start a bit
      if (isShort)
      {
        decoder.state = 1;
        return false;
      }
    }
    else if (decoder.state == 1)
    {
      decoder.state = 0;
      if (isShort)
        decoderAddBit(1);
      else
        decoderAddBit(0);

      // If we have all 3 bytes, we're done
      if (decoder.pos > 2)
        return true;
      return false;
    }
  }  // if proper width


  resetDecoder();
  return false;
}

static void decodePowermon(uint16_t val16)
{
  switch (decoder.data[0] & 3)
  {
  case OOK_PACKET_INSTANT:
    // val16 is the number of milliseconds between blinks
    // Each blink is one meter unit consumed (typically 1 Wh, but check your Power Factor
    g_RxWatts = (3600000UL / val16) * POWERFACTOR;
    break;

  case OOK_PACKET_TEMP:
    g_RxTemperature = (int8_t)(fudged_f_to_c(temp_lerp(decoder.data[1])));
    g_RxFlags = decoder.data[0];
    break;

  case OOK_PACKET_TOTAL:
    g_PrevRxWattHours = g_RxWattHours;
    g_RxWattHours = val16 * POWERFACTOR;
    // prevent rollover through the power of unsigned arithmetic
    g_TotalRxWattHours += (g_RxWattHours - g_PrevRxWattHours);
    break;
  }
}

static void decodeRxPacket(void)
{

  uint16_t val16 = *(uint16_t *)decoder.data;
  if (digitalRead(DPIN_SYNC) == HIGH) {
    BlinkStatusLED(STATE_ID_SYNC_ON, STATUS_ORANGE);
    if (crc8(decoder.data, 3) == 0)
    {
      g_TxId = decoder.data[1] << 8 | decoder.data[0];
      StoreTXID(g_TxId);
      Serial.print(F("# New ID: 0x"));
      Serial.println(val16, HEX);
      BlinkStatusLED(STATE_ID_RECD, STATUS_ORANGE);
      return;
    }
  }

  val16 -= g_TxId;
  decoder.data[0] = val16 & 0xff;
  decoder.data[1] = val16 >> 8;
  if (crc8(decoder.data, 3) == 0)
  {
    decodePowermon(val16 & 0xfffc);
    g_RxDirty = true;
    g_RxLast = millis();
  }
  else
  {
    Serial.println(F("# CRC ERR"));
    BlinkStatusLED(STATE_CRCERR, STATUS_RED);    
  }
}

static void rxSetup(void)
{
  setupPinChangeInterrupt();
}

static void ookRx(void)
{
  uint16_t v;
  ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
    v = pulse_433;
    pulse_433 = 0;
  }
  if (v != 0)
  {
    if (decodeRxPulse(v) == 1)
    {
      decodeRxPacket();
      resetDecoder();
    }
  }

  // If it has been more than 250ms since the last receive, dump the data
  else if (g_RxDirty && (millis() - g_RxLast) > 250U)
  {

    /*
     track duration since last report
     
     If > ~31.8 s (318nn ms), we have missed a packet, 
     and the instantaneous Power reading 
     isn't continuous. 
     */
    g_PrevPrintTime_ms = g_PrintTime_ms;
    g_PrintTime_ms = millis();
    g_PrintTimeDelta_ms = g_PrintTime_ms - g_PrevPrintTime_ms;

    Serial.print(F("PrintDelta_ms: ")); 
    Serial.print(g_PrintTimeDelta_ms, DEC);
    Serial.print(F(" Energy_Wh: ")); 
    Serial.print(g_RxWattHours, DEC);  
    Serial.print(F(" Total_Energy_Wh: ")); 
    Serial.print(g_TotalRxWattHours, DEC);
    Serial.print(F(" Power_W: ")); 
    Serial.print(g_RxWatts, DEC);
    Serial.print(F(" Temp_C: "));
    Serial.print(g_RxTemperature, DEC);
    Serial.print(F(" Flags: "));
    Serial.println(g_RxFlags, BIN);

    BlinkStatusLED(STATE_GOOD, STATUS_GREEN);
    
    g_RxDirty = false;
  }
  else if (g_RxLast != 0 && (millis() - g_RxLast) > 32000U) { 
    Serial.println(F("# Missed Packet"));
    g_RxLast = millis();
    BlinkStatusLED(STATE_MISSED,STATUS_RED);
  }
}

void StoreTXID(uint16_t ID)
{
  uint8_t b1, b2;

  b1 = ID;
  b2 = ID/256;

  EEPROM.write(0,b1);
  EEPROM.write(1,b2);
  
}

uint16_t RetrieveTXID()
{
  uint8_t b1,b2;

  b1 = EEPROM.read(0);
  b2 = EEPROM.read(1);

  return b1 + (b2 * 256);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DPIN_GRN_LED, OUTPUT);
  pinMode(DPIN_RED_LED, OUTPUT);
  pinMode(DPIN_SYNC, INPUT);
 
  BlinkStatusLED(5, STATUS_OFF);

  g_TxId = RetrieveTXID();
  Serial.begin(38400);
  Serial.print(F("# Powermon433 built "__DATE__" "__TIME__));
  Serial.print(F("# Listening for Sensor ID: 0x"));
  Serial.println(g_TxId, HEX);

  rxSetup();

  g_TotalRxWattHours = 0;
  g_PrintTimeDelta_ms = 0;
  g_PrintTime_ms = 0;
}

void loop()
{
  ookRx();
}

