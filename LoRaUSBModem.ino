#include "Arduino.h"
//#include <WiFi.h>

#ifdef CubeCell_GPS
#include "LoRaWan_APP.h"
#include "cubecell_SSD1306Wire.h"
#endif

#ifdef WIFI_LoRa_32_V2
#include <ESP32_LoRaWAN.h>
#endif

#ifdef CubeCell_GPS
extern SSD1306Wire  display;
SSD1306Wire Display = display;
#endif

/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#ifndef LoraWan_RGB
#define LoraWan_RGB 0
#endif

#define RF_FREQUENCY                                446420000 // Hz

#define TX_OUTPUT_POWER                             20        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       9         // [SF7..SF12]
#define LORA_CODINGRATE                             4         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            0 //1000
#define TX_TIMEOUT_VALUE                            3000
#define BUFFER_SIZE                                 256     // Define the payload size here

#define ASCII_SOH                                   1       // Start Of Heading
#define ASCII_EOT                                   4       // End Of Transmission
#define ASCII_ESC                                   27      // Escape

// License only necessary for WIFI_LORA_32_V2. See https://github.com/HelTecAutomation/ESP32_LoRaWAN#how-to-use-this-library
uint32_t license[4] = {0x9733C0BA,0xA5D9BBBD,0x9601490B,0x210EDBFB};

enum LEDState { LED_RX, LED_TX, LED_OFF };

enum ReadState { RS_IDLE, RS_READ, RS_ESC };
volatile ReadState state;

static RadioEvents_t RadioEvents;

int16_t rssi,rxSize;

volatile uint8_t txBuf[BUFFER_SIZE];
volatile int16_t txBufPos;

void setup() {
#ifdef CubeCell_GPS
    boardInitMcu( );
#endif

#ifdef WIFI_LoRa_32_V2
    SPI.begin(SCK,MISO,MOSI,SS);
    Mcu.init(SS,RST_LoRa,DIO0,DIO1,license);
    pinMode(LED,OUTPUT);
    pinMode(Vext,OUTPUT);
    digitalWrite(Vext,LOW);
    delay(20);
#endif

    Serial.begin(115200);
    
    Display.init();

    rssi = txBufPos = 0;
    state = RS_IDLE;
  
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.RxTimeout = EnableRx;
    RadioEvents.TxDone = EnableRx;
    RadioEvents.TxTimeout = OnTxTimeout;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
  
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                        LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
                        LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON,
                        TX_TIMEOUT_VALUE );
   
    EnableRx();
    
    Display.clear();
    Display.drawString(0, 0, "ATMS Modem ready.");
    Display.display();
}


void loop()
{
  Radio.IrqProcess();
  if (Serial.available()) serialEvent();
}

void SetLED(LEDState s)
{
#ifdef CubeCell_GPS
  switch (s)
  {
    case LED_RX:
      turnOnRGB(COLOR_RECEIVED,0);
      break;

    case LED_TX:
      turnOnRGB(COLOR_SEND,0);
      break;

    case LED_OFF:
      turnOffRGB();
      break;
  }
#endif

#ifdef WIFI_LoRa_32_V2
  switch(s)
  {
    case LED_RX:
    case LED_TX:
      digitalWrite(LED,HIGH);
      break;

    case LED_OFF:
      digitalWrite(LED,LOW);
      break;
  }
#endif
}

String parseCallsign(uint8_t *payload, uint16_t size)
{
  String call;

  for (int i = 0; i < size; i++)
  {
    if ((payload[i] & 0x80) == 0)
    {
      call += (char)payload[i];
    }
    else
    {
      uint8_t ssid = payload[i] - 128;
      if (ssid > 0)
      {
        call += '-';
        call += ssid;
      }

      return call;
    }
  }
}

void writePayload(uint8_t *payload, uint16_t size)
{
  for (int i = 0; i < size; i++)
  {
    char c = payload[i];
    if (c == ASCII_SOH || c == ASCII_EOT || c == ASCII_ESC)
    {
      Serial.write(ASCII_ESC);
    }
    Serial.write(c);
  }
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t Rssi, int8_t snr )
{
    SetLED(LED_RX);
    rssi=Rssi;
    rxSize=size;
    //Radio.Sleep( );
    Serial.write(ASCII_SOH);
    Serial.write(rssi >> 8);
    Serial.write(rssi);
    Serial.write(snr);
    writePayload(payload, size);
    Serial.write(ASCII_EOT);

    Display.clear();
    Display.drawString(0, 0, "Last rx:");
    Display.drawString(0, 15, parseCallsign(payload, size));
    char str[30];
    sprintf(str, "RSSI: %d   SNR: %d", rssi, snr);
    Display.drawString(0, 30, str);
    Display.display();

    SetLED(LED_OFF);
}

void OnTxTimeout()
{
  Display.clear();
  Display.drawString(0, 0, "TX timed out!");
  Display.display();
  EnableRx();
}

void EnableRx()
{
  Radio.Standby();
  Radio.Rx(RX_TIMEOUT_VALUE);
  SetLED(LED_OFF);
}

void txPayload()
{
  Radio.Standby();
  Radio.Send((uint8_t*)txBuf, txBufPos);
  SetLED(LED_TX);
}

void serialEvent()
{
  while (Serial.available())
  {
    uint8_t c = Serial.read();

    if (txBufPos >= BUFFER_SIZE)
    // buffer overflow
    {
      txBufPos = 0;
      state = RS_IDLE;
    }

    switch (state)
    {
      case RS_IDLE:
        // Idle: wait for SOH, ignore everything else
        if (c == ASCII_SOH)
        {
          state = RS_READ;
          txBufPos = 0;
        }
        break;

      case RS_READ:
        // Read: fill txBuf with incoming bytes. Unpack escaped bytes
        if (c == ASCII_EOT)
        {
          txPayload();
          state = RS_IDLE;
        }
        else if (c == ASCII_ESC)
        {
          state = RS_ESC;
        }
        else
        {
          txBuf[txBufPos++] = c;
        }
        break;

      case RS_ESC:
        // Escape: this byte goes directly into the buffer
        txBuf[txBufPos++] = c;
        state = RS_READ;
        break;

      default:
        // We should never make it here.
        state = RS_IDLE;
        txBufPos = 0;
        break;
    }
  }
}
