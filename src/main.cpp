#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"
#include <SPI.h>


#define RF_FREQUENCY                                923000000 // Hz

#define TX_OUTPUT_POWER                             20        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       12         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            500
#define BUFFER_SIZE                                 30 // Define the payload size here

String msg;
String displayName;
String sendMsg;
char chr;
String mydata;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

void displayMcuInit();
void displaySendReceive();
extern SSD1306Wire display;

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}States_t;

int16_t txNumber;
States_t state;
bool sleepMode = false;
int16_t Rssi,rxSize;


void setup() {
    Serial.begin(115200);
    Mcu.begin();
    txNumber=0;
    Rssi=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone = OnRxDone;

    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 1000 );

    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
    state=STATE_TX;

display.init();
    display.flipScreenVertically();  
    display.setFont(ArialMT_Plain_10);

    //delay(1500);
    display.clear();
  
    display.drawString(0, 0, "Heltec.LoRa Initial success!");
    display.display();
    delay(1000);

}



void loop()
{

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  
  display.drawString(0, 0, "Sending packet: ");
  display.drawString(90, 0, String(txNumber));
  display.drawString(0, 20, rxpacket);
  display.display();

   

/*
  if (Serial.available()) {
    chr = Serial.read();
    Serial.print(chr); // so the user can see what they're doing :P
    if (chr == '\n' || chr == '\r') {
      msg += chr; //msg+='\0'; // should maybe terminate my strings properly....
      if (msg.startsWith("/")) {
        // clean up msg string...
        msg.trim(); msg.remove(0, 1);
        // process command...
        char cmd[1]; msg.substring(0, 1).toCharArray(cmd, 2);
        switch (cmd[0]){
          case '?':
            Serial.println("Supported Commands:");
            Serial.println("h - this message...");
            Serial.println("n - change Tx nickname...");
            Serial.println("d - print Tx nickname...");
            break;
          case 'n':
            displayName = msg.substring(2);
            Serial.print("Display name set to: "); Serial.println(displayName);
            break;
          case 'd':
            Serial.print("Your display name is: "); Serial.println(displayName);
            break;
          default:
            Serial.println("command not known... use 'h' for help...");
        }
        msg = "";
      }
      else {
        // ssshhhhhhh ;)
        Serial.print("Me: "); Serial.println(msg);
        // assemble message
        sendMsg += displayName;
        sendMsg += "> ";
        sendMsg += msg;

        txpacket = sendMsg.c_str();

        msg = "";
        sendMsg = "";
        Serial.print(": ");
      }
    }
    else {
      msg += chr;
    }
  }
*/


  
  switch(state)
  {
    case STATE_TX:
      delay(500);
      txNumber++;
      if (Serial.available()) {
        mydata = Serial.readStringUntil('\n');
        // TODO: Process the data
      }
      sprintf(txpacket,"%s, Rssi : %d",mydata,Rssi);
      Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));
      Radio.Send( (uint8_t *)txpacket, strlen(txpacket) );
      state=LOWPOWER;
      break;
    case STATE_RX:
      Serial.println("into RX mode");
      Radio.Rx( 0 );
      state=LOWPOWER;
      break;
    case LOWPOWER:
      Radio.IrqProcess( );
      break;
    default:
      break;
  }

  
}

void OnTxDone( void )
{
  Serial.print("TX done......");
  state=STATE_RX;
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.print("TX Timeout......");
    state=STATE_TX;
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Rssi=rssi;
    rxSize=size;
    memcpy(rxpacket, payload, size );
    rxpacket[size]='\0';
    Radio.Sleep( );

    Serial.printf("\r\nreceived packet \"%s\" with Rssi %d , length %d\r\n",rxpacket,Rssi,rxSize);
    Serial.println("wait to send next packet");

    state=STATE_TX;
}