#include "LoRaWan_APP.h"
#include "Arduino.h"

#define TOUCH_PIN 4

#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 5 // dBm
#define LORA_BANDWIDTH 0 // 125 kHz
#define LORA_SPREADING_FACTOR 7 // SF7
#define LORA_CODINGRATE 1 // 4/5
#define LORA_PREAMBLE_LENGTH 8 // Mesma para Tx e Rx
#define LORA_SYMBOL_TIMEOUT 0 // Simbolos
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30 

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;

bool lora_idle=true;
bool touch_detected = false;
int touch_threshold = 40; // Valor de sensibilidade do pino touch

static RadioEvents_t RadioEvents;
void OnTxDone( void );
void OnTxTimeout( void );
void IRAM_ATTR onTouch();

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    
    // Configuração do pino de toque
    touchAttachInterrupt(TOUCH_PIN, onTouch, touch_threshold);
    
    txNumber=0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         true, 0, 0, LORA_IQ_INVERSION_ON, 3000 ); 
    }

void loop() {
  if(touch_detected) {
    Serial.println("PIN TOUCH DETECTADO!");
    touch_detected = false;
  }
  
  if(lora_idle == true) {
    delay(1000);
    txNumber += 0.01;
    sprintf(txpacket,"Hello world number %0.2f",txNumber); 
    
    Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); 
    lora_idle = false;
  }
  Radio.IrqProcess( );
}

void IRAM_ATTR onTouch() {
  touch_detected = true;
}

void OnTxDone(void) {
  Serial.println("TX done......");
  lora_idle = true;
}

void OnTxTimeout(void) {
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}