#include "LoRaWan_APP.h"
#include "Arduino.h"

// Define o pino de toque a ser usado. Usamos o pino 4, que corresponde ao Touch 0 no seu Heltec V3.
#define TOUCH_PIN 4

#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 14 // dBm
#define LORA_BANDWIDTH 0 // [0: 125 kHz]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1 // [1: 4/5]
#define LORA_PREAMBLE_LENGTH 8 // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0 // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 30 // Define the payload size here

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

int16_t txNumber;
int16_t rssi,rxSize;
bool lora_idle = true;

// Variáveis para a função de toque
bool touch_detected = false;
int touch_threshold = 15; // Ajuste este valor. Um valor menor é mais sensível.

// Callback de recebimento de pacote
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

// Função de interrupção para o toque
void IRAM_ATTR onTouch();


void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Calibrando o sensor de toque...");
    Serial.println("Valor do toque (sem tocar)");
    
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
    
    // Configuração do pino de toque, com a interrupção atachada
    touchAttachInterrupt(TOUCH_PIN, onTouch, touch_threshold);
    
    Serial.println("LoRa Receiver Initializing...");
    
    txNumber=0;
    rssi=0;

    RadioEvents.RxDone = OnRxDone;
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                         0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

    Serial.println("LoRa Ready. Entering RX mode...");
}

void loop()
{
  int touchValue = touchRead(TOUCH_PIN);
  //Serial.println(touchValue);
  delay(200);

    if(touch_detected) {
      Serial.println("PIN TOUCH DETECTADO!");
      touch_detected = false;
    }
    
    if(lora_idle) {
        lora_idle = false;
        Serial.println("into RX mode");
        Radio.Rx(0);
    }
    Radio.IrqProcess( );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi_val, int8_t snr ) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    
    rssi = rssi_val;
    
    Radio.Sleep();
    
    Serial.print("Received packet: ");
    Serial.println(rxpacket);
    Serial.print("RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.print(snr);
    Serial.println(" dB");

    lora_idle = true;
}

void IRAM_ATTR onTouch() {
  touch_detected = true;
}