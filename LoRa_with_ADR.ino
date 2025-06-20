#include <SPI.h>
#include <LoRa.h>
#define LORA_SCK    5 //pin definitions (can change)
#define LORA_MISO   19
#define LORA_MOSI   27
#define LORA_SS     18
#define LORA_RST    14
#define LORA_DIO0   26

// RSSI is received signal strength indicatior lower the value better the strength (can be changed based on our requirements)
const int RSSI_HIGH_THRESH = -60;   // strong signal - need to speed up
const int RSSI_LOW_THRESH  = -100;  // weak signal - need to slow down
int spreadingFactors[]   = {12, 11, 10, 9, 8, 7}; // list of parameters to cycle between
long bandwidths[]        = {125E3, 250E3, 500E3}; // 125 kHz 250kHz 500kHz
int codingRates[]        = {5, 6, 7, 8};  // 4/5 4/6 4/7 4/
int idxSF = 0;   // starting with max range
int idxBW = 0;   
int idxCR = 0;   

void setup() {
  Serial.begin(9600);
  while (!Serial);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0); // setting LoRa pins
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("LoRa init OK");
  applyLoRaParams();
}

void loop() {
  LoRa.beginPacket();   // Sending a packet
    LoRa.print("Ping");
  LoRa.endPacket();
  Serial.print("Sent @ SF"); Serial.print(spreadingFactors[idxSF]); // so that we know the current settings
  Serial.print(" BW"); Serial.print(bandwidths[idxBW]/1e3);
  Serial.print("kHz CR4/"); Serial.println(codingRates[idxCR]);
  unsigned long start = millis();   // Listening for a reply (blocking for simplicity)
  while (millis() - start < 2000) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      String rx = LoRa.readString();
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();
      Serial.print("Recv: "); Serial.print(rx);
      Serial.print("  RSSI: "); Serial.print(rssi);
      Serial.print(" dBm  SNR: "); Serial.println(snr);
      adjustADR(rssi);
      break;
    }
  }
  delay(5000); // wait before next ping
}

void applyLoRaParams() { // Applying current parameter settings to the radio
  LoRa.setSpreadingFactor(spreadingFactors[idxSF]);
  LoRa.setSignalBandwidth(bandwidths[idxBW]);
  LoRa.setCodingRate4(codingRates[idxCR]);
  LoRa.setTxPower(17); // tx power can be tweaked too
}

// ADR ALGORTIHM 
void adjustADR(int lastRssi) {
  if (lastRssi > RSSI_HIGH_THRESH) {
    // trying to speed up: 1st priority increase BW, then lower SF, then higher CR
    if (idxBW < 2) { 
      idxBW++;
      Serial.println("ADR: increasing BW");
    }
    else if (idxSF > 0) {
      idxSF--;
      Serial.println("ADR: decreasing SF");
    }
    else if (idxCR < 3) {
      idxCR++;
      Serial.println("ADR: increasing CR");
    }
    else {
      Serial.println("ADR: link already at max data-rate");
    }
  }
  else if (lastRssi < RSSI_LOW_THRESH) {
    // if link is poor: stepping to lower data-rate
    if (idxSF < 5) {
      idxSF++;
      Serial.println("ADR: increasing SF");
    }
    else if (idxBW > 0) {
      idxBW--;
      Serial.println("ADR: decreasing BW");
    }
    else if (idxCR > 0) {
      idxCR--;
      Serial.println("ADR: decreasing CR");
    }
    else {
      Serial.println("ADR: link already at max range settings");
    }
  }
  else {
    Serial.println("ADR: link within good margin—no change");
  }
  applyLoRaParams();
}
