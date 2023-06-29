/* based on LoRaDuplex and LoRaReceiver */

#include <SPI.h>
#include <PS2X_lib.h>
#include <LoRa.h>

#define PS2_DAT                 A3
#define PS2_CMD                 A2
#define PS2_ATT                 A1
#define PS2_CLK                 A0

PS2X ps2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); while(!Serial);

  Serial.println(F("PS2 over LoRa - Transmitter POC"));

  LoRa.setPins(10, -1, -1); // CS on pin 10, no reset, no IRQ

  if(!LoRa.begin(433E6)) { // on 433MHz
    Serial.println(F("LoRa init failed"));
    while(1);
  }

  Serial.println(F("LoRa init OK"));

  int ret = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, false, false);
  if(ret != 0 && ret != 3) {
    Serial.println(F("PS2 controller init failed"));
    while(1);
  }

  Serial.println(F("PS2 controller init OK"));

  Serial.println(F("Ready to accept packets"));
}

/* command bytes (from receiver) */
#define CMD_POLL                0x01 // poll PS2 controller

/* response bytes (from transmitter) */
#define RESP_POLL_OK            0xA0 // poll succeeded (followed by packet)
#define RESP_POLL_FAIL          0xF0 // poll failed (no packet)

#define ADDR_TX                 0x55 // transmitter address
#define ADDR_RX                 0xAA // receiver address

void loop() {
  // put your main code here, to run repeatedly:
  int pkt_size = LoRa.parsePacket();

  if(pkt_size) {
    Serial.print(F("Received a packet: "));
    uint8_t packet[16];
    for(uint8_t i = 0; i < pkt_size; i++) {
      packet[i] = LoRa.read();
      Serial.print(packet[i], HEX); Serial.print(' ');
    }
    Serial.print(F(" (RSSI: ")); Serial.print(LoRa.packetRssi(), DEC);
    Serial.print(F(", SNR: ")); Serial.print(LoRa.packetSnr());
    Serial.println(')');

    if(pkt_size == 2 && packet[0] == ADDR_RX) { // valid address and size
      if(packet[1] == CMD_POLL) {
        bool ret = ps2.read_gamepad(false, 0);
        LoRa.beginPacket();
        LoRa.write(ADDR_TX);
        LoRa.write((ret) ? RESP_POLL_OK : RESP_POLL_FAIL);
        if(ret) {
          Serial.println(F("PS2 polling OK, sending packet to receiver"));
          LoRa.write(&ps2.PS2data[3], 6); // buttons + joysticks
        } else Serial.println(F("PS2 polling failed"));
        if(LoRa.endPacket()) Serial.println(F("Response sent"));
        else Serial.println(F("Cannot send response"));
      }
    }
  }
}
