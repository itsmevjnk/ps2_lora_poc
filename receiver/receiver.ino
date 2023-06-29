/* based on LoRaDuplex and LoRaReceiver */

#include <SPI.h>
#include <PS2X_lib.h>
#include <LoRa.h>

PS2X ps2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); while(!Serial);

  Serial.println(F("PS2 over LoRa - Receiver POC"));

  LoRa.setPins(10, -1, -1); // CS on pin 10, no reset, no IRQ

  if(!LoRa.begin(433E6)) { // on 433MHz
    Serial.println(F("LoRa init failed"));
    while(1);
  }

  Serial.println(F("LoRa init OK"));
}

/* command bytes (from receiver) */
#define CMD_POLL                0x01 // poll PS2 controller

/* response bytes (from transmitter) */
#define RESP_POLL_OK            0xA0 // poll succeeded (followed by packet)
#define RESP_POLL_FAIL          0xF0 // poll failed (no packet)

#define ADDR_TX                 0x55 // transmitter address
#define ADDR_RX                 0xAA // receiver address

//#define RESP_TIMEOUT            30 // response timeout (mS), undefine to disable

uint32_t t_min = 0xFFFFFFFF, t_max = 0, t_avg = 0;
float packets = 0;

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(F("Polling transmitter..."));
  LoRa.beginPacket();
  LoRa.write(ADDR_RX);
  LoRa.write(CMD_POLL);
  if(!LoRa.endPacket()) {
    Serial.println(F("failed."));
    return; // go back to beginning of loop()
  }
  Serial.println(F("OK."));
  uint32_t t_start = millis();
#ifdef RESP_TIMEOUT
  while(millis() - t_start < RESP_TIMEOUT) {
#else
  while(1) {
#endif
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

      if(packet[0] == ADDR_TX) {
        if(packet[1] == RESP_POLL_OK) {
          Serial.println(F("Response packet has arrived"));
          for(uint8_t i = 0; i < 6; i++) ps2.PS2data[i + 3] = packet[i + 2]; // copy to PS2data
          /* imitate read_gamepad() */
          ps2.last_buttons = ps2.buttons; //store the previous buttons states
          ps2.buttons =  (uint16_t)(ps2.PS2data[4] << 8) + ps2.PS2data[3];   //store as one value for multiple functions
          packets++;
          uint32_t t_pkt = millis() - t_start;
          if(t_min > t_pkt) t_min = t_pkt;
          if(t_max < t_pkt) t_max = t_pkt;
          t_avg = (t_avg * (float)(packets - 1) + (float)t_pkt) / (float)packets;
          Serial.print(F("Waiting time: ")); Serial.print(t_pkt); Serial.print(F("mS (min: ")); Serial.print(t_min); Serial.print(F(", max: ")); Serial.print(t_max); Serial.print(F(", avg: ")); Serial.print(t_avg); Serial.println(')');
          ps2_loop(); // run loop
        } else if(packet[1] == RESP_POLL_FAIL) {
          Serial.println(F("Controller error"));
        }
        return;
      }
    }
  }
#ifdef RESP_TIMEOUT
  /* if we end up here then we've timed out */
  Serial.println(F("Timed out waiting for response"));
#endif
}

/* copied from PS2X example code */
void ps2_loop() {
  if(ps2.Button(PSB_START))         //will be TRUE as long as button is pressed
    Serial.println("Start is being held");
  if(ps2.Button(PSB_SELECT))
    Serial.println("Select is being held");      

  if(ps2.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
    Serial.print("Up held this hard: ");
    Serial.println(ps2.Analog(PSAB_PAD_UP), DEC);
  }
  if(ps2.Button(PSB_PAD_RIGHT)){
    Serial.print("Right held this hard: ");
    Serial.println(ps2.Analog(PSAB_PAD_RIGHT), DEC);
  }
  if(ps2.Button(PSB_PAD_LEFT)){
    Serial.print("LEFT held this hard: ");
    Serial.println(ps2.Analog(PSAB_PAD_LEFT), DEC);
  }
  if(ps2.Button(PSB_PAD_DOWN)){
    Serial.print("DOWN held this hard: ");
    Serial.println(ps2.Analog(PSAB_PAD_DOWN), DEC);
  }   

  if (ps2.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
    if(ps2.Button(PSB_L3))
      Serial.println("L3 pressed");
    if(ps2.Button(PSB_R3))
      Serial.println("R3 pressed");
    if(ps2.Button(PSB_L2))
      Serial.println("L2 pressed");
    if(ps2.Button(PSB_R2))
      Serial.println("R2 pressed");
    if(ps2.Button(PSB_TRIANGLE))
      Serial.println("Triangle pressed");        
  }

  if(ps2.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
    Serial.println("Circle just pressed");
  if(ps2.NewButtonState(PSB_CROSS))               //will be TRUE if button was JUST pressed OR released
    Serial.println("X just changed");
  if(ps2.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
    Serial.println("Square just released");     

  if(ps2.Button(PSB_L1) || ps2.Button(PSB_R1)) { //print stick values if either is TRUE
    Serial.print("Stick Values:");
    Serial.print(ps2.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
    Serial.print(",");
    Serial.print(ps2.Analog(PSS_LX), DEC); 
    Serial.print(",");
    Serial.print(ps2.Analog(PSS_RY), DEC); 
    Serial.print(",");
    Serial.println(ps2.Analog(PSS_RX), DEC); 
  }   
}
