#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HerkulexServo.h>

#define PIN_SW_RX 10
#define PIN_SW_TX 11

SoftwareSerial   servo_serial(PIN_SW_RX, PIN_SW_TX);
HerkulexServoBus herkulex_bus(servo_serial);
HerkulexServo    *servo [8];

unsigned long last_update = 0;
unsigned long now = 0;
uint8_t cableId = 0;

const uint8_t NB_CABLES = 8;
const uint16_t SMIN = 297;
const uint16_t SMAX = 666;
const uint16_t LMIN = 974;
const uint16_t LMAX = 297;
const uint16_t SINIT = SMIN+abs(SMAX-SMIN)/4;
const uint16_t LINIT = LMIN-abs(LMIN-LMAX)/4;
const uint16_t RANGE = 25;

void setup() {
  Serial.begin(115200);
  servo_serial.begin(115200);
  delay(500);
  for(uint8_t i = 0; i<NB_CABLES; i++) {
    servo[i] = new HerkulexServo(herkulex_bus, i);
    servo[i]->setTorqueOn();  // turn power on
    Serial.print(i);
    Serial.print(") ");
    uint16_t init = (i<4) ? SINIT : LINIT;
    servo[i]->setPosition(init);
    Serial.print(init);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);
}

void loop() {
  herkulex_bus.update();

  if (Serial.available() > 0) {
    char c = Serial.read();

    String ids = "012345678";
    int isid = ids.indexOf(c);
    if (isid > -1) {
        cableId = c-'0';
        Serial.print("Selected cable: ");
        Serial.println(cableId);
    }
    else if(c == 'p') {
        // pull cable
        uint16_t position = servo[cableId]->getPosition();
        position = (cableId < 4) ? position+RANGE : position-RANGE;
        servo[cableId]->setPosition(position, 100);
        Serial.print("+");
    }
    else if(c=='m') {
        // release cable
        uint16_t position = servo[cableId]->getPosition();
        position = (cableId < 4) ? position-RANGE : position+RANGE;
        servo[cableId]->setPosition(position, 100);
        Serial.print("-");
    }
    else if(c=='a') {
        for(uint8_t i = 0; i < NB_CABLES; i++) {
        Serial.print(i);
        Serial.print(") ");
        Serial.print(servo[i]->getPosition());
        Serial.print(" ");
        }
        Serial.println();
    }
  }
}
