#include <esp32_can.h>
#include "BluetoothSerial.h"

word count;
word engine_rpm;
byte brake_front;
byte gear;
byte gear_raw;
short lean_angle;
byte throttle_position;
byte traction_control;

//TODO: Migrate to c-string
String rc3;

//leds for resja can
#define BLUE_LED 13       // GPIO13  (GPIO4 on v2.0)
#define YELLOW_LED 12     // GPIO12
#define FORCE_KEEP_ON 25  // GPIO25

//ESP32 BT
BluetoothSerial SerialBT;


void setup() {

  Serial.begin(115200);

  SerialBT.begin("RC2_CAN_TTGO");
  delay(1000);

  //resjacan initial setup
  pinMode(26, OUTPUT);  // THE CAN LIBRARY HAS THIS PIN FOR INTERRUPT FOR CAN1 (UNSUSED HERE) INPUT WITHOUT PULLUP, FORCE TO OUTPUT INSTEAD TO PREVENT ERRONEOUS INTERRUPTS.
  pinMode(FORCE_KEEP_ON, OUTPUT);
  digitalWrite(FORCE_KEEP_ON, HIGH);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);

  //we aren't using default CAN pins, set correct pins
  CAN0.setCANPins(GPIO_NUM_26, GPIO_NUM_27);

  //The S1000XR uses a 500K bus
  if (!CAN0.begin(CAN_BPS_500K)) {
    Serial.println("CAN0 Init Failed");
  } else {
    Serial.println("CAN0 Init");
    digitalWrite(YELLOW_LED, HIGH);
  }

  //we are interested in 4 PIDS
  //here are all the pids
  //https://docs.google.com/spreadsheets/d/1Tn1NUKs-iS0VNayEWnzZ38CHXNaqZsz723aXRCn4e_U/edit?usp=sharing
  Can0.setRXFilter(0, 0x10C, 0x7FF, false);
  Can0.setRXFilter(1, 0x110, 0x7FF, false);
  Can0.setRXFilter(2, 0x120, 0x7FF, false);
  Can0.setRXFilter(3, 0x2BC, 0x7FF, false);

  //now register all of the callback functions.
  Can0.setCallback(0, gotFrameMB0);
  Can0.setCallback(1, gotFrameMB1);
  Can0.setCallback(2, gotFrameMB2);
  Can0.setCallback(3, gotFrameMB3);

  //show that the device is ready
  digitalWrite(BLUE_LED, HIGH);
}


void gotFrameMB0(CAN_FRAME *frame) {
  engine_rpm = (frame->data.bytes[2] + ((frame->data.bytes[3] & B00001111) * 255)) * 5;
  lean_angle = 90 - 0.707107 * (frame->data.bytes[4] + 0.1 * (frame->data.bytes[3] >> 4));
}

void gotFrameMB1(CAN_FRAME *frame) {
  traction_control = 0.395257 * (253 - frame->data.bytes[4]);
  throttle_position = 0.5 * (frame->data.bytes[5] - 36);

  //keep it positive
  if (throttle_position < 0) {
    throttle_position = 0;
  }

  write_sentence();
}

void gotFrameMB2(CAN_FRAME *frame) {
  brake_front = frame->data.bytes[2] / 2.53;
  write_sentence();
}

void gotFrameMB3(CAN_FRAME *frame) {
  gear_raw = frame->data.bytes[5] >> 4;
  switch (gear_raw) {
    case 1:
      gear = 1;
      break;
    case 2:
      gear = 0;
      break;
    case 4:
      gear = 2;
      break;
    case 7:
      gear = 3;
      break;
    case 8:
      gear = 4;
      break;
    case 11:
      gear = 5;
      break;
    case 13:
      gear = 6;
      break;
    case 6:
      gear = 1;
      break;
    default:
      gear = 0;
      break;
  }
}

void loop() {
  //tis empty as we are doing everything via interrupts
}

void write_sentence() {
  //need String for checksum
  //move to c-string

  //get the time for RC
  //we are using the RTC as RC is sensitive to non stable update rates
  //we cast to word to keep under 65535 for RC
  count = word(millis());

  rc3 = "$RC2,,";
  rc3 += count;
  rc3 += ",,,,";
  rc3 += engine_rpm; //using the RPM field
  rc3 += ",,";
  rc3 += brake_front;
  rc3 += ",";
  rc3 += gear;
  rc3 += ",";
  rc3 += lean_angle;
  rc3 += ",";
  rc3 += throttle_position;
  rc3 += ",";
  rc3 += traction_control;
  rc3 += ",";
  rc3 += count; //expose for CSV vetting
  rc3 += ",,*";
  rc3 += checksum(rc3);
  rc3 += "\r\n";

  //Serial.print(rc3);
  SerialBT.print(rc3);
}

String checksum(String s)  // Checksum calculation
{
  byte cs = 0x00;
  const char *buf = s.c_str();
  for (unsigned int i = 1; i < s.length() - 1; i++) {
    cs = cs ^ buf[i];  // XOR
  }
  return String((cs <= 0x0f ? "0" : "") + String(cs, HEX));
}
