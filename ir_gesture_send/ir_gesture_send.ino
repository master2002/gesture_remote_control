#include <IRremote.h>
#include <Wire.h>

IRsend irsend;

enum ir_command {Undefined, Forward, Reverse, Stop, 
                 LeftSpin, RightSpin, LeftForward, RightForward, LeftReverse, RightReverse,
                 Record, Replay1, Replay2, Replay3, Replay4};

int data = 0;
//int old_data = 0;
bool new_data = false; 

void setup()
{
  Serial.begin(9600);
  Serial.println("IR Gesture Sender");
  Serial.println("======================");

  Wire.begin(4);
  Wire.onReceive(receiveEvent);
}

void loop() {
  if (new_data) {
    int copy_data = data;
    // output = PWM pin3, 200ohm range = 3 feet, 100ohm range = 5 feet
    char *gesture = "Undefined";
    switch (copy_data) {
      case Forward:   irsend.sendNEC(0xFFAA55,32); gesture = "Forward"; break;
      case Reverse:   irsend.sendNEC(0xFF629D,32); gesture = "Reverse"; break;
      case Stop:      irsend.sendNEC(0xFF6A95,32); gesture = "Stop"; break;
      case LeftSpin:  irsend.sendNEC(0xFF5AA5,32); gesture = "LeftSpin"; break;
      case RightSpin: irsend.sendNEC(0xFF4AB5,32); gesture = "RightSpin"; break;
      default: break;
    }
    new_data = false;
    Serial.println("I2C_slave(4): transmitted gesture = " + String(gesture));
  }
}

void receiveEvent(int howMany)
{
  data = Wire.read();
  new_data = true;
  //Serial.println(data);
}

