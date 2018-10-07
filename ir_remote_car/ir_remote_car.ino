#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

#include <Servo.h>                           // Include servo library
#include <boarddefs.h>
#include <IRremote.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#define RECV_PIN 7
IRrecv irrecv(RECV_PIN);

Servo servoLeft;                             // Declare left and right servos
Servo servoRight;

enum ir_command {Undefined, Forward, Reverse, Stop, 
                 LeftSpin, RightSpin, LeftForward, RightForward, LeftReverse, RightReverse,
                 Record, Replay1, Replay2, Replay3, Replay4};

void setup()                                 // Built-in initialization block
{ 
  servoRight.attach(12);
  servoLeft.attach(11);

  irrecv.enableIRIn();
  irrecv.blink13(true);
  pinMode(6,OUTPUT);
  digitalWrite(6,LOW);  

  Serial.begin(9600);
  Serial.println("Project: IR remote-controlled robot car");

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr for the 128x64 OLED
  display.display(); // show splashscreen
  delay(2000);
  display.clearDisplay();   // clears the screen and 

}

void loop(){
  decode_results results;
  enum ir_command ir_key;
  //char *banner = "ROBOT CAR";
  byte bx = display.width()/2;
  byte by = display.height()/2;
  if (irrecv.decode(&results)){    
        switch (results.decode_type){
            case NEC: Serial.print("NEC"); break ;
            case SONY: Serial.print("SONY"); break ;
            case RC5: Serial.print("RC5"); break ;
            case RC6: Serial.print("RC6"); break ;
            case DISH: Serial.print("DISH"); break ;
            case SHARP: Serial.print("SHARP"); break ;
            case JVC: Serial.print("JVC"); break ;
            case SANYO: Serial.print("SANYO"); break ;
            case MITSUBISHI: Serial.print("MITSUBISHI"); break ;
            case SAMSUNG: Serial.print("SAMSUNG"); break ;
            case LG: Serial.print("LG"); break ;
            case WHYNTER: Serial.print("WHYNTER"); break ;
            case AIWA_RC_T501: Serial.print("AIWA_RC_T501"); break ;
            case PANASONIC: Serial.print("PANASONIC"); break ;
            case DENON: Serial.print("DENON"); break ;
          default:
            case UNKNOWN: Serial.print("UNKNOWN"); break ;
          }
        Serial.print(": ");
        if (results.decode_type == NEC && results.value == REPEAT) {
          Serial.println("repeat");
        } else {
          Serial.println(results.value, HEX);
        }
        irrecv.resume();
        switch (results.value) {
          case 0xFFAA55: ir_key = Forward; by = 10; break;
          case 0xFF629D: ir_key = Reverse; by = display.height()-10; break;
          case 0xFF6A95: ir_key = Stop; break;
          case 0xFF5AA5: ir_key = LeftSpin; bx = 10; break;
          case 0xFF4AB5: ir_key = RightSpin; bx = display.width()-10; break;
          case 0xFF9A65: ir_key = LeftForward; bx = 10; by = 10; break;
          case 0xFF42BD: ir_key = RightForward; bx = display.width()-10; by = 10; break;
          case 0xFF52AD: ir_key = LeftReverse; bx = 10; by = display.height()-10; break;
          case 0xFF8A75: ir_key = RightReverse; bx = display.width()-10; by = display.height()-10; break;
          case 0xA1B79867: ir_key = Record; break;
          case 0xFF12ED: ir_key = Replay1; break;
          case 0xFF32CD: ir_key = Replay2; break;
          case 0xFF22DD: ir_key = Replay3; break;
          case 0xFF02FD: ir_key = Replay4; break;
          default: ir_key = Undefined; break;
        }
        if (ir_key != Undefined) {
          // blink led
          digitalWrite(6,HIGH);
          delay(10);
          digitalWrite(6,LOW);
#if 0
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(30,20);
  int len=strlen(banner);
  for (int idx=0; idx<len; idx++) {
    display.write(banner[idx]);
  }
  //display.print(banner);
  display.display();
  display.clearDisplay();
#else
  // draw a white circle, 10 pixel radius
  display.fillCircle(bx, by, 10, WHITE);
  display.display();
  display.clearDisplay();
 #endif
          ir_action(ir_key);
        }
  }
}

void ir_action(enum ir_command ir_key)
{
  // speedLeft, speedRight ranges: Backward  Linear  Stop  Linear   Forward
  //                               -200      -100......0......100       200
  if (ir_key == Forward) {
    servoLeft.writeMicroseconds(1700);
    servoRight.writeMicroseconds(1300);
  } else if (ir_key == Reverse) {
    servoLeft.writeMicroseconds(1300);
    servoRight.writeMicroseconds(1700);
  } else if (ir_key == Stop) {
    servoLeft.writeMicroseconds(1500);
    servoRight.writeMicroseconds(1500);
  } else if (ir_key == LeftSpin) {
    servoLeft.writeMicroseconds(1300);
    servoRight.writeMicroseconds(1300);
  } else if (ir_key == RightSpin) {
    servoLeft.writeMicroseconds(1700);
    servoRight.writeMicroseconds(1700);
  } else if (ir_key == LeftForward) {
    servoLeft.writeMicroseconds(1550);
    servoRight.writeMicroseconds(1300);
  } else if (ir_key == RightForward) {
    servoLeft.writeMicroseconds(1700);
    servoRight.writeMicroseconds(1450);
  } else if (ir_key == LeftReverse) {
    servoLeft.writeMicroseconds(1450);
    servoRight.writeMicroseconds(1700);
  } else if (ir_key == RightReverse) {
    servoLeft.writeMicroseconds(1300);
    servoRight.writeMicroseconds(1550);
  }
}

