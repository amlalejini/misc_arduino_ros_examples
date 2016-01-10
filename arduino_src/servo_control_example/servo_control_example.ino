#include <Servo.h>

Servo servo_0;

int MAX_SERVO = 150;
int MIN_SERVO = 5;

int pos = 0;
int INIT_POS = 90;
int servo_pin = 9;


String packet = "";
boolean packet_complete = false;

int limit_value(int min_val, int max_val, int value) {
   // Returns value capped at min and max values 
   if (value > max_val)
     return max_val;
   else if (value < min_val)
     return min_val;
   else
     return value;
}

void setup() {
  Serial.begin(9600);
  servo_0.attach(servo_pin);
  servo_0.write(INIT_POS);
  packet.reserve(200);
  
}

void loop() {
  // print the string when a newline arrives:
  if (packet_complete) {
    // clear the string:
    if (packet.length() >= 3) {
      // Check for delimiter
      if (packet[1] == ':') {
        // Grab label
        char label = packet[0];
        // Grab value
        int value = packet.substring(2).toInt();
        switch (label) {
          case '0':
            servo_0.write(limit_value(MIN_SERVO, MAX_SERVO, value));
            break;
          default:
            break; 
        }
//        Serial.println("=======");
//        Serial.println(label);
//        Serial.println(value);
      }
       
    } else {
      Serial.println("Malformed packet");
    }
    packet = "";
    packet_complete = false;
  }

}

void serialEventRun(void) {
  if (Serial.available()) serialEvent();
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      packet_complete = true;
    } else {
      packet += inChar;
    }
  }
}
