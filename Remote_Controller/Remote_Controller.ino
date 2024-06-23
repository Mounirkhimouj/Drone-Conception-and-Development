//#include <SoftwareSerial.h>

//SoftwareSerial Serial1(8, 7); // RX, TX

// Drone Control with Two Joysticks and Two Push Buttons

// Pins for joystick axes and buttons
const int yaw_pin = A0;        // Yaw joystick LH
const int pitch_pin = A2;      // Pitch joystick RV
const int roll_pin = A1;       // Roll joystick RH
const int altitude_pin = A3;   // Altitude joystick LV
const int button1_pin = 2;     // Button 1
const int button2_pin = 4;     // Button 2


char data[52];

// Variables to store joystick and button values
int yaw_val, pitch_val;
int roll_val, altitude_val;
int button1_val, button2_val, switch_val;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  //Serial1.begin(9600);

  // Set button pins as input with pull-up resistors
  pinMode(button1_pin, INPUT_PULLUP);
  pinMode(button2_pin, INPUT_PULLUP);
//  pinMode(switch_pin, INPUT_PULLUP);
}

void loop() {
  // Read joystick values
  yaw_val = analogRead(yaw_pin);
  pitch_val = analogRead(pitch_pin);
  roll_val = analogRead(roll_pin);
  altitude_val = analogRead(altitude_pin);

  // Read button states
  button1_val = !(digitalRead(button1_pin));
  button2_val = !(digitalRead(button2_pin));
//  switch_val = !(digitalRead(switch_pin));

  // Format the data string with leading zeros
  // 0,1,2,3,4,5,6,7
  // DEVICEiD,NAN,yaw_val,pitch_val,roll_val,altitude_val,button1_val,button2_val
  if(button2_val == LOW){
    sprintf(data,"#,0000,0000,%04d,%04d,%04d,%04d,%04d,%04d,0000,0000,", yaw_val, pitch_val, roll_val, altitude_val, button1_val, button2_val);
  }
  else{
    sprintf(data,"#,0000,0000,0000,0000,0000,0000,0000,0000,0000,0000,");
  }

  // Send the formatted data string over serial
  Serial.print(data);

  delay(200);
}
