#include <Encoder.h>

#include <NewPing.h>

#include <Wire.h>

#define TRIGGER_PIN 12
#define ECHO_PIN 11
#define MAX_DISTANCE 200  // maximum distance for the sensor (in centimeters)
#define ENCA 2 
#define ENCB 3 
#define PWM 9
#define IN1 8
#define IN2 7

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
long oldPosition = 0;
long newPosition = 0;
double angle = 0;


void setup() {
  Serial.begin(9600);
  pinMode(9,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(ENCA,INPUT_PULLUP);
  pinMode(ENCB,INPUT_PULLUP);
//  attachInterrupt(0,readEncoder,RISING);
  //attachInterrupt(1,read_Encoder,RISING);
  Serial.println("target pos");
}

void loop() {

  // set target position
  int target = 0;
  //target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 180;
  float kd = 0.1;
  float ki = 0.025;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // error
  int e = pos-target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1);

  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    
  }
  else{
    digitalWrite(in1,LOW);
    
  }  
}



//void read_Encoder() {
  // Get the current position of the encoder
  //int encoderPosition = myEncoder.read();

  // Convert the encoder position to degrees
  //float angle = (float)encoderPosition / 400.0 * 360.0;

  // Print the angle to the serial monitor
  //Serial.println(angle);

  // Wait for a short period of time before reading the encoder again
  //delay(100);
//}

Encoder myEncoder(2, 3);  // create an instance of the Encoder class, using pins 2 and 3
void readEncoder(){
  newPosition = myEncoder.read();  // read the encoder's position
  angle = (newPosition - oldPosition) * 360.0 / 500.0;  // calculate the change in angle
  oldPosition = newPosition;  // store the new position as the old position
  Serial.println(angle);
  if (angle < -1) {
    // Set direction to forward
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    // Set PWM value to control motor speed
    analogWrite(9, 128);
  } else if (angle > 1) {
    // Set direction to reverse
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    // Set PWM value to control motor speed
    analogWrite(9, 128);
  } else {
    // Stop motor
    analogWrite(9, 0);
  }
    // print the angle to the serial monitor
  delay(100);  // wait for 100 milliseconds  
}
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
void sensor() {
  unsigned int uS = sonar.ping();  // ping the sensor and get the duration of the echo pulse
  double distance = uS / US_ROUNDTRIP_CM;  // convert the duration to distance (in centimeters)
  Serial.println(distance);  // print the distance to the serial monitor
  delay(100);  // wait for 100 milliseconds
}
