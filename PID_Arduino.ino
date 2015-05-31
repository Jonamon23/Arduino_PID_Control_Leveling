#include <Wire.h> //Needed for ADXL345 and PID
#include <PID_v1.h>
#include <ADXL345.h> //Accelerometer sensor

ADXL345 acc;  //define accelerometer for reading (see library)

double X_i, Y_i, Z_i, X, Y, Z;
double fXg2 = 0, fXg5 = 0;
double fYg11 = 0, fYg12 = 0; //functions of XYZ for roll and pitch calcs

int max_speed = 254;
int min_speed = 100;
int duty_cycle = 50;

float vel = duty_cycle * 5 ; // same as DC/100 * 500  
// Max Value = 500

int vel_F_L = 60 , vel_F_R = 60 , vel_B_L = 60 , vel_B_R = 60;

// PID STUFF
double delta_F_L, delta_B_R, delta_F_R, delta_B_L;
double conversion_value = 10;

double Kp = 1;
double Ki = 0.2;
double Kd = 0.2;
double Setpoint = 0;

//PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID myPID2(&fXg2, &delta_B_R, &Setpoint, Kp, Ki, Kd, DIRECT );
PID myPID5(&fXg5, &delta_F_L, &Setpoint, Kp, Ki, Kd, DIRECT );
PID myPID11(&fYg11, &delta_F_R, &Setpoint, Kp, Ki, Kd, DIRECT );
PID myPID12(&fYg12, &delta_B_L, &Setpoint, Kp, Ki, Kd, DIRECT );


void setup() {
  DDRB |= _BV(PB5); //Maybe not needed?
  DDRB |= _BV(PB6);
  
  TCCR1A=0; //Clear Timer/Counter Control Registers A&B
  TCCR1B=0;
  TCCR3A=0;
  TCCR3B=0;
  
  TCCR1A |= _BV(WGM11); // TCCR1 controls pins 11 and 12
  TCCR1B |= _BV(WGM13)|_BV(CS10);

  TCCR3A |= _BV(WGM31); // TCCR3 controls pins 2 and 5
  TCCR3B |= _BV(WGM33)|_BV(CS30);  
  
  ICR1=500; // Set PWM freq to 16 KHz -> same as original
  ICR3=500; 

  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  OCR3B = vel; // pin 2  -> brown  -> Back Right
  OCR3A = vel; // pin 5  -> blue   -> Front Left
  OCR1A = vel; // pin 11 -> green  -> Front Right
  OCR1B = vel; // pin 12 -> orange -> Back Left

  acc.begin();  //start using accelerometer
  //Serial.begin(38400);  //set baud rate for debugging
 
  //turn the PID on
  myPID2.SetMode(AUTOMATIC); 
  myPID5.SetMode(AUTOMATIC); 
  myPID11.SetMode(AUTOMATIC); 
  myPID12.SetMode(AUTOMATIC); 

  acc.read(&X_i, &Y_i, &Z_i); //Calibrate sensor for initial ground position  
}

void loop() {

  acc.read(&X, &Y, &Z);

  fXg2 = X-X_i;// * ALPHA + (fXg * (1.0 - ALPHA)); //simple filter on measurements
  fXg5 = -(X-X_i);
  fYg11 = -(Y-Y_i);// * ALPHA + (fYg * (1.0 - ALPHA));
  fYg12 = Y-Y_i;
  
  myPID2.Compute();
  myPID5.Compute();
  myPID11.Compute();
  myPID12.Compute();

  vel_B_R = (conversion_value * delta_B_R) + 150;
  vel_F_L = (conversion_value * delta_F_L) + 150;
  vel_F_R = (conversion_value * delta_F_R) + 150;
  vel_B_L = (conversion_value * delta_B_L) + 150;

  analogWrite(2 , vel_B_R); // pin 2  -> brown  -> Back Right
  analogWrite(5 , vel_F_L); // pin 5  -> blue   -> Front Left
  analogWrite(11, vel_F_R); // pin 11 -> green  -> Front Right
  analogWrite(12, vel_B_L); // pin 12 -> orange -> Back Left
}
