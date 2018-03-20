/*
 * Davis Drone Club Gimbal v1.3 firmware
 * 
 * CHANGELOG
 * - Changed PID tuning values from variables to definitions
 * 
 * PID_v1 library can be downloaded from here: https://github.com/br3ttb/Arduino-PID-Library
 * Based on code from http://www.brokking.net/imu.html
 */

#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

//NUMBER OF POINTS TO AVERAGE IN OUTPUT AVERAGING
#define NUM_AVG 3

//PID TUNING VALUES- TUNE THE PID LOOP HERE
#define KP 0.015
#define KI 6.000
#define KD 0.020

//Declaring  global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temp;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

float outputAvg[NUM_AVG];
float avgOut;

//Declaring servo object
Servo servo;

//Declare PID values
double input, setpoint, output;

PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);


void setup() {
  Wire.begin();             //Initialize I2C communication
  Serial.begin(57600);      //Initialize serial communication at 115200

  pinMode(13, OUTPUT);      //Set pinmode of LED pin to output
  
  servo.attach(3);          //Declare servo output pin
  servo.write(70);          //Initialize to 70
  delay(1000);

  init_mpu();               //Initialize MPU6050 registers

  digitalWrite(13, HIGH);   //Turn on LED

  for(int i = 0; i < NUM_AVG - 1; i++){     //Initialize the output average array
    outputAvg[i] = 70;
  }
  
  cal_mpu();                //Calibrate gyroscope
  setpoint = 0;
  output = 90;
  myPID.SetOutputLimits(35,105);
  myPID.SetMode(AUTOMATIC);
  digitalWrite(13, LOW); 
  loop_timer = micros();    //Initialize loop timer
}

void loop() {
  read_mpu();                   //Read data
  process_mpu();                //Process data
  input = angle_pitch_output;   //Input into PID loop
  myPID.Compute();              //Compute output

  calc_avg();                   //Output averaging
   
  servo.write((int)(avgOut));   //Write output to servo
  avgOut = 0;
  printValues();                //Print values to serial

  while(micros() - loop_timer < 4000);  //Constrain each loop to 4000us long for 250Hz refresh rate
  loop_timer = micros();                //Update the loop timer
}

void printValues(){
  Serial.print(angle_pitch_output);
  Serial.print("-");
  Serial.println(angle_roll_output);
}

void calc_avg(){
  for(int i = 0; i < NUM_AVG - 1; i++){   //Shift element arrays down 1 index
    outputAvg[i] = outputAvg[i+1];  
  }
  outputAvg[NUM_AVG-1] = output;          //Update last element of array

  for(int i = 0; i <= (NUM_AVG-1); i++){  //Sum all elements of array
    avgOut += outputAvg[i];
  }
  avgOut = avgOut/NUM_AVG;                //Average by dividing number of elements
  Serial.print((int) avgOut);
  Serial.print("-");
}

