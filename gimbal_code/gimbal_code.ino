/*
 * Davis Drone Club Gimbal v1.4 firmware
 * 
 * Serial Connection Branch
 * This is untested software
 * 
 * CHANGELOG
 * - Reverted bauderate to 9600
 * - Added responses for tuning via serial monitor
 * 
 * PID_v1 library can be downloaded from here: https://github.com/br3ttb/Arduino-PID-Library
 * Based on code from http://www.brokking.net/imu.html
 * 
 * Kalman filter library can be downloaded here: https://github.com/TKJElectronics/KalmanFilter
 */

#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

//NUMBER OF POINTS TO AVERAGE IN OUTPUT AVERAGING
#define NUM_AVG 5

//PID TUNING VALUES- TUNE THE PID LOOP HERE
double kp = 0.5;
double ki = 4;
double kd = 0.0015;

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

String inString;                //Stores converted input
String parse_head, parse_data;  //Splits input into head and data
int delimIndex;                 //Index of delimiter within string
String tuneP = "TUNEP";         //Header content for tuning packet
String tuneI = "TUNEI";         //Header content for tuning packet
String tuneD = "TUNED";         //Header content for tuning packet
String spStr = "SETSP";
String dbStr = "SETDB";


double deadband = 2.0;

//Declaring servo object
Servo servo;

//Declare PID values
double input, setpoint, output;

PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
//Variables for data output
String warn       = "WARN;";
String dataString = "DATA;";
String dataDelim  = ",";

void setup() {
  Wire.begin();             //Initialize I2C communication
  Serial.begin(115200);      //Initialize serial communication at 9600

  pinMode(13, OUTPUT);      //Set pinmode of LED pin to output
  
  servo.attach(3);          //Declare servo output pin
  servo.write(90);          //Initialize to 70
  delay(1000);

  init_mpu();               //Initialize MPU6050 registers

  digitalWrite(13, HIGH);   //Turn on LED

  for(int i = 0; i < NUM_AVG - 1; i++){     //Initialize the output average array
    outputAvg[i] = 90;
  }
  
  cal_mpu();                //Calibrate gyroscope
  setpoint = 0;
  output = 90;
  myPID.SetOutputLimits(50,130);
  myPID.SetMode(AUTOMATIC);
  myPID.SetControllerDirection(REVERSE);  //DIRECT for normal mounting, REVERSE for inverted

  digitalWrite(13, LOW); 
  loop_timer = micros();    //Initialize loop timer
}

void loop() {
  read_mpu();                   //Read data
  process_mpu();                //Process data
  input = angle_pitch_output;   //Input into PID loop

  calc_avg();                   //Output averaging
  
  if(abs(input-setpoint) < deadband)
  {
    input = setpoint;
  }
  else
  {
    servo.write((int)(output));   //Write output to servo
  }   
  myPID.Compute();              //Compute output
  avgOut = 0;
  printData(angle_pitch_output, millis());                //Print values to serial

  if(Serial.available()>0){
    inString = "";
    while(Serial.available()>0){
      inString+= char(Serial.read());
    }

    delimIndex = inString.indexOf(';');
    parse_head = inString.substring(0,delimIndex);
    parse_data = inString.substring(delimIndex + 1);

    if(parse_head == tuneP)
    {
      processTune(parse_data, 'P');
    }
    else if(parse_head == tuneI)
    {
      processTune(parse_data, 'I');
    }
    else if(parse_head == tuneD)
    {
      processTune(parse_data, 'D');
    }
    else if(parse_head == spStr)
    {
      setpoint = parse_data.toDouble(); 
    }
    else if(parse_head == dbStr)
    {
      deadband = parse_data.toDouble();
    }
    else
    {
      Serial.println(warn + parse_head);
    }
  }
  if(micros()-loop_timer > 4000)
  {
    Serial.println(micros()-loop_timer);
  }
  while(micros() - loop_timer < 4000);  //Constrain each loop to 4000us long for 250Hz refresh rate
  loop_timer = micros();                //Update the loop timer
}

void printData(float angle, unsigned long currTime){
  String printStr = dataString + angle + dataDelim + currTime;
  Serial.println(printStr);
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
}

void processTune(String data, char mode){
  if(mode == 'P')
  {
    kp = data.toDouble();
    myPID.SetTunings(kp, ki, kd);
  }
  else if(mode == 'I')
  {
    ki = data.toDouble();
    myPID.SetTunings(kp, ki, kd);
  }
  else if(mode == 'D')
  {
    kd = data.toDouble();
    myPID.SetTunings(kp, ki, kd);
  }
  Serial.println("TUNING UPDATE RECEIVED");
}

