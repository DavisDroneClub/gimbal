/*
 * ----------------------------------------------------------------------
 * Davis Drone Club Gimbal v2.0 firmware
 * 
 * CHANGELOG
 * - Added EEPROM functionality
 * - Reordered code into functions
 * - Added "functions", "eeprom" and "communication" tabs
 * 
 * PID_v1 library can be downloaded from here: https://github.com/br3ttb/Arduino-PID-Library
 * 
 * --------------------ADJUSTABLE SETTINGS--------------------
 */
//PID CONTROLLER DIRECTION: 0 FOR INVERTED, 1 FOR REGULAR
boolean gimbalDir = 1;

//PRINT DATA TO SERIAL: true FOR ENABLED, false FOR DISABLE 
boolean print_en = false;

/*
 * --------------------VARIABLE DECLARATIONS------------------
 */
//Include libraries
#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include <EEPROM.h>
 
//Declare a union datatype to allow easy conversion between float and byte
//More information on union: https://www.tutorialspoint.com/cprogramming/c_unions.htm
union EEPOMvals {               //Define variables stored in EEPROM
  float  val_float;
  byte   val_byte[4];
} kp, ki, kd, deadband;

//Declaring global variables for IMU and angle calculations
int gyro_x, gyro_y, gyro_z;
double acc_x, acc_y, acc_z, acc_total_vector;
int temp;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;
double loop_timer;
int  loop_duration;
float angle_pitch, angle_roll;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

//Declaring strings for Serial communication
String inString;                //Stores converted input
String parse_head, parse_data;  //Splits input into head and data
int delimIndex;                 //Index of delimiter within string
String tuneP = "TUNEP";         //Header content for tuning packet
String tuneI = "TUNEI";         //Header content for tuning packet
String tuneD = "TUNED";         //Header content for tuning packet
String spStr = "SETSP";         //Header content for setpoint packet
String dbStr = "SETDB";         //Header content for deadband packet
String drStr = "SETDR";         //Header content for direction packet
String ptStr = "SETPT";         //Header content for print packet
String peStr = "GETEP";         //Header content to get EEPROM data
String warnString = "WARN";     //Warning header
String loopString = "LOOP";     //Loop warning header
String dataString = "DATA";     //Data header
String eeprString = "EEPR";     //EEPROM header
String headDelim  = ";";        //Header delimiter
String dataDelim  = ",";        //Data delimiter

//Declaring servo object
Servo servo;

//Declare PID values
double input, setpoint, output;
PID myPID(&input, &output, &setpoint, kp.val_float, ki.val_float, kd.val_float, DIRECT);


/*
 * --------------------SETUP FUNCTION--------------------
 */
void setup() {
  readVals();                     //Read PID values from EEPROM
  myPID.SetTunings(kp.val_float, ki.val_float, kd.val_float);
  init_comm();                    //Initialize I2C and Serial communication
  pinMode(13, OUTPUT);            //Set pinmode of LED pin to output
  init_servo();                   //Initialize servo motor
  delay(1000);                    //Delay to allow gimbal to settle
  init_mpu();                     //Initialize MPU6050
  digitalWrite(13, HIGH);         //Turn on LED
  cal_mpu();                      //Calibrate gyroscope
  setpoint = 0;                   //Set PID setpoint and initial output
  output = 90;                    
  myPID.SetOutputLimits(50,130);  //Set output limits of PID controller
  myPID.SetMode(AUTOMATIC);       //Activate PID controller by setting to AUTOMATIC
  myPID.SetControllerDirection(gimbalDir);  //Set mounting direction

  digitalWrite(13, LOW);          //Turn off LED after initialization
  loop_timer = micros();          //Initialize loop timer
}

/*
 * --------------------LOOP FUNCTION---------------------
*/
void loop() {
  read_mpu();                     //Read data
  process_mpu();                  //Process data
  input = angle_pitch_output;     //Input into PID loop
  actuate();                      //Actuate gimbal and deadband

  myPID.Compute();                //Compute output

  //Print data to serial if PRINT is enabled
  if(print_en) printData(angle_pitch_output, millis());

  checkSerial();                  //Check for serial commands
  enforceLoop();                  //Enforce loop timer
}
