/**
 * MPU6050 INTERFACE V1.0
 * PROVIDES INITIALIZATION, READING, CALLIBRATION AND PROCESSING FUNCTIONALITY
 */

//Adjustable parameters
#define IMU_ADDR 0x68
#define NUM_CAL  500

void init_mpu(){                    //Sets registers to initialize MPU6050
  Wire.beginTransmission(IMU_ADDR); //Initialize I2C communication
  Wire.write(0x6B);                 //Address 6B
  Wire.write(0x00);                 //Write 00 to register 6B (Disable low power mode)
  Wire.endTransmission();           //End transmission

  Wire.beginTransmission(IMU_ADDR); //Initialize I2C communication
  Wire.write(0x1C);                 //Address 1C
  Wire.write(0x10);                 //Write 10 to register 1C (Set accelerometer to +/- 8g scale)
  Wire.endTransmission();           //End transmission

  Wire.beginTransmission(IMU_ADDR); //Initialize I2C communication
  Wire.write(0x1B);                 //Address 1B
  Wire.write(0x08);                 //Write 08 to register 1B (Set gyroscope to 500dps scale)
  Wire.endTransmission();           //End transmission
}

void read_mpu(){                        //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(IMU_ADDR);         //Start communicating with the MPU-6050
  Wire.write(0x3B);                     //Send the requested starting register
  Wire.endTransmission();               //End the transmission
  Wire.requestFrom(IMU_ADDR,14);            //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);         //Wait until all the bytes are received

  acc_x  = Wire.read()<<8|Wire.read();  //Add the low and high byte to the acc_x variable
  acc_y  = Wire.read()<<8|Wire.read();  //Add the low and high byte to the acc_y variable
  acc_z  = Wire.read()<<8|Wire.read();  //Add the low and high byte to the acc_z variable
  temp   = Wire.read()<<8|Wire.read();  //Add the low and high byte to the temperature variable
  gyro_x = Wire.read()<<8|Wire.read();  //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();  //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();  //Add the low and high byte to the gyro_z variable
}

void cal_mpu(){                                         //Subroutine for callibrating MPU6050
  for (int cal_int = 0; cal_int < NUM_CAL ; cal_int ++){   //Iterate 2000 times
    //if(cal_int % 125 == 0)Serial.print(".");            //Print a dot every 125 readings
    read_mpu();                                         //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gyro_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                           //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_cal /= NUM_CAL;                                   //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= NUM_CAL;                                   //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= NUM_CAL;                                   //Divide the gyro_z_cal variable by 2000 to get the avarage offset
}

void process_mpu(){
  //Compensate for calibration offset
  gyro_x -= gyro_x_cal;  
  gyro_y -= gyro_y_cal;  
  gyro_z -= gyro_z_cal;  

  //"Integrate" gyroscope reading to get current angle
  // Multiply each reading by 65.5 to convert to deg/second, Multiply by 1/250Hz to get seconds
  //0.0000611 = 65.5/250Hz
  //sine to transfer roll to pitch if yaw is present

  
  angle_pitch += gyro_x * 0.0000611;
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  
  angle_roll += gyro_y * 0.0000611;
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

  //Acceleromter angle reading
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));   //Magnitude of sum vector
  angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;        //Use trigonometry to find angles
  angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;

  //Combine gyroscope and accelerometer readings to correct for drift
  if(set_gyro_angles){
    angle_pitch = angle_pitch * 0.9 + angle_pitch_acc * 0.1; 
    angle_roll = angle_roll * 0.9 + angle_roll_acc * 0.1;     
  }
  else{   //Initialize values if first loop
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }

  //Complementary filter
  angle_pitch_output = angle_pitch_output * 0.8 + angle_pitch * 0.2;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output  = angle_roll_output  * 0.8 + angle_roll  * 0.2;   //Take 90% of the output roll value and add 10% of the raw roll value

}

