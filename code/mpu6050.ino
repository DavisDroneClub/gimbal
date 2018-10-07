/*
 * ----------------------------------------------------------------------
 * MPU6050 INTERFACE V1.0
 * PROVIDES INITIALIZATION, READING, CALLIBRATION AND PROCESSING FUNCTIONALITY
 * 
 * Based on code from http://www.brokking.net/imu.html
 * ----------------------------------------------------------------------
 */

//Adjustable parameters
#define IMU_ADDR 0x68
#define NUM_CAL  500

void init_mpu(){           //Sets registers to initialize MPU6050
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

void read_mpu(){           //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(IMU_ADDR);         //Start communicating with the MPU-6050
  Wire.write(0x3B);                     //Send the requested starting register
  Wire.endTransmission();               //End the transmission
  Wire.requestFrom(IMU_ADDR,14);            //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);         //Wait until all the bytes are received
  for(int i = 0; i < 3; i++) {
    acc[i]  = (Wire.read()<<8|Wire.read()) * (0.000244141) * ( 9.80665);  //Add the low and high byte to the acc_x variable
  }
  temp   = Wire.read()<<8|Wire.read();   //Add the low and high byte to the temperature variable
  for(int i = 0; i < 3; i++) {
    gyro[i]  = (Wire.read()<<8|Wire.read()) * (0.007629395) * DEG_TO_RAD;  //Add the low and high byte to the acc_x variable
  }
}
void cal_mpu(){            //Subroutine for callibrating MPU6050
  for (int i = 0; i < 3; i++) {
    gyro_cal[i] = 0;
  }

  acc_cal = 0;

  for (int cal_int = 0; cal_int < NUM_CAL ; cal_int ++){  //Iterate 2000 times
    //if(cal_int % 125 == 0)Serial.print(".");            //Print a dot every 125 readings
    read_mpu();                                           //Read the raw acc and gyro data from the MPU-6050
    for (int i = 0; i < 3; i++) {
      gyro_cal[i] += gyro[i];
    }
    acc_cal += sqrt(pow(acc[0],2)+pow(acc[1],2)+pow(acc[2],2));
    delay(3);                                             //Delay 3us to simulate the 250Hz program loop
  }
  for (int i = 0; i < 3; i++) {
    gyro_cal[i] /= NUM_CAL;
  }
  acc_cal /= NUM_CAL;

  gyro_x_cal.val_float = gyro_cal[0];
  gyro_y_cal.val_float = gyro_cal[1];
  gyro_z_cal.val_float = gyro_cal[2];
  writeVals();
}

void process_mpu(){
  /*
    Normalize gyro measurements into pure quaternion
  */
  gyro_raw_q[0] = 0;
  gyro_raw_q[1] = -0.5*gyro[0];
  gyro_raw_q[2] = -0.5*gyro[1];
  gyro_raw_q[3] = -0.5*gyro[2];

  /*
    Calculate adapative gain
  */
  err_mag = (abs(vect_magnitude(acc))-acc_cal)/acc_cal;
  alpha = BASE_GAIN*err_gain_factor(err_mag, THRESH_LOW, THRESH_HIGH);

  //Calculate global rate from local rate
  multiply(gyro_raw_q, prev_q, rate_gyro_global);

  //Integrate global rate
  for (int i = 0; i < 4; i++) {
      gyro_q[i] = prev_q[i] + gyro_raw_q[i] * INTERVAL;
      //gyro_q_norm[i] = gyro_q[i];
  }
  
  //Step 3
  quat_to_inverse_matrix(gyro_q, gyro_R);

  //Step 4
  vect_normalize(acc);
  matrix_multiply(gyro_R, acc, g_predict);
  vect_normalize(g_predict);

  //Step 5
  d_q_acc[0] = sqrt((g_predict[2]+1)/2);
  d_q_acc[1] = -g_predict[1]/(sqrt(2*(g_predict[2]+1)));
  d_q_acc[2] = g_predict[0]/(sqrt(2*(g_predict[2]+1)));
  d_q_acc[3] = 0;

  //Step 6
  if(d_q_acc[0]>EPSILON) {
    //LEPR
    for(int i = 0; i < 4; i++) {
      d_acc_interp[i] = (1-alpha)*q_i[i]+alpha*d_q_acc[i];
    }
    quat_normalize(d_acc_interp);
  }
  else {
    slerp(alpha, d_q_acc, d_acc_interp);
  }

  //Step 7
  multiply(gyro_q, d_acc_interp, q_out);

  for(int i = 0; i < 4; i++) {
    prev_q[i] = q_out[i];
  }

  yaw = atan2(-2*q_out[1]*q_out[2]+2*q_out[0]*q_out[3],
                 pow(q_out[0],2) + pow(q_out[1],2) -
                 pow(q_out[2],2) - pow(q_out[3],2))*RAD_TO_DEG;

  pitch = asin(2*q_out[1]*q_out[3]+2*q_out[0]*q_out[2])*RAD_TO_DEG;
  roll  = atan2(-2*q_out[2]*q_out[3]+2*q_out[0]*q_out[1],
                pow(q_out[3],2) - pow(q_out[2],2) -
                pow(q_out[1],2) + pow(q_out[0],2))*RAD_TO_DEG;
}

