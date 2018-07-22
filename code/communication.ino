/*
 * ----------------------------------------------------------------------
 * COMMUNICATION FILE
 * PROVIDES FUNCTIONS TO FACILITATE COMMUNICATION
 * ----------------------------------------------------------------------
 */

/*
 * Initialize communication channels 
 * To be run once at program start
 */
void init_comm() {
  Wire.begin();              //Initialize I2C communication
  Serial.begin(115200);      //Initialize serial communication at 9600
}

/*
 * Perform actions based on header
 * To add new actions:
 * 1) Add new else-if block and place functions to perform within block
 * 2) Add new header name to "code" tab
 */
void serialHandler(String parse_head, String parse_data){
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
      setpoint = parse_data.toFloat(); 
    }
    else if(parse_head == dbStr)
    {
      deadband.val_float = parse_data.toFloat();
    }
    else if(parse_head == drStr)
    {
      gimbalDir = parse_data.toInt();
      myPID.SetControllerDirection(gimbalDir);
    }
    else if(parse_head == enStr)
    {
      int gimbalEn = parse_data.toInt();
      if(gimbalEn == 1)
      {
        myPID.SetMode(AUTOMATIC);
      }
      else if(gimbalEn == 0)
      {
        myPID.SetMode(MANUAL);
        output=90;
      }
    }
    else if(parse_head == cbStr)
    {
      delay(1000);
      cal_mpu();
      delay(500);
    }
    else if(parse_head == poStr)
    {
      int pos = parse_data.toInt();
      if(pos>HIGH_BOUND)
      {
        output = HIGH_BOUND;
      }
      else if(pos < LOW_BOUND)
      {
        output = LOW_BOUND;
      }
      else
      {
        output = pos;
      }
    }
    else if(parse_head == ptStr)
    {
      print_en = parse_data.toInt();
    }
    else if(parse_head == peStr)
    {
      printEEPROM();
    }
    else if(parse_head == veStr)
    {
      printVer();
    }
    else if(parse_head == dnStr)
    {
      Serial.println(devnString + headDelim + "ddc-gimbal");
    }
    else
    {
      Serial.println(warnString + headDelim + parse_head);
    }
}

/*
 * Print data for checking gimbal performance
 * Pass angle and current time
 */
void printData(float angle, unsigned long currTime){
  //Combine data into string
  String printStr = dataString + headDelim + angle + dataDelim + loop_duration + dataDelim + output + dataDelim + currTime;
  Serial.println(printStr);
}

/*
 * Non-blocking implementation of reading lines from serial
 */
int readline(int input_char, char *buffer, int buffer_length){
  static int curr_pos = 0;
  int final_pos;

  if (input_char>0) {
    switch(input_char) {
      case '\r':
        break;
      case '\n':
        final_pos = curr_pos;
        curr_pos = 0;
        return final_pos;
      default:
        if(curr_pos<buffer_length-2) {
          buffer[curr_pos++] = input_char;
          buffer[curr_pos] = '\0';
        }
    }
  }
  return 0;
}

/*
 * Read character from serial and perform action if line completed
 */
void checkSerial(){
  String inString, head, data;
  int headIndex;
  
  if(readline(Serial.read(), input_buf, 80) > 0) {
    inString = String(input_buf);
    headIndex = inString.indexOf(";");
    head = inString.substring(0, headIndex);
    data = inString.substring(headIndex+1);
    head.trim();
    data.trim();
    serialHandler(head, data);
  }  
}

/*
 * Print EEPROM values to Serial
 */
void printEEPROM() {
  String printStr = eeprString + headDelim + String(kp.val_float,4) 
    + dataDelim + String(ki.val_float,4) + dataDelim + String(kd.val_float,4) 
    + dataDelim + deadband.val_float + dataDelim + gyro_x_cal.val_float
    + dataDelim + gyro_y_cal.val_float + dataDelim + gyro_z_cal.val_float;
  Serial.println(printStr);
}

/**
 * Print gimbal firmware version to Serial
 */
void printVer() {
  String printStr = versString + headDelim + String(VERSION[0]) + dataDelim + String(VERSION[1]);
  Serial.println(printStr);
}

