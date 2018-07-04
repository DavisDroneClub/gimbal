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
 * Print data for checking gimbal performance
 * Pass angle and current time
 */
void printData(float angle, unsigned long currTime){
  //Combine data into string
  String printStr = dataString + headDelim + angle + dataDelim + loop_duration + dataDelim + currTime;
  Serial.println(printStr);
}

/*
 * Check if serial data is available and perform actions
 */
void checkSerial() {
  if(Serial.available()>0){             //If Serial data is available
    inString = "";                      //Initialize inString variable
    while(Serial.available()>0){
      inString+= char(Serial.read());   //Read data byte by byte
    }

    //Split string into header and data by ; delimiter
    delimIndex = inString.indexOf(headDelim); 
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
      deadband.val_float = parse_data.toDouble();
    }
    else if(parse_head == drStr)
    {
      gimbalDir = parse_data.toInt();
      myPID.SetControllerDirection(gimbalDir);
    }
    else if(parse_head == ptStr)
    {
      print_en = parse_data.toInt();
    }
    else if(parse_head == peStr)
    {
      printEEPROM();
    }
    else
    {
      Serial.println(warnString + headDelim + parse_head);
    }
  }  
}

/*
 * Print EEPROM values to Serial
 */
void printEEPROM() {
  String printStr = eeprString + headDelim + String(kp.val_float,4) 
    + dataDelim + String(ki.val_float,4) + dataDelim + String(kd.val_float,4) 
    + dataDelim + deadband.val_float;
  Serial.println(printStr);
}
