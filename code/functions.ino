/*
 * ----------------------------------------------------------------------
 * FUNCTIONS FILE
 * PROVIDES GENERAL FUNCTIONS FOR OPERATING THE GIMBAL
 * ----------------------------------------------------------------------
 */

/**
 * Watchdog setup
 * Resets the program if any error occurs
 */
void wdt_setup() {
  cli();                            //Block interrupts
  WDTCSR |= (1<<WDCE)|(1<<WDE);     //Enter watchdog setup mode
  //Set watchdog timer for 1 minute
  WDTCSR = (1<<WDE)|(0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0);  
  sei();                            //Re-enable interrupts
}

/*
 * Initialize servo
 * To be run once at program start
 */
void init_servo() {
  servo.attach(3);          //Declare servo output pin
  servo.write(90);          //Initialize to 90  
}

/*
 * Tune PID value
 * data (input): desired value
 * mode (input): value to set (P,I,D)
 */
void processTune(String data, char mode){
  if(mode == 'P')                           //If P value selected
  {
    kp.val_float = data.toFloat();         //Update kp and PID tuning
    myPID.SetTunings(kp.val_float, ki.val_float, kd.val_float);
    writeVals();                            //Update EEPROM values
    Serial.println("TUNING UPDATE RECEIVED");
  }
  else if(mode == 'I')                      //If I value selected
  {
    ki.val_float = data.toFloat();         //Update ki and PID tuning
    myPID.SetTunings(kp.val_float, ki.val_float, kd.val_float);
    writeVals();                            //Update EEPROM values
    Serial.println("TUNING UPDATE RECEIVED");
  }
  else if(mode == 'D')                      //If D value selected
  {
    kd.val_float = data.toFloat();         //Update kd and PID tuning
    myPID.SetTunings(kp.val_float, ki.val_float, kd.val_float);
    writeVals();                            //Update EEPROM values
    Serial.println("TUNING UPDATE RECEIVED");
  }
  else                                      //If unknown mode passed
  {
    Serial.println(warnString + headDelim + "Unknown mode");
  }
}

/*
 * Actuate servo motor and calculate deadband
 * To be run every program loop
 */
void actuate() {
  //If difference between input and setpoint is less than deadband
  if(abs(input-setpoint) < deadband.val_float)
  {
    input = setpoint;             //Set setpoint to input, output = 0
  }
  else                            //If difference is larger than deadband
  {
    servo.write((int)(output));   //Write output to servo
  }   
}

/*
 * 
 */
void enforceLoop() {
  loop_duration = micros()-loop_timer;
  if(loop_duration > 5555)               //If the loop timer exceeds 5555us
  {
    Serial.println(loopString + headDelim + loop_duration); //Print out current loop duration
  }
  else                                   //If loop timer falls below 5555us
  {
    while(micros() - loop_timer < 5555); //Wait until 5555us passes
  }
  loop_timer = micros();                 //Update the loop timer
}
