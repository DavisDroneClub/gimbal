/*
 * ----------------------------------------------------------------------
 * EEPROM FILE
 * PROVIDES FUNCTIONS FOR READING AND WRITING FROM EEPROM
 * ----------------------------------------------------------------------
 * 
 * The Arduino EEPROM is used to store the PID tuning values
 * Change the EEPROM values through the serial port communication
 * 
 * DATA       ADDRESSES
 * Kp         0 - 3
 * Ki         4 - 7
 * Kd         8 - 11
 * deadband   12- 15
 * x_cal      16- 19
 * y_cal      20- 23
 * z_cal      24- 27
 */

/**
 * Write current values to EEPROM
 */
void writeVals() {
  for(int i = 0; i < 4; i++) {
    EEPROM.update( 0+i, kp.val_byte[i]);
    EEPROM.update( 4+i, ki.val_byte[i]);
    EEPROM.update( 8+i, kd.val_byte[i]);
    EEPROM.update(12+i, deadband.val_byte[i]);
    EEPROM.update(16+i, gyro_x_cal.val_byte[i]); 
    EEPROM.update(20+i, gyro_y_cal.val_byte[i]); 
    EEPROM.update(24+i, gyro_z_cal.val_byte[i]); 
  }
}

/*
 * Read EEPROM values to variables
 */
void readVals() {
  for(int i = 0; i < 4; i++) {
    kp.val_byte[i]         = EEPROM.read(0+i);
    ki.val_byte[i]         = EEPROM.read(4+i);
    kd.val_byte[i]         = EEPROM.read(8+i);
    deadband.val_byte[i]   = EEPROM.read(12+i);
    gyro_x_cal.val_byte[i] = EEPROM.read(16+i);
    gyro_y_cal.val_byte[i] = EEPROM.read(20+i);
    gyro_z_cal.val_byte[i] = EEPROM.read(24+i);
  }
}



