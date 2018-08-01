#!/usr/bin/python3
'''
GIMBAL CLASS
PROVIDES FUNCTIONS FOR INTERACTING WITH GIMBAL SOFTWARE V2.X

Organization: Davis Drone Club
Author: Nicholas Chan (ngchan@ucdavis.edu)
Date: July 2018
'''

import logging
from logging.handlers import RotatingFileHandler
import serial
from typing import Union, Text
import time

class Gimbal:
    """Functions for communicating with Gimbal 2.0.

    Attributes:
        arduino: serial.Serial object
        data_delim: delimiter for parsing gimbal data
        out_headers: valid data headers for writing to gimbal
        in_headers: valid data headers for reading from gimbal
    """
    
    def __init__(self, port: Text, delim: Text= ';'):
        """
        Initializes gimbal class and connects to gimbal

        Args:
            port (str): COM port of gimbal.
            delim (str, optional): common data delimiter. Defaults to ';'.
            
        Raises:
            PermissionError: If COM port permission is denied or COM port open elsewhere.
            FileNotFoundError: If COM port could not be found.
            TimeoutError: If COM port times out.
            Exception: All other serial related errors.
        """
        self.out_headers = ["TUNEP", "TUNEI", "TUNED", "SETSP", "SETDB",
                            "SETDR", "SETPT", "SETEN", "SETPO", "CALGY",
                            "GETEP", "GETVE", "GETDN"]
        
        self.in_headers = ["WARN", "LOOP", "DATA", "EEPR", "VERS", "DEVN"]
        self.data_delim = delim
        self._init_log()
        self._connect(port)
        time.sleep(3)
        self.send("GETDN",0)
        try:
            name = self.read("DEVN").strip().split(';')
            if name[1] != "ddc-gimbal":
                raise Exception("Not a gimbal!")
        except Exception as e:
            self.log.error(str(e))
            raise Exception(str(e))
        
    def set_enable(self, value):
        """
        Enables or disables the gimbal
        """
        if value == 1 or value == 0:
            self.send("SETEN", value)
        else:
            raise ValueError('Invalid value')

    def set_direction(self, value):
        """
        Sets direction of gimbal
        """
        if value == 1 or value == 0:
            self.send("SETDR", value)
        else:
            raise ValueError('Invalid value')

    def set_position(self, value):
        """
        Sets position of gimbal
        """
        self.send("SETPO", round(value))
        
    def close(self):
        """
        closes the com port
        """
        self.arduino.close()
            
    def read(self, header: Text):
        """
        Reads lines from the gimbal until desired header is found.

        Returns:
            *Return here*

        Raises:
            ValueError: If desired header is invalid
        """
        if header not in self.in_headers:
            raise ValueError('Invalid header')
        try:
            data = self.arduino.readline()
            data = data.decode('utf-8').strip()

            for i in range(50):
                if data == '':
                    raise TimeoutError('Reached end of buffer')
                elif header not in data:
                    time.sleep(0.05)
                    data = self.arduino.readline()
                    data = data.decode('utf-8').strip()
                else:
                    self.log.info('RECV:' + data)
                    return data
            raise TimeoutError('Not found in last 10 messages')
        except TimeoutError as e:
            raise e
        except Exception as e:
            print('err: ' + str(e))
        

    def send(self, header: Text, data: Union[int, float]):
        """
        Formats and sends a data packet to the gimbal.

        Args:
            header (str): the header value as a string
            data (float/int): data as a float or an int

        Raises:
            ValueError: If header or data is invalid.
            TimeoutError: If serial timeout exceeded.
        """
        if not (isinstance(data, float) or isinstance(data, int)):
            err_msg = 'Gimbal.send(): Only float or int allowed'
            self.log.error(err_msg)
            raise ValueError(err_msg)
        elif len(header) == 0:
            err_msg = 'Gimbal.send(): No header provided'
            self.log.error(err_msg)
            raise ValueError(err_msg)
        elif header not in self.out_headers:
            err_msg = 'Gimbal.send(): Invalid header'
            self.log.error(err_msg)
            raise ValueError(err_msg)
        msg = header + self.data_delim + str(data) + '\n'
        
        try:
            self.arduino.write(msg.encode('utf-8'))
        except serial.SerialTimeoutException as e:
            err_msg = "SerialTimeout: " + str(e)
            self.log.error(err_msg)
            raise TimeoutError(err_msg)
        else:
            self.log.info('SENT:' + msg.strip())
            
    def _connect(self, port: Text, timeout: float = .5, baude: int = 115200):
        """
        Connects to the gimbal serial port.

        Args:
            port (str): the serial port name.
            timeout (float, optional): timeout duration Defaults to 5.0.
            baude (int, optional): serial port speed. Defaults to 115200.

        Raises:
            PermissionError: If COM port permission is denied, COM port open elsewhere.
            FileNotFoundError: If COM port could not be found.
            TimeoutError: If COM port times out.
            Exception: All other serial related errors.
        """
        try:
            self.arduino = serial.Serial(port,baude, timeout=timeout, write_timeout=0)
        except serial.SerialException as e:
            if "PermissionError" in str(e):
                msg = "COM port may be open elsewhere: "+str(e)
                self.log.error(msg)
                raise PermissionError(msg)
            elif "FileNotFoundError" in str(e):
                msg = "COM port not found: "+str(e)
                self.log.error(msg)
                raise FileNotFoundError(msg)
            else:
                msg = "SerialException: "+str(e)
                self.log.error(msg)
                raise Exception(msg)
        except serial.SerialTimeoutException as e:
            msg = "TimeoutError: "+str(e)
            self.log.error(msg)
            raise TimeoutError(msg)
        except Exception as e:
            msg = "Generic exception: "+str(e)
            self.log.error(msg)
            raise Exception(msg)
        else:
            self.log.info("Connected on " + port)
        
    def _init_log(self, filename: Text = 'gimbal.log',
                formatstr: Text = '%(asctime)s:%(levelname)s:%(message)s'):
        """
        Initializes the gimbal logger

        Args:
            filename (str): name of log file (default 'gimbal.log')
            formatstr (str): string format of log entries (default time:level:message)
        """
        log_format = logging.Formatter(formatstr)
        log_file   = filename
        log_handler= RotatingFileHandler(log_file, mode='a', maxBytes=5*1024*1024, backupCount=1,
                                         encoding=None, delay=0)
        log_handler.setFormatter(log_format)
        log_handler.setLevel(logging.DEBUG)

        self.log = logging.getLogger('root')
        self.log.setLevel(logging.DEBUG)
        self.log.addHandler(log_handler)

    def get_version(self):
        self.send("GETVE", 0)
        packet = self.read("VERS").split(';')
        version = packet[1].split(',')
        return version
