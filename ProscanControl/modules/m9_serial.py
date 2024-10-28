#!/usr/bin/python3


import sys
import platform
import socket


############################################
print('platform.system(): ', platform.system())
hostname = socket.gethostname()
print('hostname: ', hostname)
############################################



############################################
if hostname == 'shiva2':
  #
  import serial
  #
  # Starts serial port when module is loaded.
  # spo:  serial port
  ### spo = serial.Serial(
  ###   port='COM5', baudrate=9600, bytesize=8,
  ###   timeout=2, stopbits=serial.STOPBITS_ONE
  ###   )
  try:
    spo = serial.Serial(
      port='COM5', baudrate=9600, bytesize=8,
      timeout=1, stopbits=serial.STOPBITS_ONE
      )
  except:
    print("Error.  An exception was raised by the")
    print("  call to serial.Serial().")
    print("  - Do you have two programs trying to")
    print("    access the serial port maybe?")
    sys.exit(1)
  #
else:
  from modules.c_sim_serial import *
  spo = c_sim_serial()
############################################





