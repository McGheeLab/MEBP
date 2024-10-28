#!/usr/bin/python3

# m1:  basic control functions

import math

from modules.m9_serial import spo





#######################################################
def send(s):
  ss = s.strip()
  ouline = ss + "\r\n"
  send = bytes(ouline.encode())
  spo.write( send )
  i = 0
  while True:
    serda = spo.readline()
    slen = len(serda)
    if slen == 0:  break
    dade = serda.decode("Ascii")
    print("  serda "+str(i)+" (L"+str(slen)+"):  ["+dade+"]")
    i += 1
#######################################################

#######################################################
def cbuf():  # clear the buffer.
  serda = ""
  ###############
  ### For testing cbuf()
  # for i in range(4):
  #   serda = spo.readline()
  #   slen = len(serda)
  #   dade = serda.decode("Ascii")
  #   print("serda "+str(i)+" (L"+str(slen)+"):  ["+dade+"]")
  ###############
  print("cbuf:")
  i = 0
  while True:
    serda = spo.readline()
    slen = len(serda)
    if slen == 0:  break
    dade = serda.decode("Ascii")
    print("  serda "+str(i)+" (L"+str(slen)+"):  ["+dade+"]")
    i += 1
  print("  Clear.")
#######################################################


#######################################################
# Reports the current stage position.
def p():
  cbuf()  # Make sure the current buffer is clear.
  ouline = "p\r\n"
  send = bytes(ouline.encode())
  # spo.write(b"p\r\n")  # ask for the Prior stage current position
  spo.write( send )
  serda = spo.readline()
  print("serda :  ", end='', flush=True)
  print(serda.decode("Ascii"))
#######################################################


def get_p():
  cbuf()  # Make sure the current buffer is clear.
  #
  ouline = "p\r\n"
  send = bytes(ouline.encode())
  # spo.write(b"p\r\n")  # ask for the Prior stage current position
  spo.write( send )
  serda = spo.readline()
  # print("serda :  ", end='', flush=True)
  # print(serda.decode("Ascii"))
  #
  l = serda.decode("Ascii")
  ll = l.split(',')
  x = int( ll[0] )
  y = int( ll[1] )
  return x, y

def get_p3():
  cbuf()  # Make sure the current buffer is clear.
  #
  ouline = "p\r\n"
  send = bytes(ouline.encode())
  # spo.write(b"p\r\n")  # ask for the Prior stage current position
  spo.write( send )
  serda = spo.readline()
  # print("serda :  ", end='', flush=True)
  # print(serda.decode("Ascii"))
  #
  l = serda.decode("Ascii")
  ll = l.split(',')
  x = int( ll[0] )
  y = int( ll[1] )
  z = int( ll[2] )
  return x, y, z


def go_p3(x,y,z):
  ouline = "g"
  ouline += " {0:d}".format( x )
  ouline += " {0:d}".format( y )
  ouline += " {0:d}".format( z )
  # print("Going to:   ["+ouline+"]")
  ouline += "\r\n"
  send = bytes( ouline.encode() )
  spo.write( send )



#######################################################
# Zeroes the stage at the current position.
def p0():
  spo.write(b"px 0\r\n")
  spo.write(b"py 0\r\n")
#######################################################

#######################################################
# Zeroes just the x position of the stage.
def px0():
  spo.write(b"px 0\r\n")
#######################################################

#######################################################
# Zeroes just the y position of the stage.
def py0():
  spo.write(b"py 0\r\n")
#######################################################


#######################################################
# Move 1000 pru to the left (pru is positive to left)
def grx1000():
  spo.write(b"gr 1000 0\r\n")
#######################################################



