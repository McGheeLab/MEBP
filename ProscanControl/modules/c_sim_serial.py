#!/usr/bin/python3




class c_sim_serial:
  def __init__(self):
    self.x = 0
  ###
  def write(self, s):
    print("> ", s)
  ###
  def readline(self):
    return bytes( "101,102,103\r\n".encode() )
  ###




