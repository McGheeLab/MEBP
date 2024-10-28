#!/usr/bin/python3

import sys
import os
import winsound
import time

from modules.m1 import *
from modules.m9_serial import spo



#######################################################
class c_locup:
  #
  def __init__(self):
    # center of insert:  cx cy
    self.cx = None
    self.cy = None
    self.cnum = -1    # the culture number
    self.pattern_file = "locup_pattern.data"
  #
  def set_cnum(self, cnum):
    self.cnum = cnum
  #
  def load_config(self):
    print("Loading config...")
    #
    fname_base =  "locup.config"
    fname_default = "config/"+fname_base
    fname_user = "user/"+fname_base
    #
    if not os.path.isfile( fname_default ):
      print("Error.  Missing default config file.")
      print("  fname: ", fname_default)
      sys.exit(1)
    self.load_one_config(fname_default)
    #
    if os.path.isfile( fname_user ):
      self.load_one_config(fname_user)
    #
  #
  def load_one_config(self, fname):
    print("  Loading: ", fname )
    f = open(fname)
    for l in f:
      l = l.strip()
      if len(l) == 0:  continue
      if l[0] == '#':  continue
      if not l.startswith('!'):
        print("Error e20.  Bad line reading config.")
        print("  fname: ", fname)
        print("  bad line: ", l)
        sys.exit(1)
      ll = l.split(' ')
      mm = [m.strip() for m in l.split(';')]
      key = mm[0]
      ###
      if key == '!culture_diam':
        self.culture_diam = int(mm[1])
        self.culture_r = int(self.culture_diam / 2)
      elif key == '!pattern_file':
        self.pattern_file = mm[1]
      elif key == '!end_of_data':
        break
      else:
        print("Error.  Unrecognized key in config file.")
        print("  key: ", key)
        print("  fname: ", fname)
        sys.exit(1)
    f.close()
    # print("  culture diam: ", self.culture_diam)
  #
  #
  def go_fov(self, i1o_fov):
    i = i1o_fov-1
    if i < 0 or i >= self.n_pattern:
      print("Error.  locup asked for OOR FOV.")
      print("  i1o_fov: ", i1o_fov)
      print("  n_pattern:  ", self.n_pattern)
      return
    #########
    x = self.cx + self.pax[i]
    y = self.cy + self.pay[i]
    ouline = "g"
    ouline += " {0:d}".format( x )
    ouline += " {0:d}".format( y )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
    #########
  #
  def run_pattern(self):
    ######################
    print()
    for i in range(self.n_pattern):
      #########
      pline = ""
      if self.pam[i] != "":
        pline += self.pam[i]
      pline += " : c"+str(self.cnum)
      pline += " : "+self.paname[i]  # +'\n'
      print(pline)
      #########
      x = self.cx + self.pax[i]
      y = self.cy + self.pay[i]
      ouline = "g"
      ouline += " {0:d}".format( x )
      ouline += " {0:d}".format( y )
      ouline += "\r\n"
      send = bytes( ouline.encode() )
      spo.write( send )
      #########
      # uline = input("  Hit [enter] when done (q=quit):  \n")
      uline = input("  Hit [enter] when done (q=quit):  ")
      if uline == 'q':  return -1
    ######################
    self.beep(1)
    return 0
  #
  def go_edge(self, capo):  # capo:  "cardinal point"
    #  self.culture_r = int(self.culture_diam / 2)
    if self.cx == None or self.cy == None:
      print("Warning.  Culture center not set.")
      print("  Not moving.")
      return
    if capo == 'N' or capo == 'n':
      x = self.cx
      y = self.cy + self.culture_r
    elif capo == 'S' or capo == 's':
      x = self.cx
      y = self.cy - self.culture_r
    elif capo == 'W' or capo == 'w':
      x = self.cx + self.culture_r
      y = self.cy
    elif capo == 'E' or capo == 'e':
      x = self.cx - self.culture_r
      y = self.cy
    else:
      print("Error.  Unrecognized cardinal point.")
      print("  capo: ", capo)
      print("  Expected on of:  N W S E n w s e.")
      return
    ouline = "g"
    ouline += " {0:d}".format( x )
    ouline += " {0:d}".format( y )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def beep(self, n_beep):
    for i in range(n_beep):
      if i != 0:  time.sleep(0.1)
      winsound.Beep(1600,200)  # (freq in Hz, duration in ms)
      # os.system('\a')
      # sys.stdout.write('\a')
      # sys.stdout.flush()
  #
  def get_useq(self, se):
    # Gets the useq from the start edge.
    # useq is the sequence in which edges will be reset.
    start_edge = se.upper() # Converts n->N, s->S, ...
    if len(se) == 0:  start_edge = 'N'  # Assume N.
    if   start_edge == 'N':  useq = ['N', 'S', 'W', 'E']
    elif start_edge == 'S':  useq = ['S', 'N', 'W', 'E']
    elif start_edge == 'W':  useq = ['W', 'E', 'N', 'S']
    elif start_edge == 'E':  useq = ['E', 'W', 'N', 'S']
    else:                    useq = None
    return useq
  #
  def get_edges(self, start_edge=None, go_to_first_edge=False):
    #
    # 0 E, 1 N, 2 W, 3 S
    edvalx = [0,0,0,0]
    edvaly = [0,0,0,0]
    edi = 0  # Edges in eval{x,y} order.  0E, 1N, 2W, 3S
    dima = { 'E':0, 'N':1, 'W':2, 'S':3 }
    #
    print("Resetting edges for culture "+str(self.cnum)+".")
    #
    ped = 'N'
    useq = self.get_useq(ped)
    #
    if start_edge != None:
      ped = start_edge
      useq = self.get_useq(start_edge)
    ######################################
    # Get temporary edge positions.
    ped = None
    n_edge = 4
    for esi in range(n_edge):
       # useq[esi]:  Edges in sequence they are set.
      preped = ped
      ped = useq[esi]
      if esi == 0:
        if go_to_first_edge:  self.go_edge(ped)
        pline = "Adjust "+ped+" edge and hit [enter], q to quit.\n"
        pline += "  You can choose a different edge to start.\n"
        pline += "  E.g. w[edge] to force west edge.\n"
      else:
        self.go_edge_rough(preped, ped)
        pline = "Adjust "+ped+" edge and hit [enter], q to quit.\n"
      while True:
        print(pline)
        uline = input("u>> ")
        if uline == '':  break
        if uline == 'q':  return -1
        if esi == 0:
          uu = uline.upper()
          if uu=='N' or uu=='S' or uu=='W' or uu=='E':  ped = uu
          elif uu == '':  break
          else:  continue
          useq = self.get_useq(uu)
          break
      # Get and record temporary values.
      edi = dima[ ped ]
      edvalx[edi], edvaly[edi] = self.get_pos()
    ######################################
    # Set the center using temporary edge positions.
    self.cx = int((edvalx[0] + edvalx[2]) / 2)
    self.cy = int((edvaly[1] + edvaly[3]) / 2)
    #
    self.beep(1)
    #
    return 0
    #
  #
  def get_pos(self):
    cbuf()
    send = bytes( "p\r\n".encode() )
    spo.write( send )
    serda = spo.readline()
    ll = serda.decode("Ascii").split(',')
    return int(ll[0]), int(ll[1]) # [0] x, [1] y.
  #
  def get_edges_2(self, start_edge):
    # Called by muwp during reset multi edges.
    # start_edge is one of these:  N S E W
    #####
    rv = self.get_edges(
      start_edge=start_edge,
      go_to_first_edge=True )
    # Returns -1 if a quit was requested.
    #
    return rv
    #
  #
  #
  def go_edge_rough(self, edge1, edge2):
    ee = edge1+"->"+edge2
    rad = int(self.culture_r)
    diam = int(2 * self.culture_r)
    #
    if   ee == 'N->S':
      dx =   0
      dy =  -diam
    elif ee == 'S->N':
      dx =   0
      dy =   diam
    elif ee == 'W->E':
      dx =  -diam
      dy =   0
    elif ee == 'E->W':
      dx =   diam
      dy =   0
    elif ee == 'N->W':
      dx =   rad
      dy =  -rad
    elif ee == 'W->N':
      dx =  -rad
      dy =   rad
    elif ee == 'N->E':
      dx =  -rad
      dy =  -rad
    elif ee == 'E->N':
      dx =   rad
      dy =   rad
    elif ee == 'S->W':
      dx =   rad
      dy =   rad
    elif ee == 'W->S':
      dx =  -rad
      dy =  -rad
    elif ee == 'S->E':
      dx =  -rad
      dy =   rad
    elif ee == 'E->S':
      dx =   rad
      dy =  -rad
    else:
      print("Warning:  c_locup::go_edge_rough() failed.")
      print("  The requested edges were not recognized.")
      print("  edge1: ", edge1)
      print("  edge2: ", edge2)
      return
    #
    ouline = "gr"  # the "r" is to go relative to current position
    ouline += " {0:d}".format( dx )
    ouline += " {0:d}".format( dy )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def go_ccw_edge_rough(self, in_edi):
    if   in_edi == 0:  self.go_E_to_N_edge_rough()
    elif in_edi == 1:  self.go_N_to_W_edge_rough()
    elif in_edi == 2:  self.go_W_to_S_edge_rough()
    elif in_edi == 3:  self.go_S_to_E_edge_rough()
  #
  def go_E_to_N_edge_rough(self):
    dx =   int(self.culture_r)
    dy =   int(self.culture_r)
    ##
    ouline = "gr"  # the "r" is to go relative to current position
    ouline += " {0:d}".format( dx )
    ouline += " {0:d}".format( dy )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def go_N_to_W_edge_rough(self):
    dx =   int(self.culture_r)
    dy = - int(self.culture_r)
    ##
    ouline = "gr"  # the "r" is to go relative to current position
    ouline += " {0:d}".format( dx )
    ouline += " {0:d}".format( dy )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def go_W_to_S_edge_rough(self):
    dx = - int(self.culture_r)
    dy = - int(self.culture_r)
    ##
    ouline = "gr"  # the "r" is to go relative to current position
    ouline += " {0:d}".format( dx )
    ouline += " {0:d}".format( dy )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def go_S_to_E_edge_rough(self):
    dx = - int(self.culture_r)
    dy =   int(self.culture_r)
    ##
    ouline = "gr"  # the "r" is to go relative to current position
    ouline += " {0:d}".format( dx )
    ouline += " {0:d}".format( dy )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def load_pattern_data(self):
    print("Loading pattern data...")
    #
    fname_base = self.pattern_file
    fname_default = "config/"+fname_base
    fname_user = "user/"+fname_base
    if os.path.isfile( fname_user ):
      fname = fname_user
      print("Found user file.")
    else:
      fname = fname_default
      print("Using default file.")
    #
    print("  Loading: ", fname )
    if not os.path.exists( fname ):
      print("Warning w34.  Missing pattern file.")
      print("  In c_locup::load_pattern_data().")
      print("  File not loaded.")
      print("  fname: ", fname)
      return
    #
    self.pam = []  # pattern message
    self.pax = []  # pattern x
    self.pay = []  # pattern y
    self.paname = []  # a short name for that item in the pattern list
    # several items can have the same name.
    f = open(fname)
    for l in f:
      l = l.strip()
      if len(l) == 0:  continue
      if l[0] == '#':  continue
      mm = [m.strip() for m in l.split(';')]
      key = mm[0]
      if key == '!m':  self.fread_pa(f, mm[1])
      else:
        print("Error e300.  Bad locup pattern file.")
        print("  line:  ", l)
        sys.exit(1)
      ###
    f.close()
    self.n_pattern = len(self.paname)
    print("  Done.")
    #
  #
  def fread_pa(self, f, pam):
    upam = pam
    for l in f:
      l = l.strip()
      if len(l) == 0:  break
      if l[0] == '#':  continue
      mm = [m.strip() for m in l.split(';')]
      if mm[0] == '!m':
        print("Error e301.  Bad locup pattern file.")
        print("  line:  ", l)
        sys.exit(1)
      #
      self.pam.append(upam)
      self.paname.append( mm[0] )
      self.pax.append( int(mm[1]) )
      self.pay.append( int(mm[2]) )
      upam = ""
      #
  #
#######################################################





