
import sys
import os
import shutil
import winsound
import time
from datetime import datetime
import math
from matplotlib import pyplot as plt

from modules.m1 import *
from modules.m9_serial import spo






#######################################################
class c_arec:
  #
  def __init__(self):
    print("Remember, use q to quit any time.")
    self.name = []  # area name
    self.tstamp = []  # time stamp
    self.px = []    # x pos
    self.py = []    # y pos
    self.pz = []    # z pos
    self.notes = [] 
    self.name_prefix = 'a'
    #
    self.n_area = 0
    #
  #
  def prefix(self,p=None):
    if p != None:
      self.name_prefix = p
    print("Using new prefix'"+self.name_prefix+"'")
    print("  Next area name will be: ",self.next_name())
  #
  def clear_areas(self):
    self.name = []  # area name
    self.tstamp = []  # time stamp
    self.px = []    # x pos
    self.py = []    # y pos
    self.pz = []    # z pos
    self.notes = [] 
    #
    self.n_area = 0
  #
  def ls(self):  # list
    for i in range(self.n_area):
      line = str(i)+'. '
      # line += "("+str(self.px[i])+','+str(self.py[i])+")"
      line += "("+str(self.px[i])+','+str(self.py[i])+","+str(self.pz[i])+")"
      line += "  ["+self.name[i]+"]"
      line += "  - " + self.notes[i]
      print(line)
    print("n_area: ", self.n_area)
  #
  def next_name(self):
    auto_n = len(self.name_prefix)+3
    next_i = 0   # The first possible will actually be 1
    n0 = len(self.name_prefix)
    for i in range(self.n_area):
      if len(self.name[i]) != auto_n:  continue
      if not self.name[i].startswith( self.name_prefix ):  continue
      # print("::> ", self.name[i])
      oor = 0  #  index out of range
      for j in range(3):  # 3 digits
        acode = ord(self.name[i][n0+j])
        if acode < 48:  oor = 1    # ascii 0
        if acode > 57:  oor = 1    # ascii 9
      if oor == 1:  continue
      cunum = int(self.name[i][n0:])
      if cunum > next_i:  next_i = cunum
    next_i += 1
    return self.name_prefix+'{0:03d}'.format( next_i )
  #
  def set(self, name=None):
    #
    if name == None:  uname = self.next_name()
    else:             uname = name
    i = self.n_area
    self.n_area += 1
    #
    self.name.append( uname )
    self.tstamp.append( datetime.now() )
    self.px.append( 0 )
    self.py.append( 0 )
    self.pz.append( 0 )
    self.notes.append( "" )
    print("Setting ", self.name[i])
    #
    ###
    ### cbuf() # Make sure the buffer is clear.
    ### send = bytes( "p\r\n".encode() )
    ### spo.write( send )
    ### serda = spo.readline()
    ### ll = serda.decode("Ascii").split(',')
    ### self.px[i] = int(ll[0])
    ### self.py[i] = int(ll[1])
    ### self.pz[i] = int(ll[2])
    self.px[i], self.py[i], self.pz[i] = self.pos()
  #
  def pos(self, mode=None):
    # report the current position
    cbuf() # Make sure the buffer is clear.
    send = bytes( "p\r\n".encode() )
    spo.write( send )
    serda = spo.readline()
    ll = serda.decode("Ascii").split(',')
    posx = int(ll[0])
    posy = int(ll[1])
    posz = int(ll[2])
    if mode == 'print':  print("x y z: ",posx, posy, posz)
    return posx, posy, posz
  #
  def go(self,name):
    j = -1
    for i in range(self.n_area):
      if name == self.name[i]:
        j = i
    if j == -1:
      print("Couldn't find area.")
      return
    ###
    x = self.px[j]
    y = self.py[j]
    z = self.pz[j]
    #
    ouline = "g"
    ouline += " {0:d}".format( x )
    ouline += " {0:d}".format( y )
    ouline += " {0:d}".format( z )
    ouline += "\r\n"
    send = bytes( ouline.encode() )
    spo.write( send )
  #
  def load(self):
    self.load_data_format_3()
  #
  def load_data_format_3(self):
    print("Loading data...")
    #
    fname_base =  "arec.data"
    fname_default = "config/"+fname_base
    fname_user = "user/"+fname_base
    if os.path.isfile( fname_user ):
      fname = fname_user
      print("Found user file.")
    else:
      fname = fname_default
      print("Using default file.")
    #
    self.clear_areas()
    #
    print("  Loading: ", fname )
    f = open(fname)
    l = f.readline().strip()
    if not l.startswith('!data_format'):
      print("Loading failed due to incorrect version.")
      print("  l:  ", l)
      print("  Expect sart:  !data_format")
      return
    ll = l.split(' ')
    if len(ll) < 2:
      print("Loading failed due to incorrect version.")
      print("  l:  ", l)
      return
    if ll[1] != '3':
      print("Loading failed due to incorrect version.")
      print("  ll[1]:  ", ll[1])
      print("  Expected:  3")
      return
    for l in f:
      l = l.strip()
      if len(l) == 0:  continue
      if l[0] == '#':  continue
      ###
      ll = l.split(';')
      if len(ll) < 7:
        print("A failure occurred during loading.")
        print("  Data not completely loaded.")
        return
      tstamp = datetime.strptime(ll[1].strip(), '%Y-%m-%d %H:%M:%S.%f')
      self.tstamp.append( tstamp )
      self.px.append( int(ll[2].strip()) )
      self.py.append( int(ll[3].strip()) )
      self.pz.append( int(ll[4].strip()) )
      self.name.append(   ll[5].strip() )
      self.notes.append(  ll[6].strip() )
      ###
    f.close()
    self.n_area = len(self.name)
    print("  Done.")
  #
  def save(self, ufname=None):
    self.save_data_format_3(ufname)
  #
  def save_data_format_3(self, ufname=None):
    savetime_dto       = datetime.now()
    savetime_for_fname = savetime_dto.strftime("%Y%m%d_%H%M%S")
    savetime           = savetime_dto.strftime("%Y-%m-%d %H:%M:%S")
    #
    if self.n_area == 0:
      print("Nothing to save.")
      return
    fname_base =  "arec.data"
    if ufname != None:
      fname_base = ufname
    #
    fname1 = 'user/arec_'+savetime_for_fname+'.data'
    fname2 = "user/"+fname_base
    #
    # Save as both fname_user and
    # as:  user/arec_yyyymmdd_hhmmss.data
    #
    ou = ''
    ou += '!data_format 3\n'
    ou += '# Save time: '+savetime+'\n'
    ou += '# Position data is in um.\n'
    ou += '# Blank lines and comment lines starting with # are allowed in the data.\n'
    ou += '# i ; time stamp ; px(inverted) ; py ; pz ; name ; notes\n'
    for i in range(self.n_area):
      tstr = self.tstamp[i].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
      ou += str(i)
      ou += " ; " + tstr
      ou += " ; " + str(self.px[i])
      ou += " ; " + str(self.py[i])
      ou += " ; " + str(self.pz[i])
      ou += " ; " + self.name[i]
      ou += " ; " + self.notes[i]
      ou += '\n'
    #
    print("Saving "+fname1+" ...")
    fz = open(fname1, 'w')
    fz.write(ou)
    fz.close()
    print("Saving "+fname2+" ...")
    fz = open(fname2, 'w')
    fz.write(ou)
    fz.close()
  #
  def backup_user_demand(self):
    ts_dto = datetime.now()
    ts = ts_dto.strftime("%Y%m%d_%H%M%S")
    ufname = "arec_"+ts+".data"
    self.save( ufname )
  #
  def ls_rel(self):
    # Show the positions of all the areas relative
    # to the current stage position.
    ### cbuf() # Make sure the buffer is clear.
    ### send = bytes( "p\r\n".encode() )
    ### spo.write( send )
    ### serda = spo.readline()
    ### ll = serda.decode("Ascii").split(',')
    ### cupx = int(ll[0])
    ### cupy = int(ll[1])
    cupx, cupy, cupz = self.pos()
    #
    # relative positions.
    repx = []
    repy = []
    repz = []
    reran = []  # relative range (distance), not including z
    reazi = []  # relative azimuth
    #
    for i in range(self.n_area):
      repx.append(0)
      repy.append(0)
      repz.append(0)
      reran.append(0)
      reazi.append(0)
      #
      repx[i] = -( self.px[i] - cupx ) # neg because inverted stage x axis
      repy[i] = self.py[i] - cupy
      repz[i] = self.pz[i] - cupz
      reran[i] = math.hypot( repx[i], repy[i] )
      reazi[i] = math.atan2( repy[i], repx[i] )
      reazi[i] *= 180 / math.pi
      reazi[i] = 90 - reazi[i]
      if reazi[i] < 0:  reazi[i] += 360
    #
    for i in range(self.n_area):
      line = str(i)+'. '
      # line += "pos_xyz("+str(self.px[i])+','+str(self.py[i])+','+str(self.pz[i])+")"
      line += "rel xyz("+str(repx[i])+','+str(repy[i])+','+str(repz[i])+")"
      line += "  "
      # line += "range_azim("+str(reran[i])+','+str(reazi[i])+")"
      line += "azim_range({0:0.0f}".format(reazi[i])
      # line += "deg,"
      # line += "°," # fails
      line += u'\u00B0'  # degree symbol
      line += ","
      line += "{0:0.0f}".format(reran[i])
      # line += ",um)"
      # line += ","
      line += u'\u00B5'  # greek mu
      line += "m)"
      line += "  ["+self.name[i]+"]"
      # line += "  - " + self.notes[i]
      uline = line.encode('utf-8')
      print(line)
    print("azimuths use geo reference frame and degrees.")
    print("current x y z: ", cupx, cupy, cupz)
    print("n_area: ", self.n_area)
    #
  #
  #
  def determine_area_cir(self):
    all_good = True
    x_min = None
    x_max = None
    y_min = None
    y_max = None
    first = True
    ###
    for i in range(self.n_area):
      if self.name[i].startswith("e+c"):
        x = self.px[i]
        y = self.py[i]
        if first:
          first = False
          x_min = x
          x_max = x
          y_min = y
          y_max = y
        else:
          if x < x_min:    x_min = x
          if x > x_max:    x_max = x
          if y < y_min:    y_min = y
          if y > y_max:    y_max = y
    ###
    if x_min == x_max or y_min == y_max:
      all_good = False
      return all_good, 0, 0, 0
    #
    diag1_x = x_max - x_min
    diag1_y = y_max - y_min
    cx0 = x_min + diag1_x/2
    cy0 = y_min + diag1_y/2
    #
    r = 0
    for i in range(self.n_area):
      if self.name[i].startswith("e+c"):
        x = self.px[i]
        y = self.py[i]
        tesr = math.hypot(x-cx0, y-cy0)
        if tesr > r:  r = tesr
    #
    return all_good, cx0, cy0, r
  #
  def determine_area_rec(self):
    all_good = True
    n_rec = 0
    x_min = None
    x_max = None
    y_min = None
    y_max = None
    first = True
    for i in range(self.n_area):
      if self.name[i].startswith("e+r"):
        n_rec += 1
        x = self.px[i]
        y = self.py[i]
        if first:
          first = False
          x_min = x
          x_max = x
          y_min = y
          y_max = y
        else:
          if x < x_min:    x_min = x
          if x > x_max:    x_max = x
          if y < y_min:    y_min = y
          if y > y_max:    y_max = y
    ###
    if x_min == x_max or y_min == y_max:
      all_good = False
      return all_good, [0,0,0,0,0], [0,0,0,0,0]
    #
    # For plotting.
    # grr:  graph of rectangle edge.
    grrx = [x_min, x_max, x_max, x_min, x_min]
    grry = [y_min, y_min, y_max, y_max, y_min]
    #
    return all_good, grrx, grry
    #
  #
  def plot(self, plot_save=0, plot_grc=0):
    # plot_save:  0 no, 1 yes
    # plot_grc:  0 no, 1 yes plot current stage position
    if self.n_area == 0:
      print("No areas defined.")
      return
    # The edge will be either a circle "e+c"/n_cir
    # or a rectangle "e+r"/n_rec.
    # Center will be at (cx, cy)
    cir_good, cx, cy, circ_r = self.determine_area_cir()
    rec_good, grrx, grry = self.determine_area_rec()
    #
    if (not cir_good) and (not rec_good):
      print("Didn't find enough e+?### names for a border.")
      print("  You need at least three (e+c) or (e+r).")
      print("  And they must be able to define a min")
      print("  and a max in both x and y.")
      # print("  n_cir (e+c): ", n_cir)
      # print("  n_rec (e+r): ", n_rec)
      return
    else:
      print("e+c:")
      print("  cx: ", cx)
      print("  cy: ", cy)
      print("  circ_r: ", circ_r)
    #
    if cir_good:
      circ_n_seg = 80
      circ_n_pnt = circ_n_seg+1
      circ_x = []
      circ_y = []
      circ_dang = math.pi * 2 / circ_n_seg
      for i in range(circ_n_pnt):
        ang = circ_dang * i
        circ_x.append( cx + circ_r * math.cos( ang ) )
        circ_y.append( cy + circ_r * math.sin( ang ) )
    #
    # gra will be the areas.
    grax = []
    gray = []
    graa = []  # annotation
    for i in range(self.n_area):
      if not self.name[i].startswith("e+"):
        grax.append( self.px[i] )
        gray.append( self.py[i] )
        graa.append( self.name[i] )
    gra_n = len(grax)
    #
    # grc:  graph current position.
    grcx = []
    grcy = []
    if plot_grc == 1:
      cbuf() # Make sure the buffer is clear.
      send = bytes( "p\r\n".encode() )
      spo.write( send )
      serda = spo.readline()
      ll = serda.decode("Ascii").split(',')
      cupx = int(ll[0])
      cupy = int(ll[1])
      #
      grcx.append(cupx)
      grcy.append(cupy)
    #
    if cir_good:
      plt.plot( circ_x, circ_y,
        color='#000099'
        )
    if rec_good:
      plt.plot( grrx, grry,
        color='#000099'
        )
    if plot_grc == 1:
      plt.plot( grcx, grcy,
        marker='+',
        markerfacecolor='None',
        markeredgecolor='#aa0000',
        linestyle='None'
        )
    plt.plot( grax, gray,
      marker='o',
      markerfacecolor='None',
      markeredgecolor='#ff0000',
      linestyle='None'
      )
    #
    ax = plt.gca()
    gra_style = dict(size=8, color='black')
    for i in range(gra_n):
      ha='left'
      ax.text( grax[i], gray[i], ' '+graa[i], ha=ha, **gra_style )
    #
    ax.invert_xaxis()
    ax.set_aspect('equal', adjustable='box')
    #
    ts_dto = datetime.now()
    ts = ts_dto.strftime("%Y%m%d_%H%M%S")
    ufname = "user/arec_"+ts+".png"
    #
    if plot_save == 1:  plt.savefig( ufname )
    plt.show()
  #
  def run(self):   # human user system
    ####################################
    while( 1 ):
      #########################
      print()
      print("Entering arec user run.")
      print("  Use q to quit.")
      while True:
        print()
        uline = input("u>> ")
        if uline == 'q':
          print()
          return
        elif uline == 'ls':  self.ls()
        elif uline == 'ls rel':
          self.ls_rel()
        elif uline == 'load':  self.load()
        elif uline == 'save':  self.save()
        elif uline == 'backup':
          self.backup_user_demand()
          # This backups up user/arec.data.
        elif uline == 'plot ?':
          print("Usage:")
          print("  plot          # Just show the plot.")
          print("  plot cp       # Also plot current position.")
          print("  plot save     # Also save a PNG of the graph.")
          print("  plot save cp  # Save PNG, include current pos.")
          print("  plot cp save  # Save PNG, include current pos.")
        elif uline == 'plot':  self.plot()
        elif uline == 'plot save':  self.plot( plot_save=1)
        elif uline == 'plot cp':  # cp:  also plot current position
          self.plot(plot_grc=1)
        elif uline == 'plot save cp':  self.plot( plot_save=1, plot_grc=1)
        elif uline == 'plot cp save':  self.plot( plot_save=1, plot_grc=1)
        elif uline == 'clear':  self.clear_areas()
        elif uline == 'pos':  self.pos(mode='print')
        elif uline.startswith( 'set' ):
          if uline == 'set':
            self.set()
          elif len(uline) > len('set '):
            aname = uline[4:]
            self.set(aname)
          else:
            print("Problem using set.  Unexpected uline.")
            print("  uline: ", uline)
        elif uline.startswith('go '):
          if len(uline) <= len('go '):
            print("No entered entered.")
          else:
            # go abc
            # 01234567
            aname = uline[3:]
            self.go( aname )
        elif uline.startswith('prefix'):
          if uline == 'prefix':
            self.prefix()  # Just show the current prefix.
          elif len(uline) <= len('prefix '):  # note the extra space
            print("No prefix entered.")
            self.prefix()
          else:
            # prefix abc
            # 01234567
            pre = uline[7:]
            self.prefix( pre )
        else:
          print("Unrecognized input.")
      #########################
    ####################################
  #
#######################################################





