#!/usr/bin/python3

from modules.m1 import *
from modules.m9_serial import spo

from modules_e.c_vec3 import *
from modules_e.c_matrix33 import *
from modules_e.c_orba_3p_transformer import *

import sys

# Sample coordinates: a_x a_y a_z
# Stage coordinates:  b_x b_y b_z

class c_brec:
  def __init__(self):
    # S0:  local coord system, origin at culture center
    # S1:  stage coord system
    #
    self.init_fidu()
    self.init_fov(prefix='a', n=0)
    #
    # Zero for stage coordinates.
    self.psx0 = 0
    self.psy0 = 0
    #
    #
  def set_psx0_psy0(self, x0, y0):
    self.psx0 = x0
    self.psy0 = y0
    #
  def init_fidu(self):
    self.S0_fidu_A = c_vec3(0,0,0)
    self.S0_fidu_B = c_vec3(0,0,0)
    self.S0_fidu_C = c_vec3(0,0,0)
    self.S1_fidu_A = c_vec3(0,0,0)
    self.S1_fidu_B = c_vec3(0,0,0)
    self.S1_fidu_C = c_vec3(0,0,0)
    #
    self.orba01 = c_orba_3p_transformer()
    self.orba01_ready = False
    #
  def init_fov(self, prefix, n):
    self.fov_cur_prefix = prefix
    self.fov_S0 = []
    self.fov_S1 = []
    self.fov_name = []
    self.n_fov = n
    self.fov_cur_prefix_i = n-1
    for i in range(self.n_fov):
      self.fov_S0.append( c_vec3() )
      self.fov_S1.append( c_vec3() )
      name = prefix+'{:03d}'.format(i)
      self.fov_name.append( name )
    #
  def set_ic(self, ic):
    self.ic = ic
    #
  def add_fov(self):
    if not self.orba01_ready:
      print("  Error.  orba01 (fidu) not ready.")
      return
    x,y,z = get_p3()
    S1_p = c_vec3(x,y,z)
    self.fov_S1.append( S1_p )
    S0_p = self.orba01.get_pnt( S1_p, direction=-1 )
    self.fov_S0.append( S0_p )
    #
    self.fov_cur_prefix_i += 1
    name = self.fov_cur_prefix
    name += '{:03d}'.format( self.fov_cur_prefix_i )
    self.fov_name.append( name )
    #
    self.n_fov += 1
    #
  def get_fov_S1(self, ifov):
    x = self.fov_S1[ifov].x
    y = self.fov_S1[ifov].y
    z = self.fov_S1[ifov].z
    return x,y,z
    #
  def run_set_S1_fidu(self):
    if not self.orba01_ready:
      return 1
      # The reason is that S0 must be set for this to work.
      # This means it was loaded from a file, which means
      # S1 was also loaded and the orba01 was define.
    rv = self.run_hui_get_S1_fidu()
    if rv != 0:  return 1
    self.calculate_coordinate_systems()
    self.calculate_S1_FOVs()
    return 0
    #
  def run_define_fidu(self, S1_ori0_x, S1_ori0_y):
    # S1_ori0_(x/y) is the insert center S1 coordinates.
    #
    rv = self.run_hui_get_S1_fidu()
    if rv != 0:  return 1
    #
    # while True:
    #   uline = input("Define fidu (Y/n)? >> ")
    #   if uline == '' or uline == 'y' or uline == 'Y':  break
    #   if uline == 'n':
    #     print("  Quit without defining fidu.")
    #     return 1
    #
    S1_A = self.S1_fidu_A.copy()
    S1_B = self.S1_fidu_B.copy()
    S1_C = self.S1_fidu_C.copy()
    #
    S0_A = c_vec3(S1_ori0_x-S1_A.x, S1_A.y-S1_ori0_y, 0.0)
    S0_B = c_vec3(S1_ori0_x-S1_B.x, S1_B.y-S1_ori0_y, 0.0)
    S0_C = c_vec3(S1_ori0_x-S1_C.x, S1_C.y-S1_ori0_y, 0.0)
    #
    self.S0_fidu_A = S0_A
    self.S0_fidu_B = S0_B
    self.S0_fidu_C = S0_C
    #
    self.calculate_coordinate_systems()
    #
    return 0
    #
    #
  def run_hui_get_S1_fidu(self):
    # rv = 0.  All ok.
    # rv = 1.  Fidu not set.
    #
    prom1 = "Go to fidu A and hit enter."
    prom2 = "brec-"+str(self.ic)+">> "
    print("Remember, q quits with no changes made.")
    ufid = 'A'
    while True:
      print(prom1)
      uline = input(prom2)
      uline = ' '.join( uline.split() )  # remove duplicate spaces
      ull = uline.split(' ')
      n_ull = len(ull)
      if ull[0] == 'q':
        print("  Quit defining fidu.")
        return 1
      # if len(ull) != 0:
      if len(uline) != 0:
        print("uError.")
        continue
      #
      if ufid == 'A':
        x,y,z = get_p3()
        S1_A = c_vec3(x,y,z)
        ufid  = 'B'
        prom1 = "Go to fidu B and hit enter."
      elif ufid == 'B':
        x,y,z = get_p3()
        S1_B = c_vec3(x,y,z)
        ufid  = 'C'
        prom1 = "Go to fidu C and hit enter."
      elif ufid == 'C':
        x,y,z = get_p3()
        S1_C = c_vec3(x,y,z)
        break
      #
    #
    self.S1_fidu_A = S1_A.copy()
    self.S1_fidu_B = S1_B.copy()
    self.S1_fidu_C = S1_C.copy()
    #
    return 0
    #
  def calculate_coordinate_systems( self ):
    self.orba01 = c_orba_3p_transformer()
    self.orba01.set_fidu_S0_A( self.S0_fidu_A )
    self.orba01.set_fidu_S0_B( self.S0_fidu_B )
    self.orba01.set_fidu_S0_C( self.S0_fidu_C )
    self.orba01.set_fidu_S1_A( self.S1_fidu_A )
    self.orba01.set_fidu_S1_B( self.S1_fidu_B )
    self.orba01.set_fidu_S1_C( self.S1_fidu_C )
    self.orba01.pro()
    self.orba01_ready = True
    #
  def calculate_S1_FOVs(self):
    for i in range(self.n_fov):
      S0_p = self.fov_S0[i]
      S1_p = self.orba01.get_pnt( S0_p )
      self.fov_S1[i] = S1_p
      #
    #
  def get_save1(self):
    ou = ''
    ou += '#############################\n'
    ou += '!brec ; '+str(self.ic)+'\n'
    ou += '!n_fov ; '+str(self.n_fov)+'\n'
    if not self.orba01_ready:
      return ou
    #
    ou += '#\n'
    ou += '!S0_fidu_A'
    ou += ' ; {:7.1f}'.format( self.S0_fidu_A.x )
    ou += ' ; {:7.1f}'.format( self.S0_fidu_A.y )
    ou += ' ; {:7.1f}'.format( self.S0_fidu_A.z )
    ou += '\n'
    ou += '!S0_fidu_B'
    ou += ' ; {:7.1f}'.format( self.S0_fidu_B.x )
    ou += ' ; {:7.1f}'.format( self.S0_fidu_B.y )
    ou += ' ; {:7.1f}'.format( self.S0_fidu_B.z )
    ou += '\n'
    ou += '!S0_fidu_C'
    ou += ' ; {:7.1f}'.format( self.S0_fidu_C.x )
    ou += ' ; {:7.1f}'.format( self.S0_fidu_C.y )
    ou += ' ; {:7.1f}'.format( self.S0_fidu_C.z )
    ou += '\n'
    ou += '#\n'
    ou += '!S1_fidu_A'
    ou += ' ; {:7.1f}'.format( self.S1_fidu_A.x )
    ou += ' ; {:7.1f}'.format( self.S1_fidu_A.y )
    ou += ' ; {:7.1f}'.format( self.S1_fidu_A.z )
    ou += '\n'
    ou += '!S1_fidu_B'
    ou += ' ; {:7.1f}'.format( self.S1_fidu_B.x )
    ou += ' ; {:7.1f}'.format( self.S1_fidu_B.y )
    ou += ' ; {:7.1f}'.format( self.S1_fidu_B.z )
    ou += '\n'
    ou += '!S1_fidu_C'
    ou += ' ; {:7.1f}'.format( self.S1_fidu_C.x )
    ou += ' ; {:7.1f}'.format( self.S1_fidu_C.y )
    ou += ' ; {:7.1f}'.format( self.S1_fidu_C.z )
    ou += '\n'
    ou += '#\n'
    for i in range(self.n_fov):
      ou += '!fov_S0 ; '+'{:3d}'.format(i)
      ou += ' ; '+self.fov_name[i]
      ou += ' ; {:7.1f}'.format( self.fov_S0[i].x )
      ou += ' ; {:7.1f}'.format( self.fov_S0[i].y )
      ou += ' ; {:7.1f}'.format( self.fov_S0[i].z )
      ou += '\n'
    ou += '#\n'
    for i in range(self.n_fov):
      ou += '!fov_S1 ; '+'{:3d}'.format(i)
      ou += ' ; '+self.fov_name[i]
      ou += ' ; {:7.1f}'.format( self.fov_S1[i].x )
      ou += ' ; {:7.1f}'.format( self.fov_S1[i].y )
      ou += ' ; {:7.1f}'.format( self.fov_S1[i].z )
      ou += '\n'
    ou += '#\n'
    #
    ou += '!orba01.S1_ori0'
    ou += ' ; {:9.4f}'.format( self.orba01.S1_ori0.x )
    ou += ' ; {:9.4f}'.format( self.orba01.S1_ori0.y )
    ou += ' ; {:9.4f}'.format( self.orba01.S1_ori0.z )
    ou += '\n'
    ou += '!orba01.P\n'
    ou += '  '
    ou += ' ; {:9.4f}'.format( self.orba01.P.m11 )
    ou += ' ; {:9.4f}'.format( self.orba01.P.m12 )
    ou += ' ; {:9.4f}'.format( self.orba01.P.m13 )
    ou += '\n'
    ou += '  '
    ou += ' ; {:9.4f}'.format( self.orba01.P.m21 )
    ou += ' ; {:9.4f}'.format( self.orba01.P.m22 )
    ou += ' ; {:9.4f}'.format( self.orba01.P.m23 )
    ou += '\n'
    ou += '  '
    ou += ' ; {:9.4f}'.format( self.orba01.P.m31 )
    ou += ' ; {:9.4f}'.format( self.orba01.P.m32 )
    ou += ' ; {:9.4f}'.format( self.orba01.P.m33 )
    ou += '\n'
    #
    return ou
    #
  def read_f(self, f):
    n_S0_fidu = 0
    n_S1_fidu = 0
    #
    for l in f:
      l = l.strip()
      if len(l) == 0:  break
      if l[0] == '#':  continue
      mm = [m.strip() for m in l.split(';')]
      key = mm[0]
      if key == '!n_fov':
        n_fov = int(mm[1])
        self.init_fov( prefix='x', n=n_fov )
        #
      elif key == '!S0_fidu_A':
        x = float(mm[1]); y = float(mm[2]); z = float(mm[3])
        self.S0_fidu_A = c_vec3(x,y,z)
        n_S0_fidu += 1
      elif key == '!S0_fidu_B':
        x = float(mm[1]); y = float(mm[2]); z = float(mm[3])
        self.S0_fidu_B = c_vec3(x,y,z)
        n_S0_fidu += 1
      elif key == '!S0_fidu_C':
        x = float(mm[1]); y = float(mm[2]); z = float(mm[3])
        self.S0_fidu_C = c_vec3(x,y,z)
        n_S0_fidu += 1
      elif key == '!S1_fidu_A':
        x = float(mm[1]); y = float(mm[2]); z = float(mm[3])
        self.S1_fidu_A = c_vec3(x,y,z)
        n_S1_fidu += 1
      elif key == '!S1_fidu_B':
        x = float(mm[1]); y = float(mm[2]); z = float(mm[3])
        self.S1_fidu_B = c_vec3(x,y,z)
        n_S1_fidu += 1
      elif key == '!S1_fidu_C':
        x = float(mm[1]); y = float(mm[2]); z = float(mm[3])
        self.S1_fidu_C = c_vec3(x,y,z)
        n_S1_fidu += 1
      elif key == '!fov_S0':
        i_fov = int(mm[1])
        name = mm[2]
        x = float(mm[3])
        y = float(mm[4])
        z = float(mm[5])
        self.fov_S0[i_fov].set(x,y,z)
        self.fov_name[i_fov] = name
        # self.sort_fovs()
        self.determine_fov_cur_prefix_i()
      elif key == '!fov_S1':
        i_fov = int(mm[1])
        name = mm[2]
        x = float(mm[3])
        y = float(mm[4])
        z = float(mm[5])
        self.fov_S1[i_fov].set(x,y,z)
      elif key == '!orba01.S1_ori0':
        continue
        # TO IMPLEMENT:  Actually reading the data.
      elif key == '!orba01.P':
        for i in range(3):
          f.readline()
          # TO IMPLEMENT:  Actually reading the data.
      else:
        print("Error e301.  c_brec.py")
        print("  key: ", key)
        sys.exit(1)
    #
    if n_S0_fidu == 3 and n_S1_fidu == 3:
      self.calculate_coordinate_systems()
    #
    # Reset to default prefix.
    self.fov_cur_prefix = 'a'
    self.determine_fov_cur_prefix_i()
    #
    #
  def go_fidu(self, fid):
    if not self.orba01_ready:  return -1
    #
    fid = fid.upper()
    if fid!='A' and fid!='B' and fid!='C':  return -1
    if   fid == 'A':
      x=self.S1_fidu_A.x; y=self.S1_fidu_A.y; z=self.S1_fidu_A.z;
    elif fid == 'B':
      x=self.S1_fidu_B.x; y=self.S1_fidu_B.y; z=self.S1_fidu_B.z;
    elif fid == 'C':
      x=self.S1_fidu_C.x; y=self.S1_fidu_C.y; z=self.S1_fidu_C.z;
    #
    psx, psy, psz = int(x+self.psx0), int(y+self.psy0), int(z)
    go_p3(psx, psy, psz)
    #
    return 0
    #
  def go_fov_name(self, name):
    ii = -1
    for i in range(self.n_fov):
      if name == self.fov_name[i]:
        ii = i
        break
    if ii < 0:  return -1
    rv = self.go_fov_i(ii)
    return rv
  def go_fov_i(self, ifov):
    if not type(ifov) == int:          return -1
    if ifov < 0 or ifov > self.n_fov:  return -1
    #
    x,y,z = self.get_fov_S1(ifov)
    psx, psy, psz = int(x+self.psx0), int(y+self.psy0), int(z)
    go_p3(psx, psy, psz)
    #
    return 0
    #
  def run_seq(self):
    prompt = "brec>> "
    for i in range(self.n_fov):
      self.go_fov_i(i)
      while True:
        uline = input(prompt)
        uline = uline.strip()
        uline = ' '.join( uline.split() )  # remove duplicate spaces
        if len(uline) == 0:  break
        elif uline == 'q':
          print("  Quit brec run seq early.")
          return 1
        else:
          print("uError.")
    print("Done with brec run seq.")
    return 0
    #
    #
  def ls_fid(self):
    if not self.orba01_ready:
      print("Error.  orba not ready.")
      return
    print("- S0:")
    ou = '  A'
    ou += ' ; {:4.1f}'.format(self.S0_fidu_A.x)
    ou += ' ; {:4.1f}'.format(self.S0_fidu_A.y)
    ou += ' ; {:4.1f}'.format(self.S0_fidu_A.z)
    print(ou)
    ou = '  B'
    ou += ' ; {:4.1f}'.format(self.S0_fidu_B.x)
    ou += ' ; {:4.1f}'.format(self.S0_fidu_B.y)
    ou += ' ; {:4.1f}'.format(self.S0_fidu_B.z)
    print(ou)
    ou = '  C'
    ou += ' ; {:4.1f}'.format(self.S0_fidu_C.x)
    ou += ' ; {:4.1f}'.format(self.S0_fidu_C.y)
    ou += ' ; {:4.1f}'.format(self.S0_fidu_C.z)
    print(ou)
    #
    print("- S1:")
    ou = '  A'
    ou += ' ; {:4.1f}'.format(self.S1_fidu_A.x)
    ou += ' ; {:4.1f}'.format(self.S1_fidu_A.y)
    ou += ' ; {:4.1f}'.format(self.S1_fidu_A.z)
    print(ou)
    ou = '  B'
    ou += ' ; {:4.1f}'.format(self.S1_fidu_B.x)
    ou += ' ; {:4.1f}'.format(self.S1_fidu_B.y)
    ou += ' ; {:4.1f}'.format(self.S1_fidu_B.z)
    print(ou)
    ou = '  C'
    ou += ' ; {:4.1f}'.format(self.S1_fidu_C.x)
    ou += ' ; {:4.1f}'.format(self.S1_fidu_C.y)
    ou += ' ; {:4.1f}'.format(self.S1_fidu_C.z)
    print(ou)
    #
  def ls_fov(self):
    print("n_fov: ", self.n_fov)
    print("- S0:")
    for i in range(self.n_fov):
      ou = '  '
      ou += ' ; '+str(i)
      ou += ' ; '+self.fov_name[i]
      ou += ' ; {:4.1f}'.format(self.fov_S0[i].x)
      ou += ' ; {:4.1f}'.format(self.fov_S0[i].y)
      ou += ' ; {:4.1f}'.format(self.fov_S0[i].z)
      print(ou)
    print("- S1:")
    for i in range(self.n_fov):
      ou = '  '
      ou += ' ; '+str(i)
      ou += ' ; {:4.1f}'.format(self.fov_S1[i].x)
      ou += ' ; {:4.1f}'.format(self.fov_S1[i].y)
      ou += ' ; {:4.1f}'.format(self.fov_S1[i].z)
      print(ou)
    #
  def sort_fovs(self):
    if self.n_fov == 0:  return
    zipped = list( zip(self.fov_name,
                       self.fov_S0,
                       self.fov_S1
                 ))
    zipped.sort()
    self.fov_name, self.fov_S0, self.fov_S1 
    a,b,c = zip(*zipped)
    self.fov_name = list(a)
    self.fov_S0   = list(b)
    self.fov_S1   = list(c)
    #
  def determine_fov_cur_prefix_i(self):
    if self.n_fov == 0:
      self.fov_cur_prefix_i = -1
      return
    #
    self.sort_fovs()
    curi = -1
    for i in range(self.n_fov):
      prefix = self.fov_name[i][:-3] # remove the number part.
      ii = int(self.fov_name[i][-3:])
      if prefix == self.fov_cur_prefix:
        curi = ii
    #
    self.fov_cur_prefix_i = curi
    #
  def print_fov_next_name(self):
    i = self.fov_cur_prefix_i + 1
    name = self.fov_cur_prefix
    name += '{:03d}'.format( i )
    print("Next fov name: ", name)
    #
  def set_fov_cur_prefix(self, prefix):
    self.fov_cur_prefix = prefix
    self.determine_fov_cur_prefix_i()
    i = self.fov_cur_prefix_i + 1
    name = self.fov_cur_prefix
    name += '{:03d}'.format( i )
    print("Next fov name: ", name)
    #













