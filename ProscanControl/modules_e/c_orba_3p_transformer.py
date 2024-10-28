#!/usr/bin/python3

# Original development, 2023-09-08 in ipro 0906v04c.

# from prs.linal_001.c_vec3 import *
# from prs.linal_001.c_matrix33 import *
from modules_e.c_vec3 import c_vec3
from modules_e.c_matrix33 import c_matrix33

import sys

class c_orba_3p_transformer:
  def __init__(self):
    self.S0_name = None
    self.S1_name = None
    self.fidu_A_name = None
    self.fidu_B_name = None
    self.fidu_C_name = None
    #
  #
  def set_system_names(self, S0_name, S1_name):
    self.S0_name = S0_name
    self.S1_name = S1_name
  #
  def set_fidu_names(self, fidu_A_name, fidu_B_name, fidu_C_name):
    self.fidu_A_name = fidu_A_name
    self.fidu_B_name = fidu_B_name
    self.fidu_C_name = fidu_C_name
  #
  def validate_names(self, S_name, fidu_name):
    rv = 0
    f = fiduname
    A = self.fidu_A_name;  B = self.fidu_B_name;  C = self.fidu_C_name;
    #
    if self.S0_name == None or self.S1_name == None:
      print("Trying to validate S_name but names not set.")
      rv = 1
    if A==None or B==None or C==None:
      print("Trying to validate fidu_name but names not set.")
      rv = 1
    if   S_name != self.S0_name and S_name != self.S1_name:
      print("Bad S_name: ", S_name)
      rv = 1
    if f != A and f != B and f != C:
      print("Bad fidu_name: ", fidu_name)
      rv = 1
    return rv
  #
  def set_fidu(self, S_name, fidu_name, v):
    rv = self.validate_names(S_name, fidu_name)
    if rv != 0:
      print("Error e10.  Bad name.")
      print("  c_orba_3p_transformer.py.")
      sys.exit(1)
    if   S_name == S0_name:
      if   fidu_name == fidu_A_name:  self.set_fidu_S0_A( v )
      elif fidu_name == fidu_B_name:  self.set_fidu_S0_B( v )
      elif fidu_name == fidu_C_name:  self.set_fidu_S0_C( v )
    elif S_name == S1_name:
      if   fidu_name == fidu_A_name:  self.set_fidu_S1_A( v )
      elif fidu_name == fidu_B_name:  self.set_fidu_S1_B( v )
      elif fidu_name == fidu_C_name:  self.set_fidu_S1_C( v )
    #
  #
  def set_point_S0_A(self, v):  self.S0_A = v.copy()
  def set_point_S0_B(self, v):  self.S0_B = v.copy()
  def set_point_S0_C(self, v):  self.S0_C = v.copy()
  def set_point_S1_A(self, v):  self.S1_A = v.copy()
  def set_point_S1_B(self, v):  self.S1_B = v.copy()
  def set_point_S1_C(self, v):  self.S1_C = v.copy()
  #
  def set_fidu_S0_A(self, v):  self.S0_A = v.copy()
  def set_fidu_S0_B(self, v):  self.S0_B = v.copy()
  def set_fidu_S0_C(self, v):  self.S0_C = v.copy()
  def set_fidu_S1_A(self, v):  self.S1_A = v.copy()
  def set_fidu_S1_B(self, v):  self.S1_B = v.copy()
  def set_fidu_S1_C(self, v):  self.S1_C = v.copy()
  #
  def set_fidu_S0_1(self, v):  self.S0_A = v.copy()
  def set_fidu_S0_2(self, v):  self.S0_B = v.copy()
  def set_fidu_S0_3(self, v):  self.S0_C = v.copy()
  def set_fidu_S1_1(self, v):  self.S1_A = v.copy()
  def set_fidu_S1_2(self, v):  self.S1_B = v.copy()
  def set_fidu_S1_3(self, v):  self.S1_C = v.copy()
  #
  def get_vec(self, v, direction=1):
    # direction  1:  get S1 vec, convert S0->S1.
    # direction -1:  get S0 vec, convert S1->S0.
    if   direction ==  1:   v1 = self.P.mult_vec3(v)
    elif direction == -1:   v1 = self.P_inv.mult_vec3(v)
    else:
      print("Error e10.  c_orba_3p_transformer.py.")
      print("  get_vec().")
      print("  Bad direction.")
      sys.exit(1)
    return v1
  def get_pnt(self, C, direction=1):
    # v0 has components equal to coordinates of C.
    if   direction ==  1:
      v1 = self.P.mult_vec3( C )
      C1 = self.S1_ori0.add( v1 )
    elif direction == -1:
      v1 = self.P_inv.mult_vec3( C )
      C1 = self.S0_ori1.add( v1 )
    else:
      print("Error e11.  c_orba_3p_transformer.py.")
      print("  get_pnt().")
      print("  Bad direction.")
      sys.exit(1)
    return C1
    #
  def pro(self):
    # Find the transformation for converting vectors and
    # points between the two systems of coordinates.
    #
    self.determine_basis2()              # check-1
    self.determine_direction_cosines()   # check-1
    self.determine_origin_coordinates()  # check-1
    self.determine_other_bases()         # check-1
    #
  def determine_basis2(self):
    # Determine the components of basis2 vectors in
    # S0 and S1.
    self.determine_S0_basis2()
    self.determine_S1_basis2()
    #
  def determine_S0_basis2(self):
    S0_vAB = self.S0_B.minus( self.S0_A )
    S0_vAC = self.S0_C.minus( self.S0_A )
    #
    # The basis2 e_1 vector's components in S0.
    self.S0_b2_e1 = S0_vAB.copy()
    self.S0_b2_e1.make_unit()
    #
    # The basis2 e_3 vector's components in S0.
    self.S0_b2_e3 = S0_vAB.cross( S0_vAC )
    self.S0_b2_e3.make_unit()
    #
    # The basis2 e_2 vector's components in S0.
    self.S0_b2_e2 = self.S0_b2_e3.cross( self.S0_b2_e1 )
    #
  def determine_S1_basis2(self):
    S1_vAB = self.S1_B.minus( self.S1_A )
    S1_vAC = self.S1_C.minus( self.S1_A )
    #
    # The basis2 e_1 vector's components in S1.
    self.S1_b2_e1 = S1_vAB.copy()
    self.S1_b2_e1.make_unit()
    #
    # The basis2 e_3 vector's components in S1.
    self.S1_b2_e3 = S1_vAB.cross( S1_vAC )
    self.S1_b2_e3.make_unit()
    #
    # The basis2 e_2 vector's components in S1.
    self.S1_b2_e2 = self.S1_b2_e3.cross( self.S1_b2_e1 )
    #
  def determine_direction_cosines(self):
    # M converts S0->basis2
    # N converts S1->basis2
    # P converts S0->S1
    #
    # M converts S0->basis2
    self.M = c_matrix33()
    self.M.make_identity()
    self.M.m11 = self.S0_b2_e1.x # b2_e1 dot e1
    self.M.m12 = self.S0_b2_e1.y # b2_e1 dot e2
    self.M.m13 = self.S0_b2_e1.z # b2_e1 dot e3
    self.M.m21 = self.S0_b2_e2.x # b2_e2 dot e1
    self.M.m22 = self.S0_b2_e2.y # b2_e2 dot e2
    self.M.m23 = self.S0_b2_e2.z # b2_e2 dot e3
    self.M.m31 = self.S0_b2_e3.x # b2_e3 dot e1
    self.M.m32 = self.S0_b2_e3.y # b2_e3 dot e2
    self.M.m33 = self.S0_b2_e3.z # b2_e3 dot e3
    self.M_inv = self.M.copy()
    self.M_inv.transpose()
    #
    # N converts S1->basis2
    self.N = c_matrix33()
    self.N.make_identity()
    self.N.m11 = self.S1_b2_e1.x # b2_e1 dot e1
    self.N.m12 = self.S1_b2_e1.y # b2_e1 dot e2
    self.N.m13 = self.S1_b2_e1.z # b2_e1 dot e3
    self.N.m21 = self.S1_b2_e2.x # b2_e2 dot e1
    self.N.m22 = self.S1_b2_e2.y # b2_e2 dot e2
    self.N.m23 = self.S1_b2_e2.z # b2_e2 dot e3
    self.N.m31 = self.S1_b2_e3.x # b2_e3 dot e1
    self.N.m32 = self.S1_b2_e3.y # b2_e3 dot e2
    self.N.m33 = self.S1_b2_e3.z # b2_e3 dot e3
    self.N_inv = self.N.copy()
    self.N_inv.transpose()
    #
    # P:      S0->S1
    # P_inv:  S1->S0
    self.P = self.N_inv.mult_matrix33( self.M )
    self.P_inv = self.P.copy()
    self.P_inv.transpose()
    #
  def determine_other_bases(self):
    # In S0, determine S1 basis:  S0_ba1
    self.S0_ba1_e1 = self.P_inv.mult_vec3( c_vec3(1,0,0) )
    self.S0_ba1_e2 = self.P_inv.mult_vec3( c_vec3(0,1,0) )
    self.S0_ba1_e3 = self.P_inv.mult_vec3( c_vec3(0,0,1) )
    #
    # In S1, determine S0 basis:  S1_ba0
    self.S1_ba0_e1 = self.P.mult_vec3( c_vec3(1,0,0) )
    self.S1_ba0_e2 = self.P.mult_vec3( c_vec3(0,1,0) )
    self.S1_ba0_e3 = self.P.mult_vec3( c_vec3(0,0,1) )
    #
  def determine_origin_coordinates(self):
    #
    # Create AO vectors within each system.
    S0_ao0 = self.S0_A.copy()
    S0_ao0.mult_scalar( -1.0 )
    S1_ao1 = self.S1_A.copy()
    S1_ao1.mult_scalar( -1.0 )
    #
    # Determine the coordinates of S0 origin in S1.
    S1_ao0 = self.P.mult_vec3( S0_ao0 )
    self.S1_ori0 = self.S1_A.add( S1_ao0 )
    #
    # Determine the coordinates of S1 origin in S0.
    # Point (S0_ori1.x, S0_ori1.y).
    S0_ao1 = self.P_inv.mult_vec3( S1_ao1 )
    self.S0_ori1 = self.S0_A.add( S0_ao1 )
    #





