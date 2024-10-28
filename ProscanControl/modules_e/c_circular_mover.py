#!/usr/bin/python3

import math

class c_circular_mover:
  def __init__(self, r=1, x0=0, y0=0):
    self.r = r
    self.x0 = x0
    self.y0 = y0
  def set_r(self, r):
    self.r = r
  def set_r_from_p(self, px, py):
    self.r = math.hypot(px-self.x0, py-self.y0)
    #
  def get_ang_from_p(self, px, py):
    ang = math.atan2( py-self.y0, px-self.x0 )
    return ang
    #
  def set_center(self, x0,y0):
    self.x0 = x0
    self.y0 = y0
  def move_p_dang(self, px, py, dang):
    # ignores r
    qx = px-self.x0
    qy = py-self.y0
    cthe = math.cos(dang)
    sthe = math.sin(dang)
    sx = cthe * qx - sthe * qy
    sy = sthe * qx + cthe * qy
    return self.x0+sx, self.y0+sy
  def move_p_dang_deg(self, px, py, dang_deg):
    # ignores r
    dang = dang_deg * math.pi/180
    return self.move_p_dang(px,py,dang)
  def move_from_ang1(self, ang1, dang):
    # r must be preset.
    px = self.x0 + self.r * math.cos(ang1)
    py = self.y0 + self.r * math.sin(ang1)
    return self.move_p_dang(px, py, dang)
  def move_from_ang1_deg(self, ang1_deg, dang_deg):
    # r must be preset.
    ang1 = ang1_deg * math.pi/180
    dang = dang_deg * math.pi/180
    return self.move_from_ang1(ang1, dang)


