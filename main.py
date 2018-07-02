#!/usr/bin/python
from time import sleep
import maestro
import math

# servo = maestro.Controller()
# servo.setAccel(0,4)      #set servo 0 acceleration to 4
# servo.close()

#####################################################################################################################################

class Hexapod():
	def __init__(self):
		# servo = maestro.Controller()
		self.Tibia = 11.5
		self.Femur = 7.0
		self.Coxa  = 5.6

		self.leg_position_offsets = []
		for leg in range(0, 6):
			leg_offset = {}
			leg_offset['x'] = 0
			leg_offset['y'] = 0
			leg_offset['t'] = 0
			self.leg_position_offsets.append(leg_offset)

		self.angles = []
		for leg in range(0, 6):
			servo_angle = {}
			servo_angle['t'] = 0
			servo_angle['f'] = 0
			servo_angle['c'] = 0
			self.angles.append(servo_angle)

		self.ik_position = []
		for leg in range(0, 6):
			leg_pose = {}
			leg_pose['x'] = 0
			leg_pose['y'] = 0
			leg_pose['z'] = 0
			self.ik_position.append(leg_pose)

		self.ik_adjustments = []
		for leg in range(0, 6):
			adjust = {}
			adjust['x'] = 0
			adjust['y'] = 0
			adjust['z'] = 0
			self.ik_adjustments.append(adjust)

		self.ik_angles_limits = []
		for leg in range(0, 3):
			limit = {}
			limit['h'] = 135
			limit['l'] = 45
			self.ik_angles_limits.append(limit)

		self.ik_angles()

	def ik_angles(self):
	  #Coxa
	  coxa_lowest = 45
	  coxa_highest = 135
	  self.ik_angles_limits[0]['l'] = coxa_lowest
	  self.ik_angles_limits[0]['h'] = coxa_highest
	  
	  #Femur
	  femur_lowest = 45
	  femur_highest = 135
	  self.ik_angles_limits[1]['l'] = femur_lowest;
	  self.ik_angles_limits[1]['h'] = femur_highest;
	  
	  #6000 is 90
	  tibia_lowest = 45;
	  tibia_highest = 135;
	  self.ik_angles_limits[2]['l'] = tibia_lowest;
	  self.ik_angles_limits[2]['h'] = tibia_highest;
	  
	  # Calculate hexagon points poses
	  # Body offsets   
	  self.leg_position_offsets[0]['x'] = 0.0
	  self.leg_position_offsets[0]['y'] = 0.0
	  self.leg_position_offsets[0]['t'] = 60.0
	  
	  self.leg_position_offsets[1]['x'] = 0.0
	  self.leg_position_offsets[1]['y'] = 0.0
	  self.leg_position_offsets[1]['t'] = 0.0
	  
	  self.leg_position_offsets[2]['x'] = 0.0
	  self.leg_position_offsets[2]['y'] = 0.0
	  self.leg_position_offsets[2]['t'] = 300.0
	  
	  self.leg_position_offsets[3]['x'] = 0.0
	  self.leg_position_offsets[3]['y'] = 0.0
	  self.leg_position_offsets[3]['t'] = 240.0
	  
	  self.leg_position_offsets[4]['x'] = 0.0
	  self.leg_position_offsets[4]['y'] = 0.0
	  self.leg_position_offsets[4]['t'] = 180.0
	  
	  self.leg_position_offsets[5]['x'] = 0.0
	  self.leg_position_offsets[5]['y'] = 0.0
	  self.leg_position_offsets[5]['t'] = 120.0

	  x = self.Coxa + self.Femur;
	  y = self.Coxa + self.Femur;
	  z = self.Tibia;

	  #Init Feet position
	  for leg in range(0, 6):
	    theta = self.leg_position_offsets[leg]['t'] * math.pi / 180.0
	    
	    self.ik_position[leg]['x'] = x * math.cos(theta)
	    self.ik_position[leg]['y'] = y * math.sin(theta)
	    self.ik_position[leg]['z'] = z

	def moveServo(self, ServoChannel, target):
		leg = ServoChannel / 3;
		servo = ServoChannel % 3;
		target += self.ik_adjustments[leg][servo]

		target = map(target, low, high, 4000, 8000);
		# self.servo.setTarget(ServoChannel,target)
	  
	def body_ik(self, leg, dx, dy, dz, rx, ry, rz):
	  rx = rx * math.pi / 180
	  ry = ry * math.pi / 180
	  rz = rz * math.pi / 180

	  total_x = self.ik_position[leg]['x'] + dx + self.leg_position_offsets[leg]['x']
	  total_y = self.ik_position[leg]['y'] + dy + self.leg_position_offsets[leg]['y']
	  total_z = self.ik_position[leg]['z']
	  
	  srx = math.sin(rx)
	  crx = math.cos(rx)
	  sry = math.sin(ry)
	  cry = math.cos(ry)
	  
	  srz = math.sin(rz + rz)
	  crz = math.cos(rz + rz)
	  
	  body_ik_x = total_x * cry * crz - total_y * cry * srz + total_z * sry - total_x
	  body_ik_y = (total_x * crx * srz + total_x * crz * sry * srx + total_y * crz * crx - total_y * srz * sry * srx - total_z * cry * srx) - total_y
	  body_ik_z = (total_x * srz * srx - total_x * crz * crx * sry + total_y * crz * srx + total_y * crx * srz * sry + total_z * cry * crx) - total_z

	  new_x = body_ik_x + dx + self.ik_position[leg]['x'];
	  new_y = body_ik_y + dy + self.ik_position[leg]['y'];
	  new_z = body_ik_z + dz + self.ik_position[leg]['z'];
	  
	  self.leg_ik(leg, new_x, new_y, new_z);

	def boundTheta(self, t):
	  if t < 0:
	    return 360 + t
	  elif t >= 360:
	    return t - 360
	  return t

	def removeNZ(self, v):
	  if v == -0:
	    return 0
	  return v

	def leg_ik(self, leg, x, y, z):
	  x = self.removeNZ(x)
	  y = self.removeNZ(y)
	  z = self.removeNZ(z)
	  
	  CFD = math.sqrt(x*x + y*y)
	 
	  # //https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
	  g = math.atan2(y, x);
	  
	  L  = math.sqrt(z*z + (CFD - self.Coxa)*(CFD - self.Coxa));

	  a1 = math.acos(z/L);
	  a2 = math.acos((self.Tibia*self.Tibia - self.Femur*self.Femur - L*L) / (-2 * self.Femur * L));
	  a = a1 + a2;

	  b  = math.acos((L*L - self.Tibia*self.Tibia - self.Femur*self.Femur) / (-2 * self.Tibia * self.Femur));

	  Coxa_angle  = g * 180.0/math.pi 
	  Femur_angle = a * 180.0/math.pi
	  Tibia_angle = b * 180.0/math.pi

	  if leg < 3:
	    self.angles[leg]['x'] = self.boundTheta(90 + Coxa_angle - self.leg_position_offsets[leg]['t']);#theta
	  else:
	    self.angles[leg]['x'] = 180-self.boundTheta(90 + Coxa_angle - self.leg_position_offsets[leg]['t'])#theta

	  self.angles[leg]['y'] = 180-Femur_angle;
	  self.angles[leg]['z'] = Tibia_angle;

	  print(self.angles[leg]['x'])
	  print(self.angles[leg]['y'])
	  print(self.angles[leg]['z'])

	def update_angles(self):
	  servo = 0;
	  for i in range(0, 6):#(int leg = 0; leg < 6; leg++)
	    self.moveServo(servo, self.angles[leg]['x'])
	    servo+=1
	    self.moveServo(servo, self.angles[leg]['y'])
	    servo+=1
	    self.moveServo(servo, self.angles[leg]['z'])
	    servo+=1

	def moveBody(self, x, y, z, rx, ry, rz):
		for leg in range(0, 6):
			self.body_ik(leg, x, y, z, rx, ry, rz)

hexapod = Hexapod()