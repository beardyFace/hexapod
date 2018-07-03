import maestro
import math

# servo = maestro.Controller()
# servo.setAccel(0,4)      #set servo 0 acceleration to 4
# servo.close()

#####################################################################################################################################

class Hexapod():
	def __init__(self):
		# self.servo = maestro.Controller()
		self.hex_leng = 60
		self.Tibia = 115.0
		self.Femur = 70.0
		self.Coxa  = 56.0

		self.leg_position_offsets = []
		for leg in range(0, 6):
			leg_offset = {}
			leg_offset['x'] = 0
			leg_offset['y'] = 0
			leg_offset['t'] = 0
			self.leg_position_offsets.append(leg_offset)

		self.servo_angles = []
		for leg in range(0, 6):
			servo_angle = {}
			servo_angle['t'] = 0
			servo_angle['f'] = 0
			servo_angle['c'] = 0
			self.servo_angles.append(servo_angle)

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

		self.body_pose = {}
		self.initIKangles()
		self.setBody(0,0,0,0,0,0)

	def initIKangles(self):
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

		# Body offsets   
		self.leg_position_offsets[0]['x'] = self.hex_leng/2.0
		self.leg_position_offsets[0]['y'] = math.sqrt(self.hex_leng*self.hex_leng - self.hex_leng/2*self.hex_leng/2)
		self.leg_position_offsets[0]['t'] = 60.0

		self.leg_position_offsets[1]['x'] = self.hex_leng
		self.leg_position_offsets[1]['y'] = 0.0
		self.leg_position_offsets[1]['t'] = 0.0

		self.leg_position_offsets[2]['x'] = self.hex_leng/2.0
		self.leg_position_offsets[2]['y'] = -math.sqrt(self.hex_leng*self.hex_leng - self.hex_leng/2*self.hex_leng/2)
		self.leg_position_offsets[2]['t'] = 300.0

		self.leg_position_offsets[3]['x'] = -self.hex_leng/2.0
		self.leg_position_offsets[3]['y'] = -math.sqrt(self.hex_leng*self.hex_leng - self.hex_leng/2*self.hex_leng/2)
		self.leg_position_offsets[3]['t'] = 240.0

		self.leg_position_offsets[4]['x'] = -self.hex_leng
		self.leg_position_offsets[4]['y'] = 0.0
		self.leg_position_offsets[4]['t'] = 180.0

		self.leg_position_offsets[5]['x'] = -self.hex_leng/2
		self.leg_position_offsets[5]['y'] = math.sqrt(self.hex_leng*self.hex_leng - self.hex_leng/2*self.hex_leng/2)
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

		# for leg in range(0, 6):
		# 	print("####################################")
		# 	print("L: "+str(leg))
		# 	print("leg_offset")
		# 	print("X: "+str(self.leg_position_offsets[leg]['x']))
		# 	print("Y: "+str(self.leg_position_offsets[leg]['y']))
		# 	print("T: "+str(self.leg_position_offsets[leg]['t']))
		# 	print("ik_position")
		# 	print("X: "+str(self.ik_position[leg]['x']))
		# 	print("Y: "+str(self.ik_position[leg]['y']))
		# 	print("Z: "+str(self.ik_position[leg]['z']))
	  
	def bodyIK(self, leg, dx, dy, dz, rx, ry, rz):
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

		self.legIK(leg, new_x, new_y, new_z);

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

	def legIK(self, leg, x, y, z):
		x = self.removeNZ(x)
		y = self.removeNZ(y)
		z = self.removeNZ(z)

		CFD = math.sqrt(x*x + y*y)

		# //https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
		g = math.atan2(y, x)

		L  = math.sqrt(z*z + (CFD - self.Coxa)*(CFD - self.Coxa))

		a1 = math.acos(z/L);
		a2 = math.acos((self.Tibia*self.Tibia - self.Femur*self.Femur - L*L) / (-2 * self.Femur * L))
		a = a1 + a2;

		b  = math.acos((L*L - self.Tibia*self.Tibia - self.Femur*self.Femur) / (-2 * self.Tibia * self.Femur))

		Coxa_angle  = g * 180.0/math.pi 
		Femur_angle = a * 180.0/math.pi
		Tibia_angle = b * 180.0/math.pi

		if leg < 3:
			self.servo_angles[leg]['c'] = self.boundTheta(90 + Coxa_angle - self.leg_position_offsets[leg]['t'])#theta
		else:
			self.servo_angles[leg]['c'] = 180-self.boundTheta(90 + Coxa_angle - self.leg_position_offsets[leg]['t'])#theta

		self.servo_angles[leg]['f'] = 180-Femur_angle;
		self.servo_angles[leg]['t'] = Tibia_angle;

		print("####################################")
		print("L: "+str(leg))
		print("C: "+str(self.servo_angles[leg]['c']))
		print("F: "+str(self.servo_angles[leg]['f']))
		print("T: "+str(self.servo_angles[leg]['t']))		
		
	def translate(self, value, leftMin, leftMax, rightMin, rightMax):
	    # Figure out how 'wide' each range is
	    leftSpan = leftMax - leftMin
	    rightSpan = rightMax - rightMin

	    # Convert the left range into a 0-1 range (float)
	    valueScaled = float(value - leftMin) / float(leftSpan)

	    # Convert the 0-1 range into a value in the right range.
	    return rightMin + (valueScaled * rightSpan)

	def moveServo(self, servo, low, high, target):
		target = self.translate(target, low, high, 4000, 8000)
		# self.servo.setTarget(servo, target)

	def updateAngles(self):
		servo = -1;
	  	for leg in range(0, 6):
			servo+=1
			low = self.ik_angles_limits[0]['l']
			high = self.ik_angles_limits[0]['h']
			self.moveServo(servo, low, high, self.servo_angles[leg]['c'])

			servo+=1
			low = self.ik_angles_limits[1]['l']
			high = self.ik_angles_limits[1]['h']
			self.moveServo(servo, low, high, self.servo_angles[leg]['f'])

			servo+=1
			low = self.ik_angles_limits[2]['l']
			high = self.ik_angles_limits[2]['h']
			self.moveServo(servo, low, high, self.servo_angles[leg]['t'])

	def updateBodyPose(self):
		for leg in range(0, 6):
			self.bodyIK(leg, self.body_pose['x'], self.body_pose['y'], self.body_pose['z'], self.body_pose['rx'], self.body_pose['ry'], self.body_pose['rz'])
		self.updateAngles()

	def setBody(self, x, y, z, rx, ry, rz):
		self.body_pose['x'] = x
		self.body_pose['y'] = y
		self.body_pose['z'] = z

		self.body_pose['rx'] = rx
		self.body_pose['ry'] = ry
		self.body_pose['rz'] = rz

		self.updateBodyPose()

	def moveBody(self, dx, dy, dz, drx, dry, drz):
		self.body_pose['x'] += dx
		self.body_pose['y'] += dy
		self.body_pose['z'] += dz

		self.body_pose['rx'] += drx
		self.body_pose['ry'] += dry
		self.body_pose['rz'] += drz

		self.updateBodyPose()


