import maestro
import math
from time import sleep
import cv2
import numpy as np
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

        cv2.namedWindow('hex', cv2.WINDOW_NORMAL)
        cv2.moveWindow('hex', 100,600)
        cv2.resizeWindow('hex', 500, 500)

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

        self.leg_pose = []
        for leg in range(0, 6):
            leg_pose = {}
            leg_pose['x'] = 0
            leg_pose['y'] = 0
            leg_pose['z'] = 0
            self.leg_pose.append(leg_pose)

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

        self.draw_pose = []
        for leg in range(0, 6):
            adjust = {}
            adjust['x'] = 0
            adjust['y'] = 0
            adjust['z'] = 0
            self.draw_pose.append(adjust)

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

            self.leg_pose[leg]['x'] = x * math.cos(theta)
            self.leg_pose[leg]['y'] = y * math.sin(theta)
            self.leg_pose[leg]['z'] = z
      
    def bodyIK(self, leg, dx, dy, dz, rx, ry, rz):
        rx = rx * math.pi / 180
        ry = ry * math.pi / 180
        rz = rz * math.pi / 180

        total_x = self.leg_pose[leg]['x'] + dx + self.leg_position_offsets[leg]['x']
        total_y = self.leg_pose[leg]['y'] + dy + self.leg_position_offsets[leg]['y']
        total_z = self.leg_pose[leg]['z']

        srx = math.sin(rx)
        crx = math.cos(rx)
        sry = math.sin(ry)
        cry = math.cos(ry)

        srz = math.sin(rz + rz)
        crz = math.cos(rz + rz)

        body_ik_x = total_x * cry * crz - total_y * cry * srz + total_z * sry - total_x
        body_ik_y = (total_x * crx * srz + total_x * crz * sry * srx + total_y * crz * crx - total_y * srz * sry * srx - total_z * cry * srx) - total_y
        body_ik_z = (total_x * srz * srx - total_x * crz * crx * sry + total_y * crz * srx + total_y * crx * srz * sry + total_z * cry * crx) - total_z

        new_x = body_ik_x + dx + self.leg_pose[leg]['x']
        new_y = body_ik_y + dy + self.leg_pose[leg]['y']
        new_z = body_ik_z + dz + self.leg_pose[leg]['z']

        self.draw_pose[leg]['x'] = new_x
        self.draw_pose[leg]['y'] = new_y
        self.draw_pose[leg]['z'] = new_z

        self.legIK(leg, new_x, new_y, new_z)

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

        self.drawState()

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

    def moveLeg(self, leg, dx, dy, dz):
        self.leg_pose[leg]['x'] = self.leg_pose[leg]['x'] + dx
        self.leg_pose[leg]['y'] = self.leg_pose[leg]['y'] + dy
        self.leg_pose[leg]['z'] = self.leg_pose[leg]['z'] + dz

        self.legIK(leg, self.leg_pose[leg]['x'], self.leg_pose[leg]['y'], self.leg_pose[leg]['z'])
        self.updateAngles()

    def walk(self, distance, angle):
        angle = angle * math.pi/180
        
        dx = math.sin(angle) * distance
        dy = math.cos(angle) * distance
        dz = -20

        #Step one
        print(1)
        self.moveLeg(0, dx, dy, dz)#Up and forward
        self.moveLeg(2, dx, dy, dz)
        self.moveLeg(4, dx, dy, dz)
        sleep(1)
        print(2)
        self.moveLeg(1, -dx, -dy, 0)#Backward
        self.moveLeg(3, -dx, -dy, 0)
        self.moveLeg(5, -dx, -dy, 0)
        sleep(1)

        #Step two
        print(3)
        self.moveLeg(0, 0, 0, -dz)#Downward
        self.moveLeg(2, 0, 0, -dz)
        self.moveLeg(4, 0, 0, -dz)
        sleep(1)
        print(4)
        self.moveLeg(1, dx, dy, dz)#Up and forward
        self.moveLeg(3, dx, dy, dz)
        self.moveLeg(5, dx, dy, dz)
        sleep(1)

        #Step three
        print(5)
        self.moveLeg(0, -dx, -dy, 0)#Backward
        self.moveLeg(2, -dx, -dy, 0)
        self.moveLeg(4, -dx, -dy, 0)
        sleep(1)
        print(6)
        self.moveLeg(1, 0, 0, -dz)#Downward
        self.moveLeg(3, 0, 0, -dz)
        self.moveLeg(5, 0, 0, -dz)
        sleep(1)

    def drawState(self):
        w = 512
        h = 512
        img = np.zeros((w,h,3), np.uint8)
        
        draw_x = w/2
        draw_y = h/2
        for leg in range(0, 6):
            cv2.circle(img,(int(self.body_pose['x']+draw_x),int(self.body_pose['y']+draw_y)), 5, (255,0,255), -1)
            #foot pose
            x = int(self.leg_pose[leg]['x'] + draw_x)
            y = int(self.leg_pose[leg]['y'] + draw_y)
            cv2.circle(img,(x,y), 5, (0,0,255), -1)

            font                   = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (x,y)
            fontScale              = 1
            fontColor              = (255,255,255)
            lineType               = 2

            cv2.putText(img, str(leg), 
                bottomLeftCornerOfText, 
                font, 
                fontScale,
                fontColor,
                lineType)

            #coxa_pose
            x = int(self.leg_position_offsets[leg]['x'] + draw_x)
            y = int(self.leg_position_offsets[leg]['y'] + draw_y)
            cv2.circle(img,(x,y), 5, (0,255,0), -1)
            
        cv2.imshow('hex', img)
        cv2.waitKey(1)
