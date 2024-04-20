"""
Utility to represent the orientation of the IMU in 3D using Pygame and PyOpenGL.
Heavily inspired by this repository: https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation
"""

import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

import numpy as np
import quaternion

import socket

# Navigate to https://teleplot.fr and copy the port below
copied_port = 25335
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

SMALL_EPSILON = 0.000001

useSerial = True # set true for using serial for data transmission, false for wifi
useQuat = True   # set true for using quaternions, false for using y,p,r angles

input_quaternion = quaternion.quaternion(1, 0, 0, 0)
sensor_off_quat = quaternion.quaternion(1, 0, 0, 0)
zero_quaternion = quaternion.quaternion(1, 0, 0, 0)
compute_quaternion = quaternion.quaternion(1, 0, 0, 0)

sensor_offset_quaternion = quaternion.quaternion( 0.5896463, -0.3902784, 0.5896463, -0.3902784 )
# This offset quaternion was obtained through this site:
# https://www.andre-gaschler.com/rotationconverter/
# Using the Euler angles of multiple axis rotations (degrees) (XYZ order)
# x = -90deg; y = 90deg; z = 23deg; #TODO: confirm the 23 deg
# Result: Quaternion [x, y, z, w] [ -0.3902784, 0.5896463, -0.3902784, 0.5896463 ]


import serial
ser = serial.Serial('/dev/tty.usbmodem101', 38400)


def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("PyTeapot IMU orientation visualization")
    resizewin(640, 480)
    init()

    global input_quaternion
    global sensor_off_quat

    while 1:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        # We read the quaternion coming from the remote
        input_quaternion = read_data()
        # We realign the sensor in line with the remote
        sensor_off_quat = sensor_offset_quaternion * input_quaternion * sensor_offset_quaternion.conjugate()
        # We zero the quaternion based on the control position (set through button press)
        compute_quaternion = zero_quaternion * sensor_off_quat
        # We draw on screen
        draw(compute_quaternion)

        yaw, pitch, roll = quaternionToControlAngles(compute_quaternion)

        sock.sendto(f"3D|my_super_cube:S:cube:W:20:D:10:H:1:C:blue:Q:{compute_quaternion.x}:{compute_quaternion.y}:{compute_quaternion.z}:{compute_quaternion.w}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"yaw:{yaw}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"pitch:{pitch}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"roll:{roll}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"w:{compute_quaternion.w}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"x:{compute_quaternion.x}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"y:{compute_quaternion.y}".encode(), ("teleplot.fr", copied_port))
        sock.sendto(f"z:{compute_quaternion.z}".encode(), ("teleplot.fr", copied_port))

        pygame.display.flip()

    ser.close()

def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)



def read_data():
    global zero_quaternion
    global sensor_off_quat

    ser.reset_input_buffer()
    line = ser.readline().decode('UTF-8').replace('\n', '')

    quat = {'w': 0, 'x': 0, 'y': 0, 'z': 0}
    while True:
        line = ser.readline().decode('utf-8').strip()
        if line.lower() == 'save':
            zero_quaternion = sensor_off_quat.inverse()
        elif line.startswith('>w:'):
            quat['w'] = float(line.split(':')[1])
        elif line.startswith('>x:'):
            quat['x'] = float(line.split(':')[1])
        elif line.startswith('>y:'):
            quat['y'] = float(line.split(':')[1])
        elif line.startswith('>z:'):
            quat['z'] = float(line.split(':')[1])
        # Assuming the 'z' component is the last to be sent before a new set begins
        if quat['w'] and quat['x'] and quat['y'] and quat['z']:
            return quaternion.quaternion(quat['w'], quat['x'], quat['y'], quat['z'])


def draw(q):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "PyTeapot", 18)
    drawText((-2.6, 1.6, 2), "Module to visualize quaternion or Euler angles data", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    [yaw, pitch , roll] = quat_to_ypr(q)
    q.w = max(min(q.w, 1.0), -1.0)
    angle = 2 * math.acos(q.w) * 180.00 / math.pi
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(angle, -1 * q.x, q.z, q.y)


    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def quat_to_ypr(q):
    q_n = q.normalized()
    yaw   = math.atan2(2.0 * (q_n.x * q_n.y + q_n.w * q_n.z), q_n.w * q_n.w + q_n.x * q_n.x - q_n.y * q_n.y - q_n.z * q_n.z)
    pitch = -math.asin(2.0 * (q_n.x * q_n.z - q_n.w * q_n.y))
    roll  = math.atan2(2.0 * (q_n.w * q_n.x + q_n.y * q_n.z), q_n.w * q_n.w - q_n.x * q_n.x - q_n.y * q_n.y + q_n.z * q_n.z)
    pitch *= 180.0 / math.pi
    yaw   *= 180.0 / math.pi
    yaw   -= -0.13  # Declination at Chandrapur, Maharashtra is - 0 degress 13 min
    roll  *= 180.0 / math.pi
    return [yaw, pitch, roll]


def quaternionToControlAngles(q):
    """
    The extraction of the YPR angles does not correspond to any of the standard Euler angles sequences.
    The reason for this is to provide the best possible feeling to the pilot, preventing the hand from
    tilting too far in the corners and ensuring symmetry for the pitch and roll angles.
    """
    q0 = q.w
    q1 = q.z
    q2 = q.x
    q3 = q.y

    InvSqrt_q0q0_q1q1 = 1/math.sqrt(q0 * q0 + q1 * q1)

    # float r11, r12;

    # float xY = 0;
    # float xP = 0;
    # float xR = 0;

    # Yaw extraction

    q0_tors = q0 * InvSqrt_q0q0_q1q1
    q1_tors = q1 * InvSqrt_q0q0_q1q1

    fAngle = 2 * math.acos( q0_tors )
    if( 1 - ( q0_tors * q0_tors ) < SMALL_EPSILON ):
        xY = q1_tors
    else:
        scale = math.sqrt( 1 - q0_tors * q0_tors )
        xY = q1_tors / scale

    xY = xY * fAngle

    # Pitch extraction
    # Using euler angles order zyx and using z as pitch
    r11 = 2*(q1*q2 + q0*q3)
    r12 = q0*q0 + q1*q1 - q2*q2 - q3*q3
    xP = math.atan2(r11, r12)

    # Roll extraction
    # Using euler angles order yzx and using y as roll
    r11 = -2*(q1*q3 - q0*q2)
    # //r12 = q0*q0 + q1*q1 - q2*q2 - q3*q3; // already calculated above
    xR = math.atan2(r11, r12)

    yaw = math.degrees( xY )
    pitch = math.degrees( xR )
    roll = math.degrees( xP )

    return (yaw, pitch, roll)



if __name__ == '__main__':
    main()