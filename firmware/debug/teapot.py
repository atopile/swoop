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
ser = serial.Serial('/dev/tty.usbmodem1101', 38400)


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

    try:
        w = float(line.split('w')[1])
        nx = float(line.split('a')[1])
        ny = float(line.split('b')[1])
        nz = float(line.split('c')[1])
        return quaternion.quaternion(w, nx, ny, nz)
    # If save command is sent
    except:
        zero_quaternion = sensor_off_quat.inverse()
        return quaternion.quaternion(1, 0, 0, 0)


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


if __name__ == '__main__':
    main()