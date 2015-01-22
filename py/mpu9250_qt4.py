#!/usr/bin/env python
# -*- coding: ms949 -*-
#
#     pyqt_example.py 
#
# sudo apt-get install python-qt4 python-opengl python-qt4-gl python-numpy
#

import sys
import math
import time 
import serial 
import traceback

from PyQt4 import QtCore, QtGui, QtOpenGL
import numpy as np

try:
    from OpenGL.GL import *
    from OpenGL.GLU import *
    from OpenGL.GLUT import *
    
except ImportError:
    app = QtGui.QApplication(sys.argv)
    QtGui.QMessageBox.critical(None, "OpenGL hellogl",
            "PyOpenGL must be installed to run this example.")
    sys.exit(1)

#
# QT resource file  
# 
# pyrcc4 myahrs_model_texture.qrc -o myahrs_model_texture.py
import myahrs_model_texture


#
# helper 
#
def glut_print( x,  y,  font,  text, r,  g , b , a):

    blending = False 
    if glIsEnabled(GL_BLEND) :
        blending = True

    #glEnable(GL_BLEND)
    glColor3f(1,1,1)
    glRasterPos2f(x,y)
    for ch in text :
        glutBitmapCharacter( font , ctypes.c_int( ord(ch) ) )


    if not blending :
        glDisable(GL_BLEND) 


class WidgetAhrs3dModel(QtOpenGL.QGLWidget):
    #
    # signal 
    #
    xRotationChanged = QtCore.pyqtSignal(int)
    yRotationChanged = QtCore.pyqtSignal(int)
    zRotationChanged = QtCore.pyqtSignal(int)
 
    def __init__(self, parent=None):
        super(WidgetAhrs3dModel, self).__init__(parent)
        
        #
        # Model의 Roll, Pitch, Yaw 
        #
        self.xRot = 0 # roll
        self.yRot = 0 # pitch 
        self.zRot = 0 # yaw 
        
        # 
        # 시점을 이동하기 위해 구면좌표계를 사용해보자. 
        # 
        self.R = 6
        self.THETA_DEG = -110
        self.PHI_DEG = 0
        
        self.R_MIN = 4.5
        self.R_MAX = 15.0
        
        self.THETA_MIN = -180.0
        self.THETA_MAX = -90.0
        
        self.lastPos = QtCore.QPoint()
        self.clearColor = QtGui.QColor()

    def minimumSizeHint(self):
        return QtCore.QSize(50, 50)

    def sizeHint(self):
        return QtCore.QSize(400, 400)

    def setXRotation(self, angle):
        if angle != self.xRot:
            self.xRot = angle
            self.xRotationChanged.emit(angle)
            self.updateGL()
            
    def setYRotation(self, angle):
        if angle != self.yRot:
            self.yRot = angle
            self.yRotationChanged.emit(angle)
            self.updateGL()
            
    def setZRotation(self, angle):
        if angle != self.zRot:
            self.zRot = angle
            self.zRotationChanged.emit(angle)
            self.updateGL()

    def setEulerAngle(self, euler_angle):
        roll, pitch, yaw = euler_angle
        self.xRot = roll
        self.yRot = -pitch
        self.zRot = -yaw
        self.updateGL()            

    def initializeGL(self): 
        glutInit(sys.argv)        
        
        glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH)
        
        #
        # load textrues 
        #
        
        textures = []
        
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/ahrs_texture_+z.png"))) 
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/ahrs_texture_-z.png"))) 
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/ahrs_texture_+y.png"))) 
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/ahrs_texture_-y.png"))) 
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/ahrs_texture_+x.png"))) 
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/ahrs_texture_-x.png"))) 
        textures.append(self.bindTexture(QtGui.QPixmap(":/images/chess_board.png")))    
        
        self.textures = textures      
            
        quadratic = gluNewQuadric() 
        self.quadratic = quadratic
        
        gluQuadricNormals(quadratic, GLU_SMOOTH)   # Create Smooth Normals (NEW) 
        gluQuadricTexture(quadratic, GL_TRUE)      # Create Texture Coords (NEW)
         
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_LINE_SMOOTH)
        
        glClearColor(0.0, 0.0, 0.0, 0.0)     # This Will Clear The Background Color To Black
        glClearDepth(1.0)                    # Enables Clearing Of The Depth Buffer
        glDepthFunc(GL_LESS)                 # The Type Of Depth Test To Do
        glEnable(GL_DEPTH_TEST)              # Enables Depth Testing
        glShadeModel(GL_SMOOTH)              # Enables Smooth Color Shading
        
        
        glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP)
        glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP)
        

    def paintGL(self):
        self.qglClearColor(self.clearColor)
                 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)    # Clear The Screen And The Depth Buffer
        glLoadIdentity()                    # Reset The View
        
        #
        # 시점변환 
        #  - 센서 좌표와 일치하도록 -z가 화면의 위를 향하도록 배치한다. 
        #
        if(True):
            phi = np.pi/180.0*self.PHI_DEG
            theta = np.pi/180.0*self.THETA_DEG
            
            eye_x = self.R * np.sin(theta) * np.cos(phi)
            eye_y = self.R * np.sin(theta) * np.sin(phi)
            eye_z = self.R * np.cos(theta)
        else:
            # debug 
            eye_x, eye_y, eye_z = -5, -5, -3
                        
        gluLookAt(eye_x, eye_y, eye_z,   # eye
                   0, 0, 0,     # center
                   0, 0, -1)    # up     

        #
        # TEXT  
        #        
        #self.DrawText()
        
        #
        # 배경 (체스판) 
        #
        self.DrawChessBoard()   

        #
        # 센서 모델 
        #
        self.DrawSensor()
                
        
    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        
#         side = min(width, height)
#         glViewport((width - side) / 2, (height - side) / 2, side, side)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()                    # Reset The Projection Matrix
                                            # Calculate The Aspect Ratio Of The Window
        gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
    
        glMatrixMode(GL_MODELVIEW)       
                

    def mousePressEvent(self, event):
        self.lastPos = event.pos()
        

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
            
        if(False): # ORG 
            if event.buttons() & QtCore.Qt.LeftButton:
                self.setXRotation(self.xRot + 8 * dy)
                self.setYRotation(self.yRot + 8 * dx)
            elif event.buttons() & QtCore.Qt.RightButton:
                self.setXRotation(self.xRot + 8 * dy)
                self.setZRotation(self.zRot + 8 * dx)

        self.lastPos = event.pos()
        
        #print 'mouseMoveEvent %d, %d'%(dx, dy)
        
        self.PHI_DEG += dx 
        
        self.THETA_DEG -= dy
        if(self.THETA_DEG < self.THETA_MIN):
            self.THETA_DEG = self.THETA_MIN
        if(self.THETA_DEG > self.THETA_MAX):
            self.THETA_DEG = self.THETA_MAX
            
        self.updateGL()
        
        #print 'RTP = ', self.R, self.THETA_DEG, self.PHI_DEG

    def wheelEvent(self, event):
        super(WidgetAhrs3dModel, self).wheelEvent(event) 
        
        # event.delta() = +- 120 
        # 맥/윈도우 동일 
        self.R += event.delta()/400.0
        
        if(self.R < self.R_MIN):
            self.R = self.R_MIN
        if(self.R > self.R_MAX):
            self.R = self.R_MAX        
        
        self.updateGL()
                
        #print "wheelEvent", event.delta() 
        #print 'RTP = ', self.R, self.THETA_DEG, self.PHI_DEG
    
    def DrawText(self):  
        
        string = "HELLO"
        
        for c in string:  
            glutBitmapCharacter(GLUT_BITMAP_8_BY_13, ord(c)) 

    def DrawChessBoard(self):
        glPushMatrix()
        
        glTranslatef(0.0, 0.0, 2.0) 
        
        glBindTexture(GL_TEXTURE_2D, self.textures[6]) # chessboard 
        
        glBegin(GL_QUADS) 
        
        glNormal3f( 0.0, 0.5, 0.0)

        glTexCoord2f(0.0, 0.0); glVertex3f(-5.0, -5.0,  0.0)    # Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0, 0.0); glVertex3f( 5.0, -5.0,  0.0)    # Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0, 1.0); glVertex3f( 5.0,  5.0,  0.0)    # Top Right Of The Texture and Quad
        glTexCoord2f(0.0, 1.0); glVertex3f(-5.0,  5.0,  0.0)    # Top Left Of The Texture and Quad
        
        glEnd();                # Done Drawing The Cube
            
        glPopMatrix()

    
    def DrawCube(self):
        glPushMatrix()
        
        scale_x = 1.0
        scale_y = 0.8
        scale_z = 0.3
        glScalef(scale_x, scale_y, scale_z)
        
        # Note there does not seem to be support for this call.
        #glBindTexture(GL_TEXTURE_2D,texture)    # Rotate The Pyramid On It's Y Axis
    
        textures = self.textures
        glBindTexture(GL_TEXTURE_2D, textures[0]) # side1
        
        glBegin(GL_QUADS)                 
        
        # +Z Face (note that the texture's corners have to match the quad's corners)
        glNormal3f( 0.0, 0.0, 0.5)
        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0,  1.0)    # Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0,  1.0)    # Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0,  1.0)    # Top Right Of The Texture and Quad
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0,  1.0)    # Top Left Of The Texture and Quad
        
        glEnd();                 
        
        ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ######
         
        glBindTexture(GL_TEXTURE_2D, textures[1]) # side2
        
        glBegin(GL_QUADS) 
        
        # -Z Face
        glNormal3f( 0.0, 0.0, -0.5)
        glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0, -1.0)    # Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0, -1.0)    # Top Right Of The Texture and Quad
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0, -1.0)    # Top Left Of The Texture and Quad
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0, -1.0)    # Bottom Left Of The Texture and Quad
        
        glEnd();                # Done Drawing The Cube
        
        ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ######
        
        glBindTexture(GL_TEXTURE_2D, textures[2]) # side3
        
        glBegin(GL_QUADS) 
        
        # +Y Face
        glNormal3f( 0.0, 0.5, 0.0)
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0)    # Top Left Of The Texture and Quad
        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0,  1.0,  1.0)    # Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0,  1.0,  1.0)    # Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0)    # Top Right Of The Texture and Quad
        
        glEnd();                # Done Drawing The Cube
        
        ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ######
        
        glBindTexture(GL_TEXTURE_2D, textures[3]) # side4
        
        glBegin(GL_QUADS) 
        
        # -Y Face
        glNormal3f( 0.0, -0.5, 0.0)       
        glTexCoord2f(1.0, 1.0); glVertex3f(-1.0, -1.0, -1.0)    # Top Right Of The Texture and Quad
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0, -1.0, -1.0)    # Top Left Of The Texture and Quad
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0)    # Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0)    # Bottom Right Of The Texture and Quad
        
        glEnd();                # Done Drawing The Cube
        
        ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ######
        
        glBindTexture(GL_TEXTURE_2D, textures[4]) # side5
        
        glBegin(GL_QUADS) 
        
        # +X Face
        glNormal3f( 0.5, 0.0, 0.0)
        glTexCoord2f(1.0, 0.0); glVertex3f( 1.0, -1.0, -1.0)    # Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0, 1.0); glVertex3f( 1.0,  1.0, -1.0)    # Top Right Of The Texture and Quad
        glTexCoord2f(0.0, 1.0); glVertex3f( 1.0,  1.0,  1.0)    # Top Left Of The Texture and Quad
        glTexCoord2f(0.0, 0.0); glVertex3f( 1.0, -1.0,  1.0)    # Bottom Left Of The Texture and Quad
        
        glEnd();                # Done Drawing The Cube
        
        ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ###### ######
        
        glBindTexture(GL_TEXTURE_2D, textures[5]) # side6
        
        glBegin(GL_QUADS) 
        
        # -X Face
        glNormal3f( -0.5, 0.0, 0.0)
        glTexCoord2f(0.0, 0.0); glVertex3f(-1.0, -1.0, -1.0)    # Bottom Left Of The Texture and Quad
        glTexCoord2f(1.0, 0.0); glVertex3f(-1.0, -1.0,  1.0)    # Bottom Right Of The Texture and Quad
        glTexCoord2f(1.0, 1.0); glVertex3f(-1.0,  1.0,  1.0)    # Top Right Of The Texture and Quad
        glTexCoord2f(0.0, 1.0); glVertex3f(-1.0,  1.0, -1.0)    # Top Left Of The Texture and Quad
        
        glEnd();                # Done Drawing The Cube
    
        glPopMatrix()
    
    def DrawSensor(self):
        glPushMatrix()

        glRotatef(self.zRot,0.0,0.0,1.0)            # Rotate The Cube On It's Z Axis
        glRotatef(self.yRot,0.0,1.0,0.0)            # Rotate The Cube On It's Y Axis
        glRotatef(self.xRot,1.0,0.0,0.0)            # Rotate The Cube On It's X Axis
        
        #glEnable(GL_TEXTURE_GEN_S)
        #glEnable(GL_TEXTURE_GEN_T)
        
        self.DrawCube()
        
        glPopMatrix()
        #glDisable(GL_TEXTURE_GEN_S)
        #glDisable(GL_TEXTURE_GEN_T)



##################################################################################################################################
#
#    MAIN  
#
##################################################################################################################################

def compute_madgiwick(data):
  """
  Implementation of madgwik filter.

        void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
        {
            float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
            float norm;
            float hx, hy, _2bx, _2bz;
            float s1, s2, s3, s4;
            float qDot1, qDot2, qDot3, qDot4;

            // Auxiliary variables to avoid repeated arithmetic
            float _2q1mx;
            float _2q1my;
            float _2q1mz;
            float _2q2mx;
            float _4bx;
            float _4bz;
            float _2q1 = 2.0f * q1;
            float _2q2 = 2.0f * q2;
            float _2q3 = 2.0f * q3;
            float _2q4 = 2.0f * q4;
            float _2q1q3 = 2.0f * q1 * q3;
            float _2q3q4 = 2.0f * q3 * q4;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q1q4 = q1 * q4;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q2q4 = q2 * q4;
            float q3q3 = q3 * q3;
            float q3q4 = q3 * q4;
            float q4q4 = q4 * q4;

            // Normalise accelerometer measurement
            norm = sqrt(ax * ax + ay * ay + az * az);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            ax *= norm;
            ay *= norm;
            az *= norm;

            // Normalise magnetometer measurement
            norm = sqrt(mx * mx + my * my + mz * mz);
            if (norm == 0.0f) return; // handle NaN
            norm = 1.0f/norm;
            mx *= norm;
            my *= norm;
            mz *= norm;

            // Reference direction of Earth's magnetic field
            _2q1mx = 2.0f * q1 * mx;
            _2q1my = 2.0f * q1 * my;
            _2q1mz = 2.0f * q1 * mz;
            _2q2mx = 2.0f * q2 * mx;
            hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
            hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
            _2bx = sqrt(hx * hx + hy * hy);
            _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            // Gradient decent algorithm corrective step
            s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
            norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
            norm = 1.0f/norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
            s4 *= norm;

            // Compute rate of change of quaternion
            qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
            qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
            qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
            qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

            // Integrate to yield quaternion
            q1 += qDot1 * deltat;
            q2 += qDot2 * deltat;
            q3 += qDot3 * deltat;
            q4 += qDot4 * deltat;
            norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
            norm = 1.0f/norm;
            q[0] = q1 * norm;
            q[1] = q2 * norm;
            q[2] = q3 * norm;
            q[3] = q4 * norm;

        }
  """ 


def parse_data_mpu9250(line):
    # raw:,39,0.42,-0.31,-26.51,-0.0049,-0.0038,-1.0103,-0.0101,0.0014,-0.4001,51.9000,26.7000,11.7000,41.5*1F
    if not line.startswith("data:"):
      return None
    line = line.strip() # just in case
    line = line.replace("data:","")
    print line
    
    fields = [x.strip() for x in line.split(',')]
    if len(fields) < 9:
      return None
    
    roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, q0,q1,q2,q3,temperature = (float(x) for x in fields)
    return (roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature)


class Window(QtGui.QWidget):
    def __init__(self, serial_device):
        super(Window, self).__init__()

        self.ahrs_model = WidgetAhrs3dModel()

        mainLayout = QtGui.QHBoxLayout()
        mainLayout.addWidget(self.ahrs_model) 
        self.setLayout(mainLayout)  

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(10) # msec (was 20)
        self.timer.timeout.connect(self._handler_timer)   
        
        self.setWindowTitle("AHRS MODEL")
        self.lastitems = []
        
        #
        # Initialize myAhrs+ 
        #
        try:
            self.serial_port = serial.Serial(serial_device, 115200, timeout=1.0)
        except serial.serialutil.SerialException:
            print 'ERROR. Can not open serial port(%s)'%(serial_device)
            exit(0)     

        self.timer.start()        
    
    def closeEvent(self, event):
        self.timer.stop()  
        self.serial_port.close() 
    
    
    def _handler_timer(self):
        #
        # wait for data message 
        #  ~RPYIMU
        # print "calling timer"
        line = self.serial_port.readline().strip()
                 
        #
        # parse data message
        #
        items = parse_data_mpu9250(line)
        if items is None:
          items = self.lastitems  
        else:
          self.lastitems = items
         
        #
        # display output 
        #
        if(items):
            roll, pitch, yaw, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature = items
            self.ahrs_model.setEulerAngle((roll, pitch, yaw))   


if __name__ == '__main__':
    #
    # tty device name of myahrs+ 
    #
    if(len(sys.argv) < 2):
        serial_device = '/dev/ttyACM0'
    else : 
        serial_device = sys.argv[1]

    app = QtGui.QApplication(sys.argv)
    window = Window(serial_device)
    window.show()
    sys.exit(app.exec_())


