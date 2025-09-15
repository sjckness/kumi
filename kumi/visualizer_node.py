#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

import numpy as np
import math

class VisualizeIMU3DNode(Node):
    def __init__(self):
        super().__init__('visualize_imu_3d_node')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'imu_rpy',
            self.imu_callback,
            10
        )

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # PyGame/OpenGL setup
        pygame.init()
        self.screen = pygame.display.set_mode((800,600), DOUBLEBUF | OPENGL)
        pygame.display.set_caption("IMU 3D Visualization")

        glEnable(GL_DEPTH_TEST)
        glClearColor(0.2, 0.2, 0.2, 1.0)
        gluPerspective(45, (800/600), 0.1, 100.0)

        # Camera spherical coords
        self.cam_radius = 10.0
        self.cam_phi = math.radians(30)
        self.cam_theta = math.radians(45)
        self.last_mouse_pos = None

        self.timer = self.create_timer(0.02, self.render)

    def imu_callback(self, msg):
        if len(msg.data) == 3:
            self.roll, self.pitch, self.yaw = msg.data

    def draw_cube(self):
        glBegin(GL_QUADS)
        # 1 unit cube centered at origin
        vertices = [
            (1,1,1), (-1,1,1), (-1,-1,1), (1,-1,1),
            (1,1,-1), (-1,1,-1), (-1,-1,-1), (1,-1,-1)
        ]
        faces = [
            (0,1,2,3),(4,5,6,7),(0,4,7,3),
            (1,5,6,2),(0,1,5,4),(3,2,6,7)
        ]
        colors = [(1,0,0),(0,1,0),(0,0,1),(1,1,0),(1,0,1),(0,1,1)]
        for i, face in enumerate(faces):
            glColor3fv(colors[i])
            for vertex in face:
                glVertex3fv(vertices[vertex])
        glEnd()

    def draw_floor(self):
        glColor3f(0.5,0.5,0.5)
        glBegin(GL_LINES)
        for i in range(-5,6):
            glVertex3f(i,0,-5)
            glVertex3f(i,0,5)
            glVertex3f(-5,0,i)
            glVertex3f(5,0,i)
        glEnd()

    def update_camera(self):
        mouse_buttons = pygame.mouse.get_pressed()
        if mouse_buttons[0]:
            x, y = pygame.mouse.get_pos()
            if self.last_mouse_pos:
                dx = x - self.last_mouse_pos[0]
                dy = y - self.last_mouse_pos[1]
                self.cam_theta += math.radians(dx * 0.5)
                self.cam_phi -= math.radians(dy * 0.5)
                self.cam_phi = np.clip(self.cam_phi, 0.1, math.pi/2 - 0.1)
            self.last_mouse_pos = (x, y)
        else:
            self.last_mouse_pos = None

        # Convert spherical to cartesian
        cam_x = self.cam_radius * math.sin(self.cam_phi) * math.cos(self.cam_theta)
        cam_y = self.cam_radius * math.cos(self.cam_phi)
        cam_z = self.cam_radius * math.sin(self.cam_phi) * math.sin(self.cam_theta)

        glLoadIdentity()
        gluLookAt(cam_x, cam_y, cam_z, 0, 1, 0, 0, 1, 0)

    def render(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
                exit()

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.update_camera()
        self.draw_floor()

        # Cubo 2x2 unit sopra il pavimento
        glPushMatrix()
        glTranslatef(0,1,0)  # sposta cubo sopra pavimento
        glRotatef(np.rad2deg(self.roll), 1,0,0)
        glRotatef(np.rad2deg(self.pitch),0,1,0)
        glRotatef(np.rad2deg(self.yaw),0,0,1)
        self.draw_cube()
        glPopMatrix()

        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = VisualizeIMU3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
