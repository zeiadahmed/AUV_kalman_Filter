#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Wrench
from math import cos, sin, fmod, pi

import transformations
from lqrpy.Matrices import FrameTranformation
import numpy as np
from lqrpy.ModelMatercies import ModelMatercies
from lqrpy.LQR import LQR


class LQR_node(Node):
    def __init__(self):
        super().__init__("node_name")
        self.stateReceiver = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.stateReceiveCallBack, 10)
        self.get_logger().info('Curerent State receiver init')
        self.forcePublisher = self.create_publisher(
            msg_type=Wrench, topic='/rexrov/thruster_manager/input', qos_profile=10)
        self.get_logger().info('Force Publisher init')
        
        self.currentState = []
        self.trans = FrameTranformation()
        self.modelMatricies = ModelMatercies()
        self.BMatrix = self.modelMatricies.calcBMatrix()
        self.lqr = LQR()

    def ExtractGlobalState(self, state):
        # Extracting Global State
        # Position:
        pos = state.pose[3]
        position = pos.position
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        quat = pos.orientation
        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        twist = state.twist[3]
        linear = twist.linear

        # Angular Velocities
        vx, vy, vz = linear.x, linear.y, linear.z
        ang = twist.angular
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]

    def wrapAngle(self, angle):

        angle = fmod(angle, (2.0 * pi))
        if (angle <= -pi):

            angle += (2.0 * pi)

        elif (angle > pi):

            angle -= (2.0 * pi)

        return angle

    def stateReceiveCallBack(self, state):

        globalState = self.ExtractGlobalState(state)

        self.trans.defineNedAngles(globalState[3:6])
        self.vel = self.trans.Transform(np.array(globalState[6:]))

        self.AMatrix = self.modelMatricies.calcAMatrix(self.vel)
        
        #  x     y   z   r   p   y    x'  y'  z'  r'  p'  y'
        self.Q = np.diag([100, 200, 100, 1, 1, 8000,
                         100, 100, 100, 100, 100, 100])
        self.R = np.eye(6)*0.01
        self.lqr.CalculateLQR(self.AMatrix, self.BMatrix, self.Q, self.R)
        k = self.lqr.getLQRGain()

        # self.currentState=np.array([xpos,ypos,zpos,roll,pitch,yaw,*self.vel])

        nextState = np.array(
            [-3, 3, 0, 0, (pi/4)*0, (pi/4)*3, 0, 0, 0, 0, 0, 0])
        error = np.subtract(globalState, nextState)

        error[3] = self.wrapAngle( error[3])
        error[4] = self.wrapAngle( error[4])
        error[5] = self.wrapAngle( error[5])

        GlobalPosError = np.array(error[0:3])
        relativeBodyPosError = self.trans.PosTransform(GlobalPosError)
        GlobalVelError = error[6:]
        relativeBodyVelError = self.trans.Transform(GlobalVelError)

        error = [*relativeBodyPosError, *error[3:6], *relativeBodyVelError]
        u = np.matmul(-k, error)
        bouyancy = 0.3*1862.87*9.81

        #  0 0 0 -bouyancy*cos(globalState[4])*sin(globalState[3]) -bouyancy*sin(4) 0
        gMatrix = np.array([0, 0, -5, -bouyancy*cos(globalState[4])*sin(globalState[3]), -bouyancy*sin(4), 0 ] )
        
        u= u + np.matmul(self.BMatrix,gMatrix)
        # print('Yaw :',self.currentState[5])
        print('Error x', error[0], 'Error y', error[1], 'Error Yaw', error[5])

        print('Force x', u[0], 'Force y ', u[1])

        publishMsg = Wrench()
        publishMsg.force.x = u[0]
        publishMsg.force.y = u[1]
        publishMsg.force.z = u[2]
        publishMsg.torque.x = u[3]
        publishMsg.torque.y = u[4]
        publishMsg.torque.z = u[5]

        self.forcePublisher.publish(publishMsg)


def main(args=None):
    rclpy.init(args=args)
    node = LQR_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
