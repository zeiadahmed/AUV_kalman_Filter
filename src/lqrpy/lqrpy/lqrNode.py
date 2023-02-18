#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from math import cos, sin, fmod, pi

from gazebo_msgs.srv import SetModelState

import transformations
from lqrpy.Matrices import FrameTranformation
import numpy as np
from lqrpy.ModelMatercies import ModelMatercies
from lqrpy.LQR import LQR


class LQR_node(Node):
    def __init__(self):
        
        super().__init__("node_name")
        # self.stateReceiver = self.create_subscription(
        #     ModelStates, '/gazebo/model_states', self.stateReceiveCallBack, 10)       #Extract Ground Truth (ModelStates) Message Subscriber
        self.is_initialized=False
        self.sensorFusionStateReceiver = self.create_subscription(
            Odometry, '/kalmen_filter/state', self.stateReceiveCallBack, 10)            #Estimated State (Odometry) Message Subscriber 

        self.get_logger().info('Curerent State receiver init')
        self.forcePublisher = self.create_publisher(
            msg_type=Wrench, topic='/rexrov/thruster_manager/input', qos_profile=10)
        self.get_logger().info('Force Publisher init')
        
        self.nextStateService = self.create_service(SetModelState,"/control/set_next_state",self.nextStateReceiveServiceCallBack)

        self.currentState = []
        self.nextState = []
        self.trans = FrameTranformation()
        self.modelMatricies = ModelMatercies()
        self.BMatrix = self.modelMatricies.calcBMatrix()

        self.lqr = LQR()

    def nextStateReceiveServiceCallBack(self,request,response):
        self.nextState = self.ExtractModelState(request.model_state)
        response.success = True
        return response


    def ExtractOdometryState(self, state):
        # Extracting Global State
        # Position:
        
        # pos = state.pose[3]        ModelStates Message      
        # position = pos.position
        position = state.pose.pose.position     # Odemetry Message
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        # quat = pos.orientation       ModelStates Message 
        quat = state.pose.pose.orientation    # Odemetry Message

        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        # twist = state.twist[3]        ModelStates Message
        # linear = twist.linear
        
        linear = state.twist.twist.linear      #Odemetry Message
        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities
    
        # ang = twist.angular           ModelStates Message
        ang = state.twist.twist.angular         # Odometry Message
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z   

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]
    
    def ExtractModelState(self,state):
        
        # Position:
        
        # pos = state.pose[3]        ModelStates Message      
        # position = pos.position
        position = state.pose.position     # ModelState Message
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        # quat = pos.orientation       ModelStates Message 
        quat = state.pose.orientation    # ModelState Message

        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        # twist = state.twist[3]        ModelStates Message
        # linear = twist.linear
        
        linear = state.twist.linear      #ModelState Message
        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities
    
        # ang = twist.angular           ModelStates Message
        ang = state.twist.angular         # ModelState Message
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z   

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]
    
    def ExtractGroundTruth(self,state):
        # Extracting Global State
        # Position:
        
        pos = state.pose[3]        #ModelStates Message      
        position = pos.position
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        quat = pos.orientation     #  ModelStates Message 

        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        twist = state.twist[3]        #ModelStates Message
        linear = twist.linear
        
        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities
    
        ang = twist.angular          # ModelStates Message
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
        if self.is_initialized == False:
            globalState = self.ExtractOdometryState(state)
            self.currentState = globalState
            self.nextState = globalState
            self.is_initialized = True
        
        else:
            # globalState = self.ExtractGroundTruth(state)  # Extract GroundTruth
            globalState = self.ExtractOdometryState(state)

            self.trans.defineNedAngles(globalState[3:6])
            self.vel = self.trans.Transform(np.array(globalState[6:]))

            self.AMatrix = self.modelMatricies.calcAMatrix(self.vel)
            
                            #  x     y   z     r   p     y   
            self.Q = np.diag([
                100,    # x
                200,    # y
                200,    # z
                500,    # roll
                500,    # pitch
                8000,   # yaw     
                1,      # x'
                1,      # y'
                1,      # z'
                1,      # roll'
                1,      # pitch'
                1       # yaw'
                ])
            
            self.R = np.diag([
                0.01,   # Force x            
                0.01,   # Force y
                0.01,   # Force z
                0.1,   # Roll Moment
                0.1,   # Pitch Moment
                0.05    # Yaw   Moment
            ])
            # self.R = np.eye(6)*0.01
            self.lqr.CalculateLQR(self.AMatrix, self.BMatrix, self.Q, self.R)
            k = self.lqr.getLQRGain()

            # self.currentState=np.array([xpos,ypos,zpos,roll,pitch,yaw,*self.vel])

            # self.nextState = np.array(
            #     [-4,4, -3, 0, (pi/4)*0, (pi/4)*2, 0, 0, 0, 0, 0, 0])
            error = np.subtract(globalState, self.nextState)

            error[3] = self.wrapAngle( error[3])
            error[4] = self.wrapAngle( error[4])
            error[5] = self.wrapAngle( error[5])

            GlobalPosError = np.array(error[0:3])
            relativeBodyPosError = self.trans.PosTransform(GlobalPosError)
            GlobalVelError = error[6:]
            relativeBodyVelError = self.trans.Transform(GlobalVelError)

            error = [*relativeBodyPosError, *error[3:6], *relativeBodyVelError]
            u = np.matmul(-k, error)
            W = 18258
            B=18522  

            #  0 0 0 -bouyancy*cos(globalState[4])*sin(globalState[3]) -bouyancy*sin(4) 0
            gMatrix = np.array([0,0,-(W-B),0,0,0 ] )
            u=u-gMatrix
            # print('Yaw :',self.currentState[5])
            print('\nError x', error[0], '\nError y', error[1],'\nError z',error[2], '\nError roll',error[3],'\nError Pitch',error[4],'\nError Yaw', error[5] )

            print('\nForce x', u[0], '\nForce y ', u[1], '\nForce z', u[2],'\nMoment Roll x', u[3], '\nMoment Pitch y ', u[4], '\nMoment Yaw z', u[5])

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
