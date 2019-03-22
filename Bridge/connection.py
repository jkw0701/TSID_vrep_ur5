from Extern import b0RemoteApi as vrep
from Bridge import configuration as cf
import time
import numpy as np

class clientsever:
    def __init__(self, control_mode= 'torque', client_name='b0RemoteApi_pythonClient', server_name = 'b0RemoteApiAddOn'):
        self.mode =control_mode
        self.client = vrep.RemoteApiClient(client_name, server_name)
        self.doNextStep = True
        self.q = np.array(np.zeros(cf.dof))
        self.qdot = np.array(np.zeros(cf.dof))
        self.index = 0
        self.targetvel = 1e+6

    def getMotorHandle(self):
        self.motorHandle = []
        for i in range(0, cf.dof):
            _, motor = self.client.simxGetObjectHandle('joint'+str(i+1), self.client.simxServiceCall())
            self.motorHandle.append(motor)    

        self.client.simxGetJointPosition(self.motorHandle[0], self.client.simxDefaultSubscriber(self.joint1Callback))    
        self.client.simxGetJointPosition(self.motorHandle[1], self.client.simxDefaultSubscriber(self.joint2Callback))    
        self.client.simxGetJointPosition(self.motorHandle[2], self.client.simxDefaultSubscriber(self.joint3Callback))    
        self.client.simxGetJointPosition(self.motorHandle[3], self.client.simxDefaultSubscriber(self.joint4Callback))    
        self.client.simxGetJointPosition(self.motorHandle[4], self.client.simxDefaultSubscriber(self.joint5Callback))    
        self.client.simxGetJointPosition(self.motorHandle[5], self.client.simxDefaultSubscriber(self.joint6Callback))    

        self.client.simxGetObjectFloatParameter(self.motorHandle[0], 2012, self.client.simxDefaultSubscriber(self.jointdot1Callback))
        self.client.simxGetObjectFloatParameter(self.motorHandle[1], 2012, self.client.simxDefaultSubscriber(self.jointdot2Callback))
        self.client.simxGetObjectFloatParameter(self.motorHandle[2], 2012, self.client.simxDefaultSubscriber(self.jointdot3Callback))
        self.client.simxGetObjectFloatParameter(self.motorHandle[3], 2012, self.client.simxDefaultSubscriber(self.jointdot4Callback))
        self.client.simxGetObjectFloatParameter(self.motorHandle[4], 2012, self.client.simxDefaultSubscriber(self.jointdot5Callback))
        self.client.simxGetObjectFloatParameter(self.motorHandle[5], 2012, self.client.simxDefaultSubscriber(self.jointdot6Callback))

    def joint1Callback(self, msg):
        self.q[0] = msg[1]
    def joint2Callback(self, msg):
        self.q[1] = msg[1]
    def joint3Callback(self, msg):
        self.q[2] = msg[1]
    def joint4Callback(self, msg):
        self.q[3] = msg[1]
    def joint5Callback(self, msg):
        self.q[4] = msg[1]
    def joint6Callback(self, msg):
        self.q[5] = msg[1]

    def jointdot1Callback(self, msg):
        self.qdot[0] = msg[1]
    def jointdot2Callback(self, msg):
        self.qdot[1] = msg[1]
    def jointdot3Callback(self, msg):
        self.qdot[2] = msg[1]
    def jointdot4Callback(self, msg):
        self.qdot[3] = msg[1]
    def jointdot5Callback(self, msg):
        self.qdot[4] = msg[1]
    def jointdot6Callback(self, msg):
        self.qdot[5] = msg[1]

    def getMotorState(self):                  
        return self.q, self.qdot

    def setMotorState(self, tdes):
        if self.mode == 'position':
            for i in range(0, cf.dof):
                self.client.simxSetJointTargetPosition(self.motorHandle[i], tdes.item(i), self.client.simxDefaultPublisher())
        elif self.mode == 'torque':
            for i in range(0, cf.dof):
                if tdes.item(i) < 0 :
                    self.client.simxSetJointTargetVelocity(self.motorHandle[i], -self.targetvel, self.client.simxDefaultPublisher())
                else :
                    self.client.simxSetJointTargetVelocity(self.motorHandle[i], self.targetvel, self.client.simxDefaultPublisher())

                self.client.simxSetJointForce(self.motorHandle[i], abs(tdes.item(i)), self.client.simxDefaultPublisher())


  
    
