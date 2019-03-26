
from Bridge import connection 
from Bridge import configuration as cf
from Bridge import kbhit
import numpy as np
import pinocchio as se3


vrep = connection.clientsever()
client = vrep.client

def simulationStepStarted(msg):
    simTime=msg[1][b'simulationTime'];
    
def simulationStepDone(msg):
    simTime=msg[1][b'simulationTime'];
    vrep.doNextStep=True

vrep.getMotorHandle()

client.simxSynchronous(True)
client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
client.simxStartSimulation(client.simxDefaultPublisher())

is_simulation_run = True
exit_flag = False
is_first = True
kbd = kbhit.KBHit()
qdes = np.array(np.zeros(6))
qdes = np.array([0.1, -1.57, 0.3, 0.4, 0.5, 0.6]) 

tdes = np.array(np.zeros(6))
err = np.matrix(np.zeros(6)).T
DT = 0.001


from Controller import robot
robot = robot.RobotState()
robot.updateKinematics(np.matrix(np.zeros(6)).T, np.matrix(np.zeros(6)).T)

i=0
while not exit_flag:    
    if kbd.kbhit():
        key = kbd.getch()
        if key == 'q':
            is_simulation_run = False
            exit_flag = True
        elif key == '\t': # TAB
            if is_simulation_run:
                print "SIMULATION PAUSE"
                is_simulation_run = False
            else:
                print "SIMULATION RUN"
                is_simulation_run = True
        elif key == 'i':
            print "Initial Posture"
            qdes = np.array(np.zeros(cf.dof)) 
        elif key == 'h':
            print "Home Posture"
            qdes = np.array([0.1, -1.57, 0.3, 0.4, 0.5, 0.6, 0.7, 0.7]) 
        elif key == 't':
            print "Task Space Control"
            qdes = np.array([0.1, -1.57, 0.3, 0.4, 0.5, 0.6, 0.7, 0.7]) 


    if is_simulation_run:
        if vrep.doNextStep: # for synchronize      
            vrep.doNextStep=False  
            q, qdot = vrep.getMotorState() #Read Device

            # PD control
            ee = robot.framePosition(np.asmatrix(q).T, 17) # SE3
            ee_vel = robot.jointLinearVelocity(np.asmatrix(q).T, np.asmatrix(qdot).T, 6)
            xdes = np.matrix([0.6448, 0.1091, 0.40]).T

            err = 400*(xdes - ee.translation) - 40*ee_vel

            # Model information
            J = robot.jointJacobian(np.asmatrix(q).T, 6)[:3,:]
            g = robot.getGravity(np.asmatrix(q).T)
            nle = robot.getNonlinear(np.asmatrix(q).T, np.asmatrix(qdot).T)
            mass = robot.getMass(np.asmatrix(q).T)
            
            # joint space control 
            kp = 200.0
            qqdot = kp*(np.asmatrix(qdes).T - np.asmatrix(q).T) - 2.0*np.sqrt(kp)*np.asmatrix(qdot).T
            tdes = mass*qqdot + nle


            # operational space control
            lambda_inv = J*np.linalg.inv(mass)*J.T + 0.001*np.eye(3) # damped pseudo-inverse
            lambda_ = np.linalg.inv(lambda_inv)
            tdes = J.T*lambda_*err + nle - 0.5*np.asmatrix(qdot).T

            # Update Kinematics & Set motor input
            robot.updateKinematics(np.asmatrix(q).T, np.asmatrix(qdot).T)
            vrep.setMotorState(np.asarray(tdes).T)   
            
            client.simxSynchronousTrigger()

    client.simxSpinOnce()
    

client.simxStopSimulation(client.simxDefaultPublisher())
print "Simulation finished"