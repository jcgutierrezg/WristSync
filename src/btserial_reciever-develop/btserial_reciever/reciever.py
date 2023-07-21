from statemachine import StateMachine, State
import time
import requests

from warnings import catch_warnings
import rclpy
import serial
import json
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Bool
from rclpy.node import Node

url = "http://192.168.216.70/pose"

posX = 0
posY = 0
posZ = 0
OriX = 0
OriY = 0
OriZ = 0

angle1 = 0.0 
angle2 = 0.0
angle3 = 0.0
angle4 = 0.0
angle5 = 0.0
angle6 = 0.0

roll = 0
pitch = 0
yaw = 0
JoyX = 0
JoyY = 0
button = 0

switchFlag1 = 0
switchFlag2 = 0
switchFlag3 = 0
switchFlag4 = 0
switchFlag5 = 0
switchFlag6 = 0

posXhome = 0.2
posYhome = 0.0
posZhome = 0.65

NullFlag = 0

 

class MasterGlove(StateMachine):
 
    # creating states
    Start = State("Start", initial = True)
    Null = State("Null")
    #ButtonIn = State("ButtonIn")
    InverseKin = State("InverseKin")
    RPY = State("RPY")
    Home = State("Home")
    DirectKin = State("DirectKin")
    Disable = State("Disable")
      
    # transitions of the state
    initiate = Start.to(Null)

    InverseKin_Null = InverseKin.to(Null)
    RPY_Null = RPY.to(Null)
    Home_Null = Home.to(Null)
    DirectKin_Null = DirectKin.to(Null)
    Disable_Null = Disable.to(Null)

    IKMode = Null.to(InverseKin)
    RPYMode = Null.to(RPY)
    HomeMode = Null.to(Home)
    DKMode = Null.to(DirectKin)
    DisableMode = Null.to(Disable)

    def __init__(self):
        super(MasterGlove, self).__init__()


    def NullState(self, BTRecieverO):

        global NullFlag, button

        if(NullFlag == 1):
            time.sleep(0.5)
            button = 0
            NullFlag = 0

        if(button == 2):
            startTime = time.time()

            while(button == 2):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.switchLaser()
            else:
                BTRecieverO.switchGripper() 

        else:
            pass

    def InverseKinState(self, BTRecieverO):

        global NullFlag, button, posX, JoyX, posY, JoyY, posZ, currentSubstate

        if(button == 2):
            startTime = time.time()

            while(button == 2):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.switchLaser()
            else:
                BTRecieverO.switchGripper()

        elif(button == 1):
            startTime = time.time()

            while(button == 1):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.moveItExec()
            else:
                BTRecieverO.moveItPlan()


        else:

            if(currentSubstate == 1):

                if(JoyX>500 and JoyX<540):

                    posX = posX

                elif(JoyX>=540):

                    posX = posX - (JoyX/512-1)/1000

                elif(JoyX<=500):

                    posX = posX + ((1023-JoyX)/512-1)/1000



                if(JoyY>500 and JoyY<540):

                    posY = posY

                elif(JoyY>=540):

                    posY = posY + (JoyY/512-1)/1000

                elif(JoyY<=500):

                    posY = posY - ((1023-JoyY)/512-1)/1000

            else:

                if(JoyX>500 and JoyX<540):

                    posZ = posZ

                elif(JoyX>=540):

                    posZ = posZ - (JoyX/512-1)/1000

                elif(JoyX<=500):

                    posZ = posZ + ((1023-JoyX)/512-1)/1000



                if(JoyY>500 and JoyY<540):

                    posY = posY

                elif(JoyY>=540):

                    posY = posY + (JoyY/512-1)/1000

                elif(JoyY<=500):

                    posY = posY - ((1023-JoyY)/512-1)/1000


    def RPYState(self, BTRecieverO):

        global NullFlag, button, roll, pitch, yaw, OriX, OriY, OriZ

        if(button == 2):
            startTime = time.time()

            while(button == 2):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.switchLaser()
            else:
                BTRecieverO.switchGripper()

        elif(button == 1):
            startTime = time.time()

            while(button == 1):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.moveItExec()
            else:
                BTRecieverO.moveItPlan()
        else:

            OriX = roll
            OriY = -pitch
            OriZ = yaw
            

    def HomeState(self, BTRecieverO):

        global NullFlag, button, posX, posY, posZ, posXhome, posYhome, posZhome, OriX, OriY, OriZ

        if(button == 2):
            startTime = time.time()

            while(button == 2):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.switchLaser()
            else:
                BTRecieverO.switchGripper()

        elif(button == 1):
            startTime = time.time()

            while(button == 1):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.moveItExec()
            else:
                BTRecieverO.moveItPlan()

        else:

            posX = posXhome

            posY = posYhome

            posZ = posZhome

            OriX = 0.0

            OriY = 0.0

            OriZ = 120.0


            #etc

    def DirectKinState(self, BTRecieverO):

        global NullFlag, button, angle1, angle2, angle3, angle4, angle5, angle6, currentSubstate, JoyX, JoyY

        if(button == 2):
            startTime = time.time()

            while(button == 2):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.switchLaser()
            else:
                BTRecieverO.switchGripper()

        elif(button == 1):
            startTime = time.time()

            while(button == 1):
                currentTime = time.time()

                totalTime = currentTime-startTime
                BTRecieverO.readGlove()

            if(totalTime >= 2.0):
                BTRecieverO.publishATTinys(3.0)

            else:
                pass

        else:

            if(currentSubstate == 1):

                if(JoyX>500 and JoyX<540):

                    angle1 = angle1

                elif(JoyX>=540):

                    angle1 = angle1 + (JoyX/512-1)/100

                elif(JoyX<=500):

                    angle1 = angle1 - ((1023-JoyX)/512-1)/100



                if(JoyY>500 and JoyY<540):

                    angle2 = angle2

                elif(JoyY>=540):

                    angle2 = angle2 + (JoyY/512-1)/100

                elif(JoyY<=500):

                    angle2 = angle2 - ((1023-JoyY)/512-1)/100

            elif(currentSubstate == 2):

                if(JoyX>500 and JoyX<540):

                    angle3 = angle3

                elif(JoyX>=540):

                    angle3 = angle3 + (JoyX/512-1)/100

                elif(JoyX<=500):

                    angle3 = angle3 - ((1023-JoyX)/512-1)/100



                if(JoyY>500 and JoyY<540):

                    angle4 = angle4

                elif(JoyY>=540):

                    angle4 = angle4 + (JoyY/512-1)/100

                elif(JoyY<=500):

                    angle4 = angle4 - ((1023-JoyY)/512-1)/100

            else:

                if(JoyX>500 and JoyX<540):

                    angle5 = angle5

                elif(JoyX>=540):

                    angle5 = angle5 + (JoyX/512-1)/100

                elif(JoyX<=500):

                    angle5 = angle5 - ((1023-JoyX)/512-1)/100



                if(JoyY>500 and JoyY<540):

                    angle6 = angle6

                elif(JoyY>=540):

                    angle6 = angle6 + (JoyY/512-1)/100

                elif(JoyY<=500):

                    angle6 = angle6 - ((1023-JoyY)/512-1)/100

        print(angle1)
        print(angle2)
        print(angle3)
        print(angle4)
        print(angle5)
        print(angle6)


class btreciever(Node):

    def __init__(self):
        global glove
        super().__init__('BTReciever')
        self.RPYinfo = self.create_publisher(Pose,'RPYinfo',10)
        self.ATTinyinfo = self.create_publisher(Pose,'ATTinyinfo',10)
        self.ArmAction = self.create_publisher(String,'robocol/arm/action',10)
        self.ExecuteBool = self.create_publisher(Bool,'robocol/arm/next_position',10)
        while(rclpy.ok()):
            masterCycle(self, glove)

    def readGlove(self):

        global roll, pitch, yaw, JoyX, JoyY, button, currentState, currentSubstate

        response = requests.get(url)

        if(response.status_code == 200):
            line = response.text

        #print(line)
        #line = "10;20;30;128;128;0"
        #print(line)
        #print("P")
        splitString = line.split(";")
        if len(splitString) == 8:

            roll = float(splitString[0])
            pitch = float(splitString[1])
            yaw = float(splitString[2])
            JoyX = int(splitString[3])
            JoyY = int(splitString[4])
            button = int(splitString[5])
            currentState = int(splitString[6])
            currentSubstate = int(splitString[7])

    def publishPose(self):

        global posX, posY, posZ, OriX, OriY, OriZ

        msg = Pose()

        msg.position.x = float(posX)
        msg.position.y = float(posY)
        msg.position.z = float(posZ)

        msg.orientation.w = float(0.0)
        msg.orientation.x = float(OriX+180.0) #ORIX
        msg.orientation.y = float(OriY+90.0) #ORIY
        msg.orientation.z = float(90.0) #ORIZ

        print(posX)
        print(posY)
        print(posZ)
        print(OriX)
        print(OriY)
        print(OriZ)

        self.RPYinfo.publish(msg)
        print("Sent Pose to planning")

    def publishATTiny(self, Mode):

        global angle1, angle2, angle3, angle4, angle5, angle6

        msg = Pose()

        msg.position.x = float(angle1)
        msg.position.y = float(angle2)
        msg.position.z = float(angle3)

        msg.orientation.w = float(Mode)
        msg.orientation.x = float(angle4)
        msg.orientation.y = float(angle5)
        msg.orientation.z = float(angle6)

        self.ATTinyinfo.publish(msg)
        print("Sent data to ATTinys")

        angle1 = 0.0
        angle2 = 0.0
        angle3 = 0.0
        angle4 = 0.0
        angle5 = 0.0
        angle6 = 0.0

    def switchLaser(self):
        global angle1, angle2, angle3, angle4, angle5, angle6

        angle1 = 0.0
        angle2 = 0.0
        angle3 = 0.0
        angle4 = 0.0
        angle5 = 0.0
        angle6 = 0.0

        self.publishATTiny(1.0)

    def switchGripper(self):

        global angle1, angle2, angle3, angle4, angle5, angle6

        angle1 = 0.0
        angle2 = 0.0
        angle3 = 0.0
        angle4 = 0.0
        angle5 = 0.0
        angle6 = 0.0

        self.publishATTiny(2.0)

    def moveItPlan(self):

        msg = String()

        msg.data = 'plan'

        self.ArmAction.publish(msg)


    def moveItExec(self):

        msgS = String()

        msgS.data = 'execute'

        self.ArmAction.publish(msgS)

        msgB = Bool()

        msgB.data = True
        self.ExecuteBool.publish(msgB)




def masterCycle(btrecieverN, gloveS):

    global currentState

    while(gloveS.Null.is_active):
        btrecieverN.readGlove()
        gloveS.NullState(btrecieverN)
        #print(currentState)
        print("Null")

        if(currentState == 2):

            gloveS.IKMode()

        elif(currentState == 3):

            gloveS.RPYMode()

        elif(currentState == 4):

            gloveS.HomeMode()

        elif(currentState == 5):

            gloveS.DKMode()
        #btrecieverN.publishPose()

    while(gloveS.InverseKin.is_active):
        btrecieverN.readGlove()
        gloveS.InverseKinState(btrecieverN)
        btrecieverN.publishPose()
        print("IK")
        if(currentState != 2):

            gloveS.InverseKin_Null()

    while(gloveS.RPY.is_active):
        btrecieverN.readGlove()
        gloveS.RPYState(btrecieverN)
        btrecieverN.publishPose()
        print("RPY")
        if(currentState != 3):

            gloveS.RPY_Null()

    while(gloveS.Home.is_active):
        btrecieverN.readGlove()
        gloveS.HomeState(btrecieverN)
        btrecieverN.publishPose()
        print("Home")
        if(currentState != 4):

            gloveS.Home_Null()

    while(gloveS.DirectKin.is_active):
        btrecieverN.readGlove()
        gloveS.DirectKinState(btrecieverN)
        print("DK")
        if(currentState != 5):

            gloveS.DirectKin_Null()


def main(args=None):
    global glove
    glove = MasterGlove()
    glove.initiate()
    rclpy.init(args=args)
    BTReciever=btreciever()
    rclpy.spin(BTReciever)
    BTReciever.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

