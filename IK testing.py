#region VEXcode Generated Robot Configuration
from vex import *
import urandom

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
PelvisMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
HipMotors_motor_a = Motor(Ports.PORT2, GearSetting.RATIO_18_1, True)
HipMotors_motor_b = Motor(Ports.PORT3, GearSetting.RATIO_18_1, False)
HipMotors = MotorGroup(HipMotors_motor_a, HipMotors_motor_b)
KneeMotor = Motor(Ports.PORT4, GearSetting.RATIO_18_1, True)


# wait for rotation sensor to fully initialize
wait(30, MSEC)
#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      Write_to_Motor_Test
#	Author:       Chinonso Ogwudu
#	Created:      04|09|2024
#	Description:  
# 
# ------------------------------------------

# Library imports
from vex import *
from math import sin,cos,tan,asin,acos,atan,sqrt,degrees,radians,pi

# variables
RestPos = (10,-10,0)     #(x,y,z)
Lerp_Step = 2
#OffSets = (Hip, Trechanter, Knee)
Joint_Data = { 
    0 : (-45,60,"Pelvis"),
    1 : (-120,120,"Hip",6),
    2 : (-165,165,"Knee",3)
}
JointVelocity = 20


# segment lengths in inches
Coxa = 5.197
Femur = 7.974
Tibia = 12.226


# essential functions
print("\n \n")
def RTD(angle):
    deg = angle * (180/pi)
    return deg
def DTR(angle):
    deg = angle * (pi/180)
    return deg

def Normalize(ang):
    while ang > 180: 
        ang -= 360
    while ang < -180:
        ang += 360
    return ang
def Check_MotorAngles(angles):
    ConstrainedAngles = []
    Checked_Angles = []
    #forces all the angles between -180 and 180
    for joint in angles:
        ConstrainedAngles.append(Normalize(joint))

    #uses joint limits to force angles into reasonable ranges
    counter =0
    for ang in ConstrainedAngles:
        if ang < Joint_Data[counter][0]:
            print( Joint_Data[counter][2], "angle is below limits")
            while ang < Joint_Data[counter][0]:  
                    ang += 1
        if ang > Joint_Data[counter][1]:
            print( Joint_Data[counter][2],"angle is above limits")
            while ang > Joint_Data[counter][1]:  
                ang -= 1               
        Checked_Angles.append(ang)
        counter +=1
        

    return Checked_Angles

def LocalFK(angles):
    Pelvis,Hip, Knee = angles[0],angles[1],angles[2]

    #step
    pos = [Coxa,0]
    pos[0] += Femur*cos(DTR(Hip))
    pos[1] += Femur*sin(DTR(Hip))

    #step again
    pos[0] += Tibia*cos(DTR(Hip+Knee))
    pos[1] += Tibia*sin(DTR(Hip+Knee))

    pos = [round(pos[0],3),round(pos[1],3),0]
    return pos

def IK(POS):
    x,y,z = POS[0],POS[1],POS[2]
    # D is the hypotenuse of xz plane
    # H is the hypotenuse of the Dy plane
    D = round(sqrt(x**2 + z**2),3)
    if D==0:
        Pelvis = 0
    else:
        Pelvis =  Normalize(round(RTD(asin(z / D)),3))


    #2dof y-d plane:
    
    #changes D to be relative to coxa
    D -= Coxa

    # Check if the target is reachable.
    C = (D**2 + y**2 - Femur**2 - Tibia**2) / (2*Femur*Tibia)
    # If C > 1, it means the target is outside the workspace of the robot
    # If C < -1, it means the target is inside the workspace, but not reachable
    if C > 1 or C < -1:
        print("!!TARGET IS NOT REACHABLE!!\n")

    H = sqrt((D**2) + y**2)

    #HipComp is the angle between H and the x-axis
    #HipInr is the interior angle of Femur-Hip-H
    HipComp = RTD(asin(y/H))
    HipInr = RTD(acos((Tibia**2 - Femur**2 - H**2 ) / (-2*Femur*H)))
    Hip = Normalize(round((HipComp+HipInr),3))

    #fix for positions behind Hip joint
    if D <= 0:
        print("reached D<0")
        Hip = round((Normalize(180 - Hip)),3)

    #Solves Knee
    KneeInterior = round(RTD(acos((H**2 - (Tibia**2 + Femur**2 )) / (-2*Tibia*Femur))),3)
    polarity = 1
    Knee = Normalize(polarity * round((180 - KneeInterior),3))
    if abs(LocalFK((Pelvis,Hip,Knee))[0]-x) > 2 or abs(LocalFK((Pelvis,Hip,Knee))[1]-y) > 2:
        polarity = -1
        Knee = Normalize(polarity * round((180 - KneeInterior),3))

    return (Pelvis, Hip, Knee)

def Pavg(points):
    Xs = [point[0] for point in points if point != None]
    Ys = [point[1] for point in points if point != None]
    Zs = [point[2] for point in points if point != None]
    if len(Xs)>1 or len(Ys)>1 or len(Zs)>1: 
        newpoint = ((round((sum(Xs)/len(Xs)),3)) , (round((sum(Ys)/len(Ys)),3)) , (round((sum(Zs)/len(Zs)),3)))
        return newpoint
def interpolate(points):
    def RepLerp(points):
        Path = []   
        for i in range(len(points)-1):
            Path += [points[i],tuple(Pavg((points[i],points[i+1])))]
        Path.append(points[-1])
        return Path
    
    LerpPath = points
    for loop in range(Lerp_Step):
        LerpPath = RepLerp(LerpPath)

    return LerpPath



########################################################################################################### 
#main

#initializing joints
PelvisMotor.set_position(0, DEGREES)
HipMotors.set_position(0, DEGREES)
KneeMotor.set_position(0, DEGREES)

PelvisAngle = 0
HipAngle = 0
KneeAngle = 0

AngleCounter = 0

PelvisMotor.set_velocity(JointVelocity, PERCENT)
HipMotors.set_velocity(JointVelocity, PERCENT)
KneeMotor.set_velocity(JointVelocity, PERCENT)

#button presses
testingX = False
testingY = False
testingZ = False
def run_on_bX():
    global testingX, testingY, testingZ
    testingX = True
    testingY = False
    testingZ = False
def run_on_bY():
    global testingX, testingY, testingZ
    testingY = True
    testingX = False
    testingZ = False
def run_on_bZ():
    global testingX, testingY, testingZ
    testingZ = True
    testingX = False
    testingY = False
def run_on_bB():
    global testingX, testingY, testingZ
    testingZ = False
    testingX = False
    testingY = False
def run_on_bA():
    global AngleCounter
    AngleCounter += 1

#test start
controller_1.buttonX.pressed(run_on_bX)
controller_1.buttonY.pressed(run_on_bY)
controller_1.buttonUp.pressed(run_on_bZ)
controller_1.buttonB.pressed(run_on_bB)
controller_1.buttonA.pressed(run_on_bA)
wait(10, MSEC)

    
def Write_to_Motors(current,angles):
    adjustments = [angles[0]-current[0], 3*(angles[1]-current[1]), 3*(angles[2]-current[2])]
    print("adjustments",adjustments)
    PelvisMotor.spin_for(FORWARD, adjustments[0], DEGREES)
    HipMotors.spin_for(FORWARD,adjustments[1],DEGREES)
    KneeMotor.spin_for(FORWARD,adjustments[2],DEGREES)

    ErrorPelvis = abs((current[0]+adjustments[0])-PelvisMotor.position(DEGREES))
    ErrorHip = abs((current[1]+adjustments[1])-HipMotors.position(DEGREES))
    ErrorKnee = abs((current[2]+adjustments[2])-KneeMotor.position(DEGREES))

    while ErrorPelvis > 5 or ErrorHip > 5 or ErrorKnee >5:
        wait(1,SECONDS)
        print("moving")
    global PelvisAngle
    global HipAngle
    global KneeAngle

    PelvisAngle = angles[0]
    HipAngle = angles[1]
    KneeAngle = angles[2]
    print("angle",angles)
    print("updated angles",PelvisAngle,HipAngle,KneeAngle)
    return "done moving"

def One_Leg_Test():
    pathX = ((10, -5, 0), (20, -5, 0))  #(start),(end)
    pathY = ((10, -10, 0), (10, -3, 0))
    pathZ = ((10, -5, -5), (10, -5, 5))

    while True:
        wait(1, SECONDS)

        
        ActivePaths = [None]  # xyz
        Keyframes = []  # (start),(end)
        starts = None
        ends = None
        frames = []
        angles = []

        if testingX == True:
            ActivePaths[0] = pathX
        elif testingY== True:
            ActivePaths[0] = pathY
        elif testingZ == True:
            ActivePaths[0] = pathZ
        else:
            ActivePaths = [(None,None,None)]

        for i in ActivePaths:
            if i != None:
                starts = i[0]
                ends = i[1]

        if starts != None and ends != None:
            Keyframes = (starts,ends)
            #print(Keyframes)
            frames = interpolate(Keyframes)
            #print(frames)
            angles = [Check_MotorAngles(IK(frame)) for frame in frames]
            #print(angles)
            ###########################################################################################################     
            if abs(angles[AngleCounter][0]-PelvisAngle) >5 or abs(angles[AngleCounter][1]-HipAngle) >5 or abs(angles[AngleCounter][2]-KneeAngle) >5:
                Write_to_Motors((PelvisAngle,HipAngle,KneeAngle),angles[AngleCounter])
    






#archived


One_Leg_Test()