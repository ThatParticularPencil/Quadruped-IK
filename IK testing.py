from math import sin,cos,tan,asin,acos,atan,sqrt,degrees,radians,pi
import time

# variables
RestPos = (10,-10,0)     #(x,y,z)
Lerp_Step = 2
#OffSets = (Hip, Trechanter, Knee)
Joint_Data = { 
    0 : (-45,60,"Pelvis"),
    1 : (-120,120,"Hip",6),
    2 : (-165,165,"Knee",3)
}


# segment lengths in inches
Coxa = 5.197
Femur = 7.974
Tibia = 12.226


# essential functions
print("\n \n")
def RTD(angle):
    deg = angle * (180/pi)
    while deg >= 360:
        deg -= 360
    return deg
def DTR(angle):
    deg = angle * (pi/180)
    while deg >= 2*pi:
        deg -= 2*pi
    return deg

def Constrain(angle):
    if angle > 180 or angle < -180:
        while angle > 180: 
            angle -= 360
        while angle < -180:
            angle += 360
    return angle
def Check_MotorAngles(angles):
    ConstrainedAngles = []
    Checked_Angles = []
    #forces all the angles between -180 and 180
    for joint in angles:
        Constrain(joint)
        ConstrainedAngles.append(joint)

    #P, H, K = ConstrainedAngles[0], ConstrainedAngles[1], ConstrainedAngles[2]

    #uses joint limits to determine force angles into reasonable ranges
    for count,angle in enumerate(ConstrainedAngles):
        if angle < Joint_Data[count][0]:
            print( f"{Joint_Data[count][2]} angle is below limits")
            while angle < Joint_Data[count][0]:  
                    angle += 1
        if angle > Joint_Data[count][1]:
            print( f"{Joint_Data[count][2]} angle is above limits")
            while angle > Joint_Data[count][1]:  
                angle -= 1               
        Checked_Angles.append(angle)
        

    return Checked_Angles

def FK(angles):
    Pelvis,Hip, Knee = angles[0],angles[1],angles[2]

    #step
    pos = [Coxa,0,0]
    pos[0] += Femur*cos(DTR(Hip))
    pos[1] += Femur*sin(DTR(Hip))

    #step again
    pos[0] += Tibia*cos(DTR(Hip+Knee))
    pos[1] += Tibia*sin(DTR(Hip+Knee))

    pos = [round(pos[0],3),round(pos[1],3),0]

    #solve z axis
    pos[2] += round((pos[0] * tan(Pelvis)),3)

    return pos

def IK(POS):
    x,y,z = POS[0],POS[1],POS[2]
    # D is the hypotenuse of xz plane
    # H is the hypotenuse of the Dy plane
    D = round(sqrt(x**2 + z**2),3)
    if D==0:
        Pelvis = 0
    else:
        Pelvis =  Constrain(round(RTD(asin(z / D)),3))


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
    Hip = Constrain(round((HipComp+HipInr),3))

    #fix for positions behind Hip joint
    if D <= 0:
        print("reached D<0")
        Hip = round((Constrain(180 - Hip)),3)

    #Solves Knee
    KneeInterior = round(RTD(acos((H**2 - (Tibia**2 + Femur**2 )) / (-2*Tibia*Femur))),3)
    polarity = 1
    Knee = Constrain(polarity * round((180 - KneeInterior),3))
    if abs(FK((Pelvis,Hip,Knee))[0]-x) > 2 or abs(FK((Pelvis,Hip,Knee))[1]-y) > 2:
        polarity = -1
        Knee = Constrain(polarity * round((180 - KneeInterior),3))


    return (Pelvis, Hip, Knee)


def Pavg(points):
    Xs = [point[0] for point in points]
    Ys = [point[1] for point in points]
    Zs = [point[2] for point in points]
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

########################################################################

# main
def One_Leg_Test():
    #axis to test
    testingX = False
    testingY = False
    testingZ = True

    while True:
        pathX = ((10,-5,0),(20,-5,0))    #(start),(end)
        pathY = ((10,-10,0),(10,-3,0))
        pathZ = ((10,-5,-5),(10,-5,5))
        ActivePaths = [None,None,None]  #xyz
        Keyframes = []                  #(start),(end)
        starts = []                     #((x,y,z),(x,y,z))
        ends = []                       #((x,y,z),(x,y,z))
        frames = []                     #((x,y,z),(...))
        angles = []                     #((P,H,K),(...))

        if testingX == True:
            ActivePaths[0] = pathX
        else:
            ActivePaths[0] = None

        if testingY== True:
            ActivePaths[1] = pathY
        else:
            ActivePaths[1] = None
        if testingZ == True:
            ActivePaths[2] = pathZ
        else:
            ActivePaths[2] = None

        for path in ActivePaths:
            if path != None:
                starts.append(path[0])
                ends.append(path[1])

        if len(starts)>0 and len(ends)>0:
            Keyframes = (Pavg(starts),Pavg(ends))

        frames = interpolate(Keyframes)
        print(frames)
        angles = [Check_MotorAngles(IK(frame)) for frame in frames]
        print(angles)
        return angles   


One_Leg_Test()
#print(IK((0,0,0)))
print("\n\n")
#print(FK((0,184,142)))