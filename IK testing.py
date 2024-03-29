from math import sin,cos,asin,acos,atan,sqrt,degrees,radians,pi

# variables
RestPos = (0,0,0)     #(x,y,z)
#OffSets = (Hip, Trechanter, Knee)
Joint_Data = { 
    0 : (-45,90,"Pelvis"),
    1 : (0,135,"Hip"),
    2 : (-150,5,"Knee")
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

def Check_MotorAngle(angles):
    ConstrainedAngles = []
    Checked_Angles = []

    #forces all the angles between -180 and 180
    for joint in angles:
        dummy = joint
        if dummy > 180 or dummy < -180:
            while dummy > 180 or dummy < -180:
                if dummy > 180:
                    dummy -= 360
                    ConstrainedAngles.append(dummy)
                elif dummy < -180:
                    dummy += 360
                    ConstrainedAngles.append(dummy)
        else:
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


def IK(POS):
    x,y,z = POS[0],POS[1],POS[2]
    # D is the hypotenuse of xz plane
    # H is the hypotenuse of the Dy plane
    D = round(sqrt(x**2 + z**2),3)
    pos = (D,y,z)
    if D==0:
        Pelvis = 0
    else:
        Pelvis =  round(RTD(asin(z / D)),3)

    H = sqrt(((D - Coxa)**2) + y**2)

    #hip0 is the angle between H and the x-axis
    #hip1 is the interior angle of Femur-Hip-H
    Hip0 = RTD(asin(y/H))
    print(H,"\n",(Tibia**2 - (Femur**2 + H**2 )) / (-2*Femur*H))
    Hip1 = RTD(acos((Tibia**2 - (Femur**2 + H**2 )) / (-2*Femur*H)))
    Hip = round((Hip0+Hip1),3)

    #Solve for the Position of the knee with the hip to fix the polarity of its angle
    #creates a line for the femur to calulate positive of negative knee angles:
    def FLP(pos,KneePosition,SolveFor ): #Femur Line Prediction
        d,y,z = pos[0],pos[1],pos[2]
        PosD,PosY = KneePosition[0],KneePosition[1]

        slope = (PosY-0)/(PosD-Coxa)
        intercept = -1*slope*Coxa

        if SolveFor == "x":
            if slope != 0:
                return (y-intercept)/slope
            else:
                return 25.397
        else:
            return (slope*d)+intercept
        
    KneePos = round(Femur*RTD(cos(Hip))+Coxa,3),round(Femur*RTD(sin(Hip)),3)
    print(KneePos)
    KneeInterior = round(RTD(acos((H**2 - (Tibia**2 + Femur**2 )) / (-2*Tibia*Femur))),3)
    print(round(FLP(pos,KneePos,"x"),3), round(FLP(pos,KneePos,"y"),3))
    if KneePos[0] < round(FLP(pos,KneePos,"x"),3):
        Knee = round(180 - (KneeInterior * -1),3)
        print("reached, less than x")
    elif KneePos[1] > round(FLP(pos,KneePos,"y"),3):
        Knee = round(180 - (KneeInterior * -1),3)
        print("reached, greater than y") 
    else:   
        Knee = 180 - KneeInterior

    print (Pelvis,Hip, Knee, "\n")
    return Pelvis, Hip, Knee


pos = (9.483,0,0) #end affector
angles = Check_MotorAngle(IK(pos))
print(angles)

print("\n\n")




#archived