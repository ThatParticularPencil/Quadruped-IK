from math import sin,cos,asin,acos,atan,sqrt,degrees,radians,pi

# variables
RestPos = (0,0,0)     #(x,y,z)
#OffSets = (Hip, Trechanter, Knee)
Joint_Limits = { 
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
        if angle < Joint_Limits[count][0]:
            print( f"{Joint_Limits[count][2]} angle is below limits")
            while angle < Joint_Limits[count][0]:  
                    angle += 1
        if angle > Joint_Limits[count][1]:
            print( f"{Joint_Limits[count][2]} angle is above limits")
            while angle > Joint_Limits[count][1]:  
                angle -= 1               
        Checked_Angles.append(angle)
        

    return Checked_Angles


def IK(pos):
    x,y,z = pos[0],pos[1],pos[2]
    # D is the hypotenuse of xz plane
    # D is the hypotenuse of the xy plane
    D = sqrt(x**2 + z**2)
    if D==0:
        Pelvis = 0 + 0
    else:
        Pelvis =  0 + RTD(asin(z / D)) 

    H = sqrt(((D - Coxa)**2) + y**2)

    #hip0 is the angle between H and the x-axis
    #hip1 is the interior angle of Femur-Hip-H
    Hip0 = RTD(asin(y/H))
    Hip1 = RTD(acos((Tibia**2 - (Femur**2 + H**2 )) / (-2*Femur*H)))
    #Hip1 in interior, so this will make it negative when necessary
    if y < 0:
        Hip1 *= -1
    Hip = 0 + (Hip0+Hip1)

    Knee = (180 - RTD(acos((H**2 - (Tibia**2 + Femur**2 )) / (-2*Tibia*Femur))))

    print (Pelvis,Hip, Knee, "\n")
    return Pelvis, Hip, Knee

pos = (15,-3,0)
angles = Check_MotorAngle(IK(pos))
print(angles)

print("\n\n")




#archived
#code for check joint angles
'''if P > 45:  
            print("Pelvis angle is above limits")
            while P > 45:
                P -= 1
        elif P < -45:
            print("Pelvis angle is below limits")
            while P < -45:
                P += 1

        if H > 135:  
            print("Hip angle is above limits")
            while H > 135:
                H -= 1
        elif H < 0:
            print("Hip angle is below limits")
            while H < 0:
                H += 1    

        if K > 5:  
            print("Knee angle is above limits")
            while K > 5:
                K -= 1
        elif K < -150:
            print("Knee angle is below limits")
            while K < -150:
                K += 1   '''