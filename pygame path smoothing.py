import pygame as pg
import numpy as np
from scipy import interpolate 
#variables
smooth = []
ControlPoints = []

def BSpline(waypoints):
    x = []
    y = []

    for point in waypoints:
        x.append(point[0])
        y.append(point[1])

    """
    spline prep with all of the x and y points returns a lot but we ask for tck:
    (t,c,k) contains containing 
    - the vector of knots, 
    - the B-spline coefficients, 
    -and the degree of the spline.

    then we make a variable "u" and fill it with any number of points between
    0 and 1 with np.linspace

    interpolate.splev takes the number of points and the spline and puts that into a variable "smooth"
    """
    tck, *rest = interpolate.splprep([x,y]) 
    u = np.linspace(0,1,num=75)
    smooth = interpolate.splev(u,tck)
    return smooth

pg.init
pg.display.set_caption("PATH PLANNING")
map = pg.display.set_mode((800,512))
map.fill((255,255,255))
running = True

while(running):
    for event in pg.event.get():
        if event.type == pg.QUIT:
            running = False
        elif event.type == pg.MOUSEBUTTONDOWN:
            pos = pg.mouse.get_pos()
            pg.draw.circle(map,(0,20,0),pos,7)
            ControlPoints.append(pos)
        elif event.type == pg.KEYDOWN:
            smooth = BSpline(ControlPoints)
            X_smooth, Y_smooth = smooth
            map.fill((255,255,255))
            ControlPoints = []

            for x,y in zip(X_smooth, Y_smooth):
                pg.draw.circle(map,(255,100,0),(x,y),2,0)   


    pg.display.update()
print(ControlPoints)