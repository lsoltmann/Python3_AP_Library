'''
    Navigation.py
    
    Description: functions used to determine distance, bearing, crosstrack, etc. given waypoints and GPS data
    
    Revision History
    27 Mar 2016 - Created and debugged
    17 Aug 2016 - Restructured and added additional functions
    
    Author: Lars Soltmann
    
    NOTES:
    - Written for python3
    
    REFERENCES:
    - http://www.movable-type.co.uk/scripts/latlong.html
    
    '''


import math

class nav:
    def __init__(self):
        #Radius of Earth
        self.ER=3958.7613*5280 #miles to ft

    ##Distance between two lat/lon coordinates
    #Input units = deg
    def distance(self,p1,p2):
        phi1=math.radians(p1[0])
        phi2=math.radians(p2[0])
        dphi=math.radians(p2[0]-p1[0])
        dlam=math.radians(p2[1]-p1[1])
        a=math.pow(math.sin(dphi*0.5),2)+math.cos(phi1)*math.cos(phi2)*math.pow(math.sin(dlam*0.5),2)
        c=2*math.atan2(math.sqrt(a),math.sqrt(1-a))
        d=self.ER*c

        return d #ft

    ##Bearing between two lat/lon coordinates
    #Input units = deg
    def bearing(self,p1,p2):
        y=math.sin(math.radians(p2[1])-math.radians(p1[1]))*\
        math.cos(math.radians(p2[0]))
        x=math.cos(math.radians(p1[0]))*math.sin(math.radians(p2[0]))-\
        math.sin(math.radians(p1[0]))*math.cos(math.radians(p2[0]))*\
        math.cos(math.radians(p2[1])-math.radians(p2[1]))
        brng=math.degrees(math.atan2(y,x))

        if brng<0:
            brng=brng+360

        return brng #deg

    ##Coordinates of the destination point given a start point, bearing, and distance
    #Input units = deg, ft
    def destination_point(self,p1,b,d):
        p2=[0,0]
        p2[0]=math.asin(math.sin(math.radians(p1[0]))*\
        math.cos(d/self.ER)+math.cos(math.radians(p1[0]))*\
        math.sin(d/self.ER)*math.cos(math.radians(b)))
        p2[1]=math.radians(p1[1])+math.atan2(math.sin(math.radians(b))*\
        math.sin(d/self.ER)*math.cos(math.radians(p1[0])),\
        math.cos(d/self.ER)-math.sin(math.radians(p1[0]))*\
        math.sin(p2[0]))
        p2[0]=math.degrees(p2[0])
        p2[1]=(math.degrees(p2[1])+540)%360-180
    
        return p2 #deg

    ##Crosstrack error at p3 along a path from p1 to p2
    #Input units = deg
    #Sign indicates side, left = neg, right = pos
    def crosstrack(self,p1,p2,p3):
        CTE=math.asin(math.sin(self.distance(p1,p3)/self.ER))*\
        math.sin(math.radians(self.bearing(p1,p3))-\
        math.radians(self.bearing(p1,p2)))*self.ER

        return CTE #ft

