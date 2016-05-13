'''
    Navigation.py
    
    Description: functions used to determine distance, bearing, crosstrack, etc. given waypoints and GPS data
    
    Revision History
    27 Mar 2016 - Created and debugged
    
    Author: Lars Soltmann
    
    INPUTS:

    
    OUTPUTS:

    
    NOTES:
    - Written for python3
    - Equations referenced from: http://www.movable-type.co.uk/scripts/latlong.html
    - '_d' means degrees
    - '_r' means radians
    
    '''


import math
import time

class nav:
    def __init__(self):
        #Radius of Earth
        self.ER=3958.7613*5280 #miles to ft

    #Distance between two lat lon coordinates in ft
    def distance(self,lat1_d,lon1_d,lat2_d,lon2_d):
        lat1_r=math.radians(lat1_d)
        lon1_r=math.radians(lon1_d)
        lat2_r=math.radians(lat2_d)
        lon2_r=math.radians(lon2_d)

        dlat=lat2_r-lat1_r
        dlon=lon2_r-lon1_r

        a=math.pow(math.sin(dlat)*0.5,2)+math.cos(lat1_r)*math.cos(lat2_r)*math.pow(math.sin(dlon)*0.5,2)
        c=2*math.atan2(math.sqrt(a),math.sqrt(1-a))
        d=self.ER*c

        return d #ft

    #Bearing in deg between point1 and point2
    def bearing(self,lat1_d,lon1_d,lat2_d,lon2_d):
        lat1_r=math.radians(lat1_d)
        lon1_r=math.radians(lon1_d)
        lat2_r=math.radians(lat2_d)
        lon2_r=math.radians(lon2_d)

        dlat=lat2_r-lat1_r
        dlon=lon2_r-lon1_r

        theta=math.atan2(math.sin(dlon)*math.cos(lat2_r),math.cos(lat1_r)*math.sin(lat2_r)-math.sin(lat1_r)*math.cos(lat2_r)*math.cos(dlon))
        theta=math.degrees(theta)
        if theta<0:
            theta=theta+360

        return theta #deg

    #Crosstrack error between current position(3) and the greater circle line between start coordinate(1) and end coordinate(2)
    def crosstrack(self,lat1_d,lon1_d,lat2_d,lon2_d,lat3_d,lon3_d):
        d13=self.distance(lat1_d,lon1_d,lat3_d,lon3_d)/self.ER
        b13=math.radians(self.bearing(lat1_d,lon1_d,lat3_d,lon3_d))
        b12=math.radians(self.bearing(lat1_d,lon1_d,lat2_d,lon2_d))

        dxt=math.asin(math.sin(d13))*math.sin(b13-b12)*self.ER

        return dxt #ft

