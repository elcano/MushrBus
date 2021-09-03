#   This file translates car GPS data and distance to landmarks in detecting Cone identity

#   GPS updates 10Hz - can prompt function run 10 times to avg
#   
#   Earths Radius = 6,378 (km)
#
#   inputs: car(lat, long, head (degrees)), cone(x +right, y +forward) meters
#   Assuming the car @ (0,0) for each distance measurement to Cone
#   
#   outputs: cone(lat, long)

import math

def findConeGPS(car, cone):
    # find the distance to the cone
    # assume car is at (0,0) -> x1, y1
    # d = sqrt((x2-x1)^2 + (y2-y1)^2) in meters
    d_to_cone = math.sqrt(cone.x**2 + cone.y**2)
    cone_diff_degree = math.degrees(math.atan(cone.y/cone.x))

    orientation_deg = 0.0
    if cone.x < 0 or cone.y < 0:
        orientation_deg = car.heading - abs(cone_diff_degree)
    else:
        orientation_deg = car.heading + abs(cone_diff_degree)

    earth_radius= 6378000
    
    print(d_to_cone)
    print(cone_diff_degree)
    print(orientation_deg)
# return value of new degree rotation to match orientation to cone and expected orientation to reach cone. 

def car():
    lat = 0.0
    lon = 0.0
    x = 0.0
    y = 0.0
    heading = 0.0
def cone():
    lat = 0.0
    lon = 0.0
    x = 0.0
    y = 0.0

if __name__ == '__main__':        
    # car.x = 0.0
    # car.y = 0.0
    car.heading = 45.0
    cone.x = 3
    cone.y = 5
    findConeGPS(car, cone)

