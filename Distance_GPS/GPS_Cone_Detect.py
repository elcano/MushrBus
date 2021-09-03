# The purpose of this file is to translate gps coordinates to find a distance and then estimate which object is in the GPS field of expectation


# GPS updates 10x a second- can prompt this function to run 10 times and then generate an average to estimate true distance
# inputs: car(lat, long, heading (degrees)), cone(x -> (-x left, +x right), y -> (-y behind, +y ahead))
# we assume every time this function is ran, that the car will be at the origin(0,0)

# EARTH_RADIUS = 6356.752   (km) -> 6378000 (m)

# output: cone(lat, long)

import math

def findConeGPS(car, cone):
    
    # origin + arctan(earth_radius, meters_y) * (180/pi)
    earth_radius = 6378000
    cone.lat = car.lat + (math.atan2(cone.y, earth_radius) * (180/math.pi))
    cos_lat = math.cos(math.pi/180 * car.lat)
    cone.lon = car.lon + (math.atan2(cone.x/cos_lat, earth_radius)) * (180/math.pi)
    # print(cone.lat)
    # print(cone.lon)

    return cone

def cone_orientation(car, cone):
    # find the distance to the cone
    # assume car is at (0,0) -> x1, y1
    # d = sqrt((x2-x1)^2 + (y2-y1)^2) in meters
    d_to_cone = math.sqrt(cone.x**2 + cone.y**2)
    cone_diff_degree = math.degrees(math.atan(cone.y/cone.x))

    correction_deg = 0.0
    if cone.x < 0 or cone.y < 0:
        correction_deg = car.heading - abs(cone_diff_degree)
    else:
        correction_deg = car.heading + abs(cone_diff_degree)
    # print(d_to_cone)
    # print(cone_diff_degree)
    # print(correction_deg)

    return correction_deg


def car():
    x = 0.0
    y = 0.0
    heading = 0.0
    lat = 0.0
    lon = 0.0

def cone():
    x = 0.0
    y = 0.0
    lat = 0.0
    lon = 0.0

if __name__ == '__main__':
    # car.x = 0.0
    # car.y = 0.0
    car.lat = 47.673057
    car.lon = -122.310880
    car.heading = 45.0
    cone.x = -7.315
    cone.y = -21.260
    findConeGPS(car, cone)
