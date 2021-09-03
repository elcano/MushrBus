#   The purpose of this file is to determine the heading vector given two points
#       return heading in meters of X1 to X2
   
import math

#   d = sqrt((x2-x1)^2 + (y2-y1)^2) in meters
#       x(#)pos is an array of 2 arrays one lat and one long

def HeadingVec(x1pos, x2pos):
    x1_lat = (x1pos.lat)
    x1_lon = (x1pos.lon)
    x2_lat = (x2pos.lat)
    x2_lon = (x2pos.lon)

    distLon = x2_lon - x1_lon
    distLat = x2_lat - x1_lat

#   calculated with Haversine formula
#   d = 2r(arcsin(sqrt((sin((lat2 - lat1)/2))^2 + cos(lat1) * cos(lat2) *
#       (sin((lon2 - lon1)/2))^2)


    # dist = math.sin(distLat/2)**2 + math.cos(x1_lat) * math.cos(x2_lat) * math.sin(distLon/2)**2
    # dist = 2 * math.asin(math.sqrt(dist))
    # #d = sqrt((x2_lat-x1_lat)**2 + (x2_l-x1_lon)**2)
    # earth_radius = 6378000
    # #dist = dist * earth_radius
    # print(dist/0.0174532925)
    # print(dist * (180/math.pi))
    print(distLon)
    print(distLat)

#   helper functions

def toRadians(position):
    return position * 0.0174532925

if __name__ == '__main__':
    def x1pos():
        lat = 0.0
        lon = 0.0
    def x2pos():
        lat = 0.0
        lon = 0.0
    x1pos.lat = 47.673057
    x1pos.lon = -122.310880
    x2pos.lat = 47.672866 
    x2pos.lon = -122.310978
    HeadingVec(x1pos,x2pos)


