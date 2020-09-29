import bresenham
from math import sin, cos, pi,tan, atan2,log
import math
from itertools import groupby
from operator import itemgetter
import tf
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PointStamped
import sensor_msgs.point_cloud2 as pc2

class localmap:
    def __init__(self, height = 800, width = 800, resolution= 0.1):
        self.height=height
        self.width=width
        self.resolution=resolution
        self.punknown=-1.0
        self.localmap=[self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.logodds=[0.0]*int(self.width/self.resolution)*int(self.height/self.resolution)
        self.origin=int(math.ceil(morigin[0]/resolution))+int(math.ceil(width/resolution)*math.ceil(morigin[1]/resolution))
        self.max_scan_range=1.0
        self.map_origin= (math.floor(self.width/(2*self.resolution)), math.floor(self.height/(2*self.resolution)))

    def resize(self, scale = 4)
        old_height = self.height
        old_width = self.width
        self.height *= scale
        self.width *= scale
        prev_map_origin = self.map_origin
        self.map_origin = (math.floor(self.width/(2*self.resolution)), math.floor(self.height/(2*self.resolution)))
        prev_origin = self.origin
        self.origin = int(self.map_origin[0]) + int(math.floor(self.map_origin[1]*self.width))
        new_local_map = [self.punknown]*int(self.width/self.resolution)*int(self.height/self.resolution)
        for i in range(len(self.localmap)):
            col_x = int(i % (old_width/self.resolution)) - prev_map_origin[0]
            row_y = int(i // (old_width/self.resolution)) - prev_map_origin[1]
            val = self.localmap[i]
            new_ind = col_x + self.map_origin[0] + int((row_y + self.map_origin[1])*self.width/self.resolution)
            new_local_map[new_ind] = val





    def updatemap(self, pcl, range_max, pose):

        robot_origin=int(pose[0])+int(math.ceil(self.width/self.resolution)*pose[1])

        for p in pc2.read_points(pcl, field_names = ('x', 'y', 'z'), skip_nans = True):
            rad = math.sqrt(p[0]*p[0] + p[1]*p[1])
            px = int(rad*cos(pose[2])/self.resolution)
            py = int(rad*sin(pose[2])/self.resolution)

            l = bresenham.bresenham([0,0],[px,py])
            for j in range(len(l.path)):                    
                lpx= int(self.map_origin[0]+pose[0]/self.resolution +l.path[j][0])
                lpy=int(self.map_origin[1]+pose[1]/self.resolution +l.path[j][1])

                if (0<=lpx<self.width/self.
                    resolution and 0<=lpy<self.height/self.resolution):
                    index= int(lpx + math.floor(self.width/self.resolution)*(lpy))
                    if(j<len(l.path)-1):
                        self.localmap[index] = 0
                    else:
                        if rad<self.max_scan_range*range_max:
                            self.localmap[index] = 100