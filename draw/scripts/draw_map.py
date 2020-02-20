#!/usr/bin/env python

import roslib, rospy, rospkg
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from std_msgs import *
import cv2
import numpy as np

UNKNOWN = -1;
FREE = 0;
OCCUPIED = 1;
INFLATED = 2;

def pos2idx(pos):
    global rbt_idx, cell_size, min_pos, occ_grid, width, height, img_mat, initialized
    w = cell_size
    mp = min_pos
    idx = ((pos[0] - mp[0])/w, (pos[1] - mp[1])/w)
    return (np.int64(round(idx[0])), np.int64(round(idx[1])))

def draw():
    global rbt_idx, cell_size, min_pos, occ_grid, width, height, img_mat, initialized
    if initialized == True:
        for i in xrange(width):
            for j in xrange(height):
                if occ_grid[i][j] == OCCUPIED:
                    img_mat[i, j, :] = (255, 255, 255) # white
                elif occ_grid[i][j] == INFLATED:
                    img_mat[i, j, :] = (180, 180, 180) # light gray
                elif occ_grid[i][j] == FREE:
                    img_mat[i, j, :] = (0, 0, 0) # black
                elif occ_grid[i][j] == UNKNOWN:
                    img_mat[i, j, :] = (255, 0, 0) # red
        # color the robot position as a crosshair
        img_mat[rbt_idx[0], rbt_idx[1], :] = (0, 255, 0) # green
        cv2.imshow('img', img_mat)
        cv2.waitKey(10)

def subscribe_pos(msg):
    global rbt_idx, initialized
    if ~np.isnan(cell_size):
        rbt_idx[0],rbt_idx[1] = pos2idx((msg.x,msg.y))


def subscribe_map(msg):
    global cell_size, min_pos, occ_grid, width, height,img_mat,initialized
    if initialized == False:
        cell_size = msg.info.resolution
        min_pos = [msg.info.origin.position.x,msg.info.origin.position.y]
        width = msg.info.width
        height = msg.info.height
        cv2.namedWindow('img', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('img', width*5, height*5) # so each cell is 5px*5px
        img_mat = np.full((width, height, 3), np.uint8(127))
    arr = np.array(msg.data)
    occ_grid = np.split(arr,width);
    draw()

def main():
    rospy.init_node('draw_node')
    global rbt_idx, cell_size, min_pos, occ_grid, width, height, img_mat, initialized
    rbt_idx = [np.nan]*2;
    min_pos = [np.nan]*2;
    width = np.nan
    height = np.nan
    cell_size = np.nan
    initialized = False

    rospy.Subscriber('robot_pose', Pose2D, subscribe_pos, queue_size=1)
    rospy.Subscriber('map', OccupancyGrid, subscribe_map, queue_size=1)

    # Wait for Subscribers to receive data.
    while np.isnan(width) or np.isnan(rbt_idx[0]):
        pass

    initialized = True;
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
