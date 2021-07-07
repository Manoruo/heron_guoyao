#!/usr/bin/env python
import rospy
import math

import sys
import time

from geometry_msgs.msg import Vector3Stamped
from heron_msgs.msg import Helm
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

pos_cur=NavSatFix()

yaw_cur = 0.00
yaw_des_old = 0.0

latitude = 39.94339440042317
longitude = -75.19981109992537

# conversions from lon/lat to meters
met_lat = 111033.4717 
met_lon = 85467.2528

square_leg_length = 0.0001

course = ([latitude, longitude], 
          [latitude + square_leg_length, longitude], 
          [latitude + square_leg_length, longitude + square_leg_length],
          [latitude, longitude + square_leg_length],
          [latitude, longitude])
         
i = 0
kp=2
wypt_dist_thresh = 2.0  # distance threshold to stop or switch to next way point
cmd_dt = 0.1            # time interval for cmd_helm publishing
base_thrust = 1.0       # the base thrust level at which the Heron will run


flag = "v"


def lawn_mower(course, flag="v"):
    points = 7
    flat_len = square_leg_length
    
    num_points = len(course)
    new_course = []
    
    if num_points >= 4:
         # Assuming we get a square entry

        p1 = course[0]
        p2 = course[1]
        p4 = course[3]

        width = (p2[0] - p1[0])
        height = (p4[1] - p1[1])

        print("Width, Height:", width, height)

        if flag == "v":
            flat = width * 0.5
            
            """for i in range(points):
                delta_y = (math.cos(-math.pi * i) * height) + p1[1]
                delta_x = (math.sin(-math.pi * i / 2) * flat_len) + p1[0]
                new_course[i] = [delta_x, delta_y]
            print(new_course)
            """
            # Hard Coded
            new_course = [[latitude, longitude],
                          [latitude, longitude + height],
                          [latitude + flat, longitude + height],
                          [latitude + flat, longitude],
                          [latitude + (2 * flat), longitude],
                          [latitude + (2 * flat), longitude + height],
                          [latitude + (3 * flat), longitude + height],
                          [latitude + (3 * flat), longitude]]
            # Loop code 
            x_multi = 0
            y_multi = 0 
            
            for i in range(points):
               
                if i != 0 and (i % 3) != 0:
                    lon = longitude + (height)
                else:
                    lon = longitude

                lat = latitude + (x_multi * flat)
                
                new_course[i] = [lat, lon]
                
                if i % 2 == 1:
                    x_multi += 1
                
            return new_course
        elif flag == "h":
            pass
        
        
    else:
        print("Supply at least 4 points")

    return course

# just for testing, take this out 
course_desired = lawn_mower(course, flag)

def nav_comp(navsat_msg):
    global pos_cur
    pos_cur = navsat_msg
    return

def yaw_callback(data_msg):
    global yaw_cur
    yaw_cur = data_msg.vector.z
    if(yaw_cur < 0):
    	yaw_cur = yaw_cur + 2 * math.pi
    return


def control_publisher(event):
    global yaw_cur, i, kp, pos_cur, yaw_des_old, course_desired, wypt_dist_thresh, cmd_dt, base_thrust, met_lat, met_lon
    pub_msg = Helm()
    helm_pub = rospy.Publisher('/cmd_helm', Helm, queue_size=100)
    lat_pub = rospy.Publisher('/waypoint_lat', Float32, queue_size=100)
    long_pub = rospy.Publisher('/waypoint_long', Float32, queue_size=100)

    pos_des_lat = course_desired[i][0]
    pos_des_lon = course_desired[i][1]

    delta_lat = (pos_des_lat-pos_cur.latitude)*met_lat
    delta_lon = (pos_des_lon - pos_cur.longitude)*met_lon    
    dist_err = math.sqrt( delta_lat**2 + delta_lon**2 )
    
    print ("dist error", dist_err)
    print (i)

    lat_pub.publish(float(course_desired[i][0]))
    long_pub.publish(float(course_desired[i][1]))

    if (dist_err < wypt_dist_thresh) :
        # go to next way point if within distance threshold
        if( i < len(course_desired)-1 ):
            i = i+1
            pos_des_lat = course_desired[i][0]
            pos_des_lon = course_desired[i][1]
            delta_lat = (pos_des_lat-pos_cur.latitude)*met_lat
            delta_lon = (pos_des_lon - pos_cur.longitude)*met_lon
            dist_err = math.sqrt( delta_lat**2 + delta_lon**2 )

        # stop if at last waypoint
        else:
            pub_msg.thrust = 0.0
            pub_msg.yaw_rate = 0.0
            helm_pub.publish(pub_msg)
            return
    
    yaw_des = math.atan2( delta_lat, delta_lon)
    if( yaw_des < 0):
        yaw_des = 2*math.pi + yaw_des

    # compute rate of change of yaw_des
    yaw_des_rate = (yaw_des-yaw_des_old)/cmd_dt;
    yaw_des_old = yaw_des;

    # compute yaw_error and change it to be within [-pi pi]
    yaw_error = yaw_des - yaw_cur
    if ( yaw_error > math.pi ):
        yaw_error = yaw_error-2*math.pi
    elif ( yaw_error < -math.pi ):
        yaw_error = 2*math.pi + yaw_error

    # controller for yaw_rate
    #pub_msg.yaw_rate = kp*yaw_error + yaw_des_rate
    pub_msg.yaw_rate = kp*yaw_error
    
    # if far away, thrusht is at base thrust level
    if( dist_err > 10 ):
        pub_msg.thrust = base_thrust    
    # gradually slow down as wel approach the waypoint
    else:
        pub_msg.thrust = base_thrust*dist_err/10.0

    # adjust thrust according to yaw_error, i.e., adjust yaw first before thrusting forwayd
    pub_msg.thrust = pub_msg.thrust*math.exp(-10*math.fabs(yaw_error) )
    
    # publish message on topic
    helm_pub.publish(pub_msg)
    
    # print("yaw desired = ", yaw_des)
    # print(" yaw current = ", yaw_cur)
    # print("yaw error= ", yaw_error)
    # print("yaw rate desired= ", pub_msg.yaw_rate)
    return
    
def course_publisher():
    global cmd_dt

    rospy.init_node('wapt_control_publisher', anonymous=True)
    rospy.Subscriber("/navsat/fix", NavSatFix,nav_comp)
    
    # /filtered_yaw is the topic on which the data from the complementary filter will arrive
    rospy.Subscriber("/imu/rpy", Vector3Stamped, yaw_callback)
    
    # publishes data on cmd_helm every cmd_dt seconds
    rospy.Timer(rospy.Duration(cmd_dt), control_publisher)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    course_publisher()
