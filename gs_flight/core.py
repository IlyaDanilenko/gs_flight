#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from rospy import ServiceProxy, Subscriber
from std_msgs.msg import Bool,Int32
from geometry_msgs.msg import Point
from gs_interfaces.srv import Event,Yaw,Position,PositionGPS,Live
from gs_interfaces.msg import PointGPS

"""
0-preflight
1-takeoff
2-landing
3-disarm
"""

class CallbackEvent:
        ALL                = 0
        COPTER_LANDED      = 1
        LOW_VOLTAGE1       = 2
        LOW_VOLTAGE2       = 3
        POINT_REACHED      = 4
        POINT_DECELERATION = 5
        TAKEOFF_COMPLETE   = 6
        ENGINES_STARTED    = 7
        SHOCK              = 8

class FlightController():
    def __init__(self, callback):
        rospy.wait_for_service("geoscan/alive")
        rospy.wait_for_service("geoscan/flight/set_event")
        rospy.wait_for_service("geoscan/flight/set_yaw")
        rospy.wait_for_service("geoscan/flight/set_local_position")
        rospy.wait_for_service("geoscan/flight/set_global_position")
        self.__alive = ServiceProxy("geoscan/alive",Live)
        self.__event_service = ServiceProxy("geoscan/flight/set_event",Event)
        self.__yaw_service=ServiceProxy("geoscan/flight/set_yaw",Yaw)
        self.__local_position_service = ServiceProxy("geoscan/flight/set_local_position",Position)
        self.__global_position_service = ServiceProxy("geoscan/flight/set_global_position",PositionGPS)
        self.__callback_event=Subscriber("geoscan/flight/callback_event",Int32, callback)
    
    def goToLocalPoint(self,x,y,z,time=0):
        if self.__alive().status:
            point = Point()
            point.x = x
            point.y = y
            point.z = z
            return self.__local_position_service(point,time).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def goToPoint(self,latitude,longitude,altitude):
        if self.__alive().status:
            point_gps = PointGPS()
            point_gps.latitude = latitude
            point_gps.longitude = longitude
            point_gps.altitude = altitude
            return self.__global_position_service(point_gps).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")
    
    def updateYaw(self, angle):
        if self.__alive().status:
            return self.__yaw_service(angle).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def preflight(self):
        if self.__alive().status:
            return self.__event_service(0).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def takeoff(self):
        if self.__alive().status:
            return self.__event_service(1).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def landing(self):
        if self.__alive().status:
            return self.__event_service(2).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")

    def disarm(self):
        if self.__alive().status:
            return self.__event_service(3).status
        else:
            rospy.logwarn("Wait, connecting to flight controller")