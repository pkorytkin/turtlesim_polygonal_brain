#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
#from std_msgs.msg import String
import geometry_msgs.msg
import turtlesim.msg
import math
#CurrentPosition=turtlesim.msg.Pose

StartPoseSaved=False
CurrentPose=turtlesim.msg.Pose()
StartPose=turtlesim.msg.Pose()
twist=geometry_msgs.msg.Twist()
#Целевая точка маршрута
CurrentPointID=0
PointList=[]
r=any
PointsCount=20
class Vector3:
    x=0
    y=0
    z=0
    def __init__(self,x=0,y=0,z=0):
        self.x=x
        self.y=y
        self.z=z
    def __sub__(self,other):
        if(type(other)==Vector3):
            return Vector3(self.x-other.x,self.y-other.y,self.z-other.z)
    def __add__(self,other):
        if(type(other)==Vector3):
            return Vector3(self.x+other.x,self.y+other.y,self.z+other.z)
    def __mul__(self,other):
        if(type(other)==Vector3):
            return self.x*other.x+self.y*other.y+self.z*other.z
        elif(type(other)==float):
            return Vector3(self.x*other,self.y*other,self.z*other)
    def __str__(self) -> str:
        return "x:"+str(self.x)+" y:"+str(self.y)+" z:"+str(self.z)
    def convertToMSG(self):
        return geometry_msgs.msg.Vector3(self.x,self.y,self.z)

def ModuleVector3(Vector:Vector3):
    return float(math.sqrt((Vector.x)**2+(Vector.y)**2+(Vector.z)**2))
def DistanceVector3(fromPosition:Vector3,toPosition:Vector3=Vector3()):
    return ModuleVector3(fromPosition-toPosition)
def Angle3D(fromVector:Vector3,toVector:Vector3):
    #https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
    print(fromVector)
    print(toVector)
    #print(str(DistanceVector3(fromVector))+" "+str(DistanceVector3(toVector)))
    #print("Angle="+str(math.acos(fromVector*toVector/(DistanceVector3(fromVector)*DistanceVector3(toVector))).real))
    sign=1
    if (fromVector.x*toVector.y-fromVector.y*toVector.x<0):
        sign=-1
    return sign*math.acos(fromVector*toVector/(DistanceVector3(fromVector)*DistanceVector3(toVector))).real
    #dot=fromVector.x*toVector.x+fromVector.y+toVector.y
    #det=fromVector.x*toVector.y-fromVector.y*toVector.x
    #angle=math.atan2(det,dot)
    #return angle
def LocalPositionToWorld(Vector:Vector3):
    return CurrentPosition()+Vector

def WorldToLocalPosition(Vector:Vector3):
    return Vector-CurrentPosition()

def CurrentPosition():
    #z=CurrentPose.z
    position=Vector3(x=CurrentPose.x,y=CurrentPose.y)
    return position
def CurrentRotation():
    rotation=Vector3(x=0,y=0,z=CurrentPose.theta)
    return rotation
def CurrentGlobalForward():
    return Vector3(x=math.cos(CurrentPose.theta).real,y=math.sin(CurrentPose.theta).real,z=0)
def CurrentLocalForward():
    return Vector3(1,0,0)

def subscriber_pose(pose:turtlesim.msg.Pose):
    global CurrentPose
    #rospy.loginfo(pose)
    #Сохранение стартовой позиции
    global StartPoseSaved
    global StartPose
    if(not StartPoseSaved):
        StartPose=pose
        StartPoseSaved=True
        pass
    #Сохранение нынешней позиции
    
    CurrentPose=pose
    #print("Saved CurrentPose="+str(pose))
    pass
def PrepareGlobals():
    global StartPoseSaved
    global CurrentPose
    global StartPose
    global twist
    global CurrentPointID
    global r
    StartPose=turtlesim.msg.Pose()
    #Нужно сохранить стартовую позицию
    global StartPoseSaved
    StartPoseSaved=False
    
    CurrentPose=turtlesim.msg.Pose()
    #print("Reseted CurrentPose="+str(CurrentPose))
    
    
    rospy.init_node("brain")
    r=rospy.Rate(1)#60hz
    
def PrepareWorkers():
    print("Angle="+str(math.degrees(Angle3D(Vector3(1,0,0),Vector3(-1,0,0)))))
    PrepareGlobals()
    global sub
    global pub
    sub=rospy.Subscriber("turtle1/pose",turtlesim.msg.Pose,tcp_nodelay=True,queue_size=1,callback=subscriber_pose)

    pub=rospy.Publisher("turtle1/cmd_vel",geometry_msgs.msg.Twist,tcp_nodelay=True,queue_size=1)

    r.sleep()

def Worker():
    global CurrentPointID
    global PointsCount
    while not(rospy.is_shutdown()):
        CurrentTargetPoint=PointList[CurrentPointID]
        print("Tetha="+str(CurrentPose.theta))
        print("CurrentPointID="+str(CurrentPointID))
        print("CurrentTargetPoint="+str(CurrentTargetPoint))
        print("CurrentPosition="+str(CurrentPosition()))
        VectorToPoint=CurrentTargetPoint-CurrentPosition()
        print("VectorToPoint="+str(VectorToPoint))
        currentGlobalForward=CurrentGlobalForward()
        print("CurrentGlobalForward="+str(currentGlobalForward))
        AngleFromForwardToPoint=Angle3D(currentGlobalForward,VectorToPoint)
        print("AngleFromForwardToPoint="+str(AngleFromForwardToPoint))
        DistanceToPoint=DistanceVector3(CurrentPosition(),CurrentTargetPoint)
        twist.angular=Vector3(0,0,0)
        twist.linear=Vector3()
        print("DistanceToPoint="+str(DistanceToPoint))
        #pub.publish(twist)
        #continue
    
        if(abs(AngleFromForwardToPoint)>0.03):
            twist.angular.z=AngleFromForwardToPoint
            """if(AngleFromForwardToPoint>0):
                twist.angular.z=AngleFromForwardToPoint
            else:
                twist.angular.z=AngleFromForwardToPoint"""
        else:
            if(DistanceToPoint>0.01):
                twist.linear=(CurrentLocalForward()*max(DistanceToPoint,0.005)).convertToMSG()
            if(DistanceToPoint<0.01):
                twist.linear=(CurrentLocalForward()*(-max(DistanceToPoint,0.005))).convertToMSG()
            else:
                CurrentPointID+=1
                if(CurrentPointID==PointsCount):
                    CurrentPointID=0
        
        
        pub.publish(twist)
        print(twist)
        r.sleep()



if __name__ == '__main__':
    try:
        
        PrepareWorkers()
        
        
        

        #Заготавливаем точки маршрута
        
        currentPosition=CurrentPosition()
        i=0
        tempVector=Vector3
        while i<PointsCount:
            tempVector=Vector3(x=math.cos((2*math.pi/PointsCount)*i).real,y=math.sin((2*math.pi/PointsCount)*i).real)+currentPosition
            PointList.append(tempVector)
            i+=1
            #print(tempVector)
            pass
        i=0
        while i<PointsCount:
            
            print(PointList[i])
            i+=1
            pass
        print()
        Worker()
    except rospy.ROSInterruptException:
        pass