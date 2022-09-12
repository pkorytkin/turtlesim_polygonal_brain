#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dis import dis
import math
from multiprocessing.dummy import Array
import rospy
import geometry_msgs.msg
import turtlesim.msg
import math
from typing import List
class Vector3(geometry_msgs.msg.Vector3):
    def __init__(self,x:float=0,y:float=0,z:float=0):
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
    #def convertToMSG(self):
    #    return geometry_msgs.msg.Vector3(self.x,self.y,self.z)
    def Module(self):
        return float(math.sqrt((self.x)**2+(self.y)**2+(self.z)**2))
    
    def normalize(self):
        dist=self.Module()
        return Vector3(self.x/dist,self.y/dist,self.z/dist)

StartPoseSaved:bool=False
#Глобальная переменная для нынешней позиции
CurrentPose:turtlesim.msg.Pose=turtlesim.msg.Pose()
#Сохранённая стартовая позиция
StartPose:turtlesim.msg.Pose=turtlesim.msg.Pose()
#Сообщение поворота отправляемое в Topic
twist:geometry_msgs.msg.Twist=geometry_msgs.msg.Twist()
#Целевая точка маршрута
CurrentPointID:int=0
#Сохранённые координаты точек маршрута
PointList:List[Vector3]=[]
#PointList=[]
#Rate
r:rospy.Rate
#Число координат выпуклого многоугольника
PointsCount:int=8

sub:rospy.Subscriber
pub:rospy.Publisher
def DistanceVector3(fromPosition:Vector3,toPosition:Vector3=Vector3()):
    return (fromPosition-toPosition).Module()
def DotProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector*toVector
def CrossProduct(fromVector:Vector3,toVector:Vector3):
    return fromVector.x*toVector.y-fromVector.y*toVector.x
def Angle3D(fromVector:Vector3,toVector:Vector3):
    #https://www.wikihow.com/Find-the-Angle-Between-Two-Vectors
    
    fromVector=fromVector.normalize()
    toVector=toVector.normalize()
    

    
    fromVectorAngle=math.atan2(fromVector.y,fromVector.x)
    toVectorAngle=math.atan2(toVector.y,toVector.x)
    
    print("from="+str(fromVectorAngle)+" to="+str(toVectorAngle))
    angle= abs(toVectorAngle-fromVectorAngle)
    if(toVectorAngle<fromVectorAngle):
        angle*=-1
    return angle
    
    #angle=math.atan2(CrossProduct(fromVector,toVector),DotProduct(fromVector,toVector))
    #angle=math.acos(DotProduct(fromVector,toVector))
    #cross=CrossProduct(fromVector,toVector)
    #sign=1
    #if(DotProduct())
    #if (fromVector.x*toVector.y-fromVector.y*toVector.x<0):
    #    sign=-1
    #angle=sign*math.acos(fromVector*toVector/(DistanceVector3(fromVector)*DistanceVector3(toVector))).real
    
    
    
    """while angle>math.pi*2:
        angle-=math.pi*2
    while angle<-math.pi*2:
        angle+=math.pi*2"""
    
    if(angle>math.pi):
        angle-=math.pi*2
    if(angle<-math.pi):
        angle+=math.pi*2
    return angle
#Преобразование локальной позиции черепахи в мировую
def LocalPositionToWorld(Vector:Vector3):
    return CurrentPosition()+Vector
#Мировая позиция в локальную
def WorldToLocalPosition(Vector:Vector3):
    return Vector-CurrentPosition()
#Нынешняя позция
def CurrentPosition():
    #z=CurrentPose.z
    position=Vector3(x=CurrentPose.x,y=CurrentPose.y)
    return position
#Нынешняя ротация в углах эйлера
def CurrentRotation():
    rotation=Vector3(x=0,y=0,z=CurrentPose.theta)
    return rotation
#Нынешней глобальный вектор вперёд
def CurrentGlobalForward():
    return Vector3(x=math.cos(CurrentPose.theta).real,y=math.sin(CurrentPose.theta).real,z=0)
#Локальный вектор вперёд x=1
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
    r=rospy.Rate(60)#60hz
    
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
        if(DistanceToPoint<-0.03):
            twist.linear=-1*CurrentLocalForward()
        elif(abs(AngleFromForwardToPoint)>0.03):
            twist.angular.z=AngleFromForwardToPoint
        else:
            if(DistanceToPoint>0.03):
                twist.linear=CurrentLocalForward()#(CurrentLocalForward()*max(DistanceToPoint,0.01)).convertToMSG()
            elif(DistanceToPoint<-0.03):
                twist.linear=-1*CurrentLocalForward()#(CurrentLocalForward()*(-max(DistanceToPoint,0.01))).convertToMSG()
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
        i:int=0
        tempVector=Vector3
        while i<PointsCount:
            tempVector=Vector3(x=math.cos((2*math.pi/PointsCount)*i).real,y=math.sin((2*math.pi/PointsCount)*i).real)+currentPosition
            PointList.append(tempVector)
            i+=1
            #print(tempVector)
            continue
        i=0
        while i<PointsCount:
            
            print(PointList[i])
            i+=1
            continue
        print()
        Worker()
    except rospy.ROSInterruptException:
        pass