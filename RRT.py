#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import numpy as np
import math
from treelib import Node, Tree
import matplotlib.pyplot as plt


def lineParameters(qnew,qnear):
    num = (qnew.y-qnear.y)
    den = (qnew.x-qnear.x)
    if num>0 and den>0:
      m = num/den
    else:
      m =0
    c = qnew.y - m*qnew.x
    return [m,c]

def euclideanDistance(A,B):
    return math.sqrt(math.pow((A.y-B.y),2) + math.pow((A.x-B.x),2))


class Point:
  def __init__(self,x,y):
    self.x =x
    self.y =y

class Rectangle:
    def __init__(self,e,f,g,h):
        self.lowerLeft = Point(e,f)
        self.upperRight= Point(g,h)

class Circle:
    def __init__(self,a,b,r):
        self.centre = Point(a,b)
        self.radius = r

class Obstacle:
    def __init__(self):
        self.no_of_obstacles = 0
        self.rectangleList = []
        self.circleList = []

    def setNumberOfObstacles(self,no):
        self.no_of_obstacles = no

    def setObstacles(self,obs,ch):
        if(ch==1):
            self.rectangleList.append(obs)
        elif(ch==2):
            self.circleList.append(obs)


    def rectObstacleCheck(self,qnew,qnear,rect):
         if qnew.x> rect.lowerLeft.x and qnew.x<rect.upperRight.x and qnew.y>rect.lowerLeft.y and qnew.y<rect.upperRight.y:
             return 0
         else:
           [m,c] = lineParameters(qnew,qnear)
           y1 = m*rect.lowerLeft.x + c
           y2 = m*rect.upperRight.x + c
           Y = [y1, y2]
           for y in Y:
             if y>=rect.lowerLeft.y and y<=rect.upperRight.y:
               return 0
           return 1

    def circleCheck(self,qnew,qnear,circle):
        d = euclideanDistance(qnew,circle.centre)
        if d < circle.radius:
            return 0
        else:
            [m,c] = lineParameters(qnew,qnear)
            A = 1 +math.pow(m,2)
            B = 2*(-circle.centre.x+m*c-circle.centre.y*m)
            C = math.pow(c,2)-math.pow(circle.radius,2)+math.pow(circle.centre.x,2)+math.pow(circle.centre.y,2)-2*circle.centre.y*c
            D = math.pow(B,2)-4*A*C
            if D>=0:
              return 0
            else:
              return 1

    def checkPoint(self,qnode):
        for circle in self.circleList:
            d = euclideanDistance(qnode,circle.centre)
            if d < circle.radius:
                return 0
        for rect in self.rectangleList:
            if qnode.x> rect.lowerLeft.x and qnode.x<rect.upperRight.x and qnode.y>rect.lowerLeft.y and qnode.y<rect.upperRight.y:
                return 0
        return 1


    def checkObstacle(self,qnew,qnear):
        for ob in self.rectangleList:
            res = self.rectObstacleCheck(qnew,qnear,ob)
            if res == 0:
                return 0
        for ob in self.circleList:
            res = self.circleCheck(qnew,qnear,ob)
            if res == 0:
                return 0
        return 1


class RRT:
     def __init__(self):
         self.x_lim=100
         self.y_lim=100
         self.tolerance = 1.5
         self.qdel = 3.5 #step size
         self.obstacles = Obstacle()
         self.tree = Tree()
         self.goal_identifier = -1

     def initRRT(self):
         self.x_lim =int(input("X axis limit"))
         self.y_lim=int(input("Y axis limit"))
         (x,y)=list(map(float, input("Enter start (x,y) coordinates: ").split()))
         self.qstart =Point(x,y)
         (x,y)=list(map(float, input("Enter destination (x,y) coordinates: ").split()))
         self.qend =Point(x,y)
         self.tree.create_node(identifier=0,data=self.qstart)


     def inputObstacle(self):
        no_of_obstacles = int(input("Enter no of obstacles: "))
        self.obstacles.setNumberOfObstacles(no_of_obstacles)
        for i in range(0,self.obstacles.no_of_obstacles):
            flag = 0
            while flag ==0:
                ch = int(input("Enter shape of obstacle: 1.Rectangle 2.Circular: "))
                if ch==1 or ch==2:
                    flag = 1
            if(ch==1):
                e,f=list(map(float, input("Enter lower left (x,y) coordinates: ").split()))
                g,h=list(map(float, input("Enter upper right (x,y) coordinates: ").split()))
                obs = Rectangle(e,f,g,h)
            if(ch==2):
                a,b=list(map(float, input("Enter centre (x,y) coordinates: ").split()))
                r = float(input("Enter radius of circle: "))
                obs = Circle(a,b,r)
            self.obstacles.setObstacles(obs,ch)


     def randPointGen(self):
      x = random.randint(0,self.x_lim)
      y = random.randint(0,self.y_lim)
      randPoint = Point(x,y)
      return randPoint

     def findNearestNode(self,qrand):
          min_dist = math.sqrt(math.pow(self.x_lim,2) + math.pow(self.y_lim,2))
          qnear = -1
          nodeList = self.tree.all_nodes_itr()
          for node in nodeList:
            d = euclideanDistance(qrand,node.data)
            if d<min_dist and d>0:
              min_dist = d
              qnear_idx = node.identifier
          return qnear_idx

     def find_qnew(self,qrand,qnear):
      d = euclideanDistance(qrand,qnear)
      t = self.qdel/d
      (x, y) = (((1 - t) * qnear.x + t * qrand.x), ((1 - t) * qnear.y + t * qrand.y))
      qnew = Point(x,y)
      return qnew

     def inCfree(self,qnew,qnear):
        res = self.obstacles.checkObstacle(qnew,qnear)
        return res

     def checkEndPoints(self):
         if(self.obstacles.checkPoint(self.qstart) == 0):
             return 0
         if(self.obstacles.checkPoint(self.qend) == 0):
             return 0
         return 1


     def evaluateRRT(self):
        self.initRRT()
        self.inputObstacle()
        if(self.checkEndPoints() == 0):
            print("One or more of the end points coincides with the obstacle and cannot be reached")
            return 0
        no_of_nodes = int(input("Enter no of nodes: "))
        k =1
        while k<no_of_nodes:
            qrand = self.randPointGen()
            qnear_idx = self.findNearestNode(qrand)
            if qnear_idx !=-1:
                qnear = self.tree.get_node(qnear_idx).data
                qnew = self.find_qnew(qrand,qnear)
                if(self.inCfree(qnew,qnear)==1):
                    self.tree.create_node(identifier=k,parent=qnear_idx,data=qnew)
                    toGoal = euclideanDistance(qnew,self.qend)
                    if toGoal<self.tolerance:
                        print("Goal reached!")
                        self.goal_identifier = k
                    k=k+1

        return 1

     def plotRRT(self):
        nodeList = self.tree.all_nodes_itr()
        xcoord=[]
        ycoord=[]
        for node in nodeList:
          xcoord.append(node.data.x)
          ycoord.append(node.data.y)

        fig= plt.figure(figsize=(10,10))


        plt.xlim(0,100)
        plt.ylim(0,100)
        plt.plot(xcoord,ycoord,'o')


        for node in nodeList:
          children = self.tree.children(node.identifier)
          parent = node.data
          for child in children:
            plt.plot([parent.x,child.data.x],[parent.y,child.data.y,],'-bo')


        node = self.goal_identifier
        while (node>0):
          parent_idx = self.tree.ancestor(node)
          parent = self.tree.get_node(parent_idx)
          child = self.tree.get_node(node)
          plt.plot([parent.data.x,child.data.x],[parent.data.y,child.data.y,],'-go')
          node = parent_idx


        for rect in self.obstacles.rectangleList:
            rectangle = plt.Rectangle((rect.lowerLeft.x,rect.lowerLeft.y), rect.upperRight.x - rect.lowerLeft.x, rect.upperRight.y - rect.lowerLeft.y, fc='r')
            plt.gca().add_patch(rectangle)

        for circ in self.obstacles.circleList:
            circle = plt.Circle((circ.centre.x, circ.centre.y), circ.radius, color='r')
            plt.gca().add_patch(circle)

        plt.show()



if __name__ == '__main__':
    try:
        rrt = RRT()
        res = rrt.evaluateRRT()
        if res == 1:
            rrt.plotRRT()
        else:
            pass

    except:
        print("ERROR")
        pass
