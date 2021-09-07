"""File for Plotting in 3D and Collision Checking"""
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as Axes3D    

import random
import math
class Node:
  def __init__(self,x,y,z,Parent=None):
    self.x=x
    self.y=y
    self.z=z
    self.Parent=Parent
    self.distance=0
  def __eq__(self, o:object) -> bool:
    return self.x==o.x and self.y==o.y



class Cube:
    def __init__(self, x=0, y=0, z=0, side=1):
        self.x = x
        self.y = y
        self.z = z
        self.side = side

    def set_voxels_in_map(self, x, y, z):
        return (x < self.x + self.side) & (x > self.x) & (y < self.y + self.side) & (y > self.y) & (z < self.z + self.side) & (z > self.z)

    def __in__(self, x, y, z):
        # Check if a given point (x, y, z) is inside the cube
        return (self.x < x < self.x + self.side) & (self.y < y < self.y + self.side) & (self.z < z < self.z + self.side)

class World:
    def __init__(self,start,goal,threshold,x=10,y=10,z=10,max_iteration=2000):
        self.xlim = x
        self.ylim = y
        self.zlim = z
        self.x, self.y, self.z = np.indices((x, y, z))
        self.start=start
        self.goal=goal
        self.threshold=threshold
        self.max_iteration=max_iteration
        self.reached=False
        self.nodes=[]

        self.cubes = []
        self.cube_objects = []

        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')

    def distance(self,node1,node2):
      p0=np.array([node1.x,node1.y,node1.z])
      p1=np.array([node2.x,node2.y,node2.z])
      dis=np.linalg.norm(p0-p1)
      return dis

    def check_collision_of_point(self, x, y, z):
        for cube in self.cube_objects:
            if cube.__in__(x, y, z):
                return True
        return False
    def get_random_node(self):
      new_node=Node(random.random()*20,random.random()*20,random.random()*20)
      
      if self.distance(new_node,self.goal)<=self.threshold:
        new_node=self.goal
        self.goal.Parent=new_node
      return new_node
    

    def check_collision_of_path(self, point1, point2):
      try:

        x1,y1,z1=point1
        x2,y2,z2=point2
      except:
        x1,y1,z1=point1.x,point1.y,point1.z
        x2,y2,z2=point2.x,point2.y,point2.z

      x = lambda alpha: (1-alpha) * x1 + x2
      y = lambda alpha: (1-alpha) * y1 + y2
      z = lambda alpha: (1-alpha) * z1 + z2

      for i in np.arange(0, 1, 0.1):
          for cube in self.cube_objects:
              if cube.__in__(x(i), y(i), z(i)):
                  return True
      return False
    def search(self):
      self.nodes.append(self.start)
      count=0
      while count<self.max_iteration:
        new_node=self.get_random_node()
        if self.check_collision_of_point(new_node.x,new_node.y,new_node.z)==True:
          continue
        for node in self.nodes:
          node.distance=self.distance(node,new_node)
        self.nodes=sorted(self.nodes,key=lambda node:node.distance)
        nearest_node=self.nodes[0]
        for node in self.nodes:
          node.distance=0
        if self.check_collision_of_path(nearest_node,new_node)==False:
          new_node.Parent=nearest_node
          self.nodes.append(new_node)
        else:
          continue
        if nearest_node.x==self.goal.x and nearest_node.y==self.goal.y and nearest_node.z==self.goal.z:
          print('reached')
          self.reached=True
          print(count)
          break
        count+=1
    def get_path(self):
      self.search()
      current_node=self.nodes[-1]
      path=[current_node]
      while(True):
        if current_node.x==self.start.x and current_node.y==self.start.y and current_node.z==self.start.z:
          path.append(current_node)
          break
        else:
          path.append(current_node.Parent)
          current_node=current_node.Parent
      
      path_planned=[(p.x,p.y,p.z) for p in path]
      return path_planned
        



      

    def combine(self):
        cubes = self.cubes.copy()
        print("Cubes: ", len(cubes))
        while len(cubes) != 1:
            cubes[0] = cubes[0] | cubes[-1]
            cubes.pop()

        return cubes[0]

    def add_cube(self, x=0, y=0, z=0, side=1):
        cube = Cube(x, y, z, side)
        self.cube_objects.append(cube)
        self.cubes.append(cube.set_voxels_in_map(self.x, self.y, self.z))


    #-----------Plotting Functions---------------------#
    def plot_world(self):

        voxels = self.combine()
        colors = np.empty(voxels.shape, dtype=object)
        for cube in self.cubes:
            colors[cube] = 'black'

        self.ax.voxels(voxels, facecolors=colors, edgecolor='k')

        plt.title("World")
        plt.xlim([0, self.xlim])
        plt.ylim([0, self.ylim])
        plt.ylim([0, self.zlim])
        plt.legend()
        plt.show()

    def plot_path(self, path):
        # Path is a list of Points
        x = [i[0] for i in path]
        y = [i[1] for i in path]
        z = [i[2] for i in path]

        self.ax.plot3D(x, y, z, 'green', label="Path")
        self.ax.scatter(x, y, z, 'red', label="Waypoint")
    
    #----------- Collision Checking Functions------------#
    
    

if __name__ == "__main__":

    
    start=Node(0,0,0)
    goal=Node(10,10,10)
    world=World(start,goal,1)

    world.add_cube(1, 1, 1, 2)
    world.add_cube(3, 0, 0, 3)

    
    path1=world.get_path()
    
    RRT_3D=World(start,goal,1)
    # Add cubes
    RRT_3D.add_cube(1, 1, 1, 2)
    RRT_3D.add_cube(3, 0, 0, 3)
    path2=RRT_3D.get_path()
    
    # Collision Checking
    point1, point2 = [2, 1, 2], [3, 4, 5]
    print(f"Check Collision of Point: {RRT_3D.check_collision_of_point(*point1)}")
    print(f"Check Collision of Path: {RRT_3D.check_collision_of_path(point1, point2)}")

    # Plotting
    world.plot_path(path1)
    world.plot_world()
    RRT_3D.plot_world()
    RRT_3D.plot_path(path2)
    
