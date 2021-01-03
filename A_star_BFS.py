import heapq
import sys
from termcolor import colored
import math
import numpy as np


class PriorityQueue:
  def __init__(self):
    self.queue = []
  def push(self, value, label):
    heapq.heappush(self.queue, (value, label))
  def pop(self):
    return heapq.heappop(self.queue)
  def is_empty(self): 
    return len(self.queue) == 0

class MapTraffic:
  def __init__(self, atlas, walls, material):
    self.n = len(atlas)
    self.m = len(atlas[0])
    self.atlas = atlas
    self.walls = walls
    self.material = material

  def in_bounds(self, p):
    x,y  = p
    return x >=0 and y>=0 and x<self.n and y<self.m

  def passable(self, p):
    for wall_pos in self.walls:
      if wall_pos == p:
        return False
    return True
  
  def neighbors(self, p):
    x, y = p
    neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
    valid_neighbors = []
    for pos in neighbors:
      if self.in_bounds(pos) and self.passable(pos):
        valid_neighbors.append(pos)
    return valid_neighbors

  def draw(self, show_weight=False, path=[]):
    for i in range(self.n):
      for j in range(self.m):
        if (i,j) in path:
          if (i,j) in self.material:
            if show_weight:
                print("$", end="\t")
            else:
                print("$", end="")
          else:
            if show_weight:
                print("+",end="\t")
            else:
                print("+", end="")        
        elif (i, j) in self.material:
            if show_weight:
              print("●", end="\t")
            else:
              print("●", end="")
        elif self.passable((i,j)):
          if show_weight:
            print("__",end="\t")
          else:
            print("_", end="")
        else:
          if show_weight:
            print("!", end="\t")
          else:
            print("!", end="")
      print()


class SearchAlg:
  def __init__(self, grid, start, energy): # khởi tạo lặp
    self.grid = grid
    self.start = start
    self.goal = (grid.n - 1, grid.m - 1)
    self.energy = energy
    self.lastEnergy = 0
    self.came_from = {}

  def trace_path(self):
    cur_node =  self.goal
    cur_energy = self.lastEnergy
    path = []
    while cur_node != self.start:
      path.append(cur_node)
      curr = self.came_from[(cur_node, cur_energy)]
      cur_node, cur_energy = curr
    
    path.append(self.start)
    path.reverse()
    return path

  def heuristic(self,p1, p2, heu_type="A"):
      return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

  def a_star(self):
    open_list = PriorityQueue()
    gScore = {(self.start, self.energy): 0}
    fScore_start = self.heuristic(self.start, self.goal) # f = g + h = 0 + heu(start, goal)
    open_list.push(fScore_start, (self.start, self.energy))
    self.came_from = {} 

    while not open_list.is_empty():
        item = open_list.pop()
        curr_fScore, curr = item
        curr_node, curr_energy = curr
        if curr_node == self.goal:
            return True
        if curr_energy > 0:
          for next_node in self.grid.neighbors(curr_node):
              if next_node in self.grid.material:
                new_energy = self.energy
              else:
                new_energy = curr_energy - 1
              new_g = gScore[(curr_node, curr_energy)] + 1
              if ((next_node, new_energy) not in gScore) or (new_g < gScore[(next_node, new_energy)]): 
                  gScore[(next_node, new_energy)] = new_g
                  fScore_next_node = gScore[(next_node, new_energy)] + self.heuristic(next_node, self.goal)

                  if next_node == self.goal:
                    self.lastEnergy = new_energy
                  open_list.push(fScore_next_node, (next_node, new_energy))
                  self.came_from[(next_node, new_energy)] = (curr_node, curr_energy)
        
    return False

  def BFS(self):
    cur_energy = self.energy
    queue = []
    queue.append((self.start, cur_energy))
    visited = []    
    self.came_from = {}
    while len(queue) > 0:
      curr = queue.pop(0)      
      cur_node, cur_energy = curr
      visited.append(curr)
      if cur_energy > 0:
        for next_node in self.grid.neighbors(cur_node):
          if next_node in self.grid.material:
              new_energy = self.energy
          else:
              new_energy = cur_energy - 1
          if next_node == self.goal:
            self.lastEnergy = new_energy
            self.came_from[(self.goal, self.lastEnergy)] = (cur_node, cur_energy)
            return True
          elif (next_node, new_energy) not in visited:            
            queue.append((next_node, new_energy))
            self.came_from[(next_node, new_energy)] = (cur_node, cur_energy)
    return False
  
 
A = np.zeros((10, 10))
W = [(0,3), (1,5), (2,2), (2,5), (3,1), (3,7), (4,4), (5,1), (5,6), (6,2), (6,3), (7,5), (7,6), (7,8), (8,0), (8,3), (8,5), (9,4)]
M = [(0,9),(1,4),(4,2),(4,8),(6,4),(7,7),(9,0)]
E = 5
g = MapTraffic(A, W, M)

print("Matrix: ")
g.draw()

print("Maze matrix with weight:: ")
g.draw(show_weight=True)

search = SearchAlg(g, (0,0), E)
print("Thuật toán BFS")
search.BFS()

g.draw(show_weight=True, path=search.trace_path())