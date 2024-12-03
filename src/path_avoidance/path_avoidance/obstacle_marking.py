import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from heapq import heappop,heappush

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.grid_size=(20,20)
        self.grid=np.zeros(self.grid_size,dtype=int)
        self.path=[]
        self.points=[(5,7),(10,7),(10,5),(8,5),(8,13)]  
        self.plan_paths()

    def mark_obstacle(self, path):
        for x, y in path:
            self.grid[y,x]=1

    def heuristic(self,a,b):
        return abs(a[0]-b[0])+abs(a[1]-b[1])

    def a_star(self,start,goal):
        open_set=[]
        heappush(open_set,(0+self.heuristic(start,goal),0,start,[]))
        visited=set()
        while open_set:
            _,cost,current,path=heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            path=path+[current]
            if current==goal:
                return path
            for dx,dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                neighbor=(current[0]+dx,current[1]+dy)
                if (0<=neighbor[0]<self.grid_size[1] and 0<=neighbor[1]<self.grid_size[0] and neighbor not in visited and self.grid[neighbor[1],neighbor[0]]==0):
                    heappush(open_set,(cost+1+self.heuristic(neighbor,goal),cost+1,neighbor,path))
        return None

    def plot_grid(self):
        plt.imshow(self.grid,cmap="gray_r")
        plt.xticks(np.arange(self.grid_size[1]),labels=np.arange(self.grid_size[1]))
        plt.yticks(np.arange(self.grid_size[0]),labels=np.arange(self.grid_size[0]))
        plt.grid(color="black",linestyle="-",linewidth=0.5)

    def plot_path(self,path,goal):
        self.plot_grid()
        x,y=zip(*path)
        plt.plot(x,y,marker="o",color="blue")
        
        goal_x,goal_y=goal
        plt.plot(goal_x,goal_y,marker="o",color="red",markersize=8)
        plt.pause(0.1)

    def plan_paths(self):
        previous_path=[]
        for i in range(1,len(self.points)):
            start=self.points[i-1]
            goal=self.points[i]
            self.get_logger().info(f'Calculating path from {start} to {goal}...')
            path=self.a_star(start, goal)
            
            if path is None:
                self.get_logger().error(f'No valid path found from {start} to {goal}!')
                break

            self.get_logger().info(f'Found path from {start} to {goal}: {path}')
            self.mark_obstacle(path)
            self.plot_path(path,goal)

        self.get_logger().info("Final destination reached.")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node=PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
