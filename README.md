This project implements a simple path planning algorithm using A* (A-star) to calculate optimal paths in a 2D grid. The project is built using ROS2 (Robot Operating System 2) and includes basic obstacle handling, grid visualization, and path plotting with `matplotlib`. The path planner is designed to find paths between multiple points on a grid while avoiding obstacles.




obstacle_marking.py-is run as a ros2 node

marking.py-raw python implementation

#Requirements
1) ROS2 (tested on Humble)
2) Python3
3) Numpy for grid management
4) Matplotlib for path visualization
5) Heapq for efficient priority queue handling

#Algorithm used
A* for obstacle marking and neighbour prediction along with path planning


**To change the points to be traversed and edit 'self.points'**


Example use cases:

1) points are [(5,7),(10,7),(10,5),(8,5),(8,13)]

![image](https://github.com/user-attachments/assets/9ef931b5-6eaf-4825-95ff-e72032eaa2e3)

Notice how the path from (8,5) to (8,13) should cross a previously traversed path. But it instead goes around it

2) points are [(3, 5), (10, 10), (15, 8), (4, 17), (18, 2), (12, 6), (7, 13)]

![image](https://github.com/user-attachments/assets/b0b1b075-4a4d-42e4-ba55-b5da8a3b7978)

3) points are [(4, 2), (17, 5), (12, 1), (8, 16), (3, 19), (14, 9), (11, 6), (19, 14), (9, 17), (16, 3), (0, 18), (7, 13), (5, 5), (6, 7), (7, 6)]

![image](https://github.com/user-attachments/assets/782acc0c-5782-415b-b5e3-d97e17259395)

as there is no path from (19,14) to (9,17)

![image](https://github.com/user-attachments/assets/973ea973-7932-4b77-8f87-79bfdc3a7dfc)


