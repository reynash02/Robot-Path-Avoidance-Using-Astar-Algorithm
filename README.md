marking.py-raw python implementation
obstacle_marking.py-is run as a ros node

Algorithm used: A* for obstacle marking and neighbour prediction along with path planning
Matplotlib to plot the robot

To change the points to be traversed and edit 'self.points'

Example use case:
points are [(5,7),(10,7),(10,5),(8,5),(8,13)]

![image](https://github.com/user-attachments/assets/9ef931b5-6eaf-4825-95ff-e72032eaa2e3)
Notice how the path from (8,5) to (8,13) should cross a previously traversed path. But it instead goes around it
