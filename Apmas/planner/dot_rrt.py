
import pathlib
import sys 
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
sys.path.append(str(pathlib.Path(__file__).parent.parent))
sys.path.append(str(pathlib.Path(__file__).parent))

from Apmas.agent.dot_agent import DotAgent
from PathPlanning.RRT.rrt import RRT
import matplotlib.pyplot as plt 
import numpy as np
import math

show_animation = True

class GlobalPlanner():
    def __init__(self) -> None:
        self.planned_path = None

class DotRRT(RRT, GlobalPlanner):
    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([-2, 15, -2, 15])
        plt.grid(True)
        plt.pause(0.01)

    def planning(self, animation=True):
        self.planned_path =  super().planning(animation)
        return self.planned_path
   
class LocalPlanner():
    def __init__(self) -> None:
        pass

    def plan(self, pos, goal):
        vx = goal[0] - pos[0] 
        vy = goal[1] - pos[1]
        return vx, vy

class PlanInfo():
    def __init__(self, obs_list, rand_area) -> None:
        self.obs_list = obs_list
        self.rand_area = rand_area

class MyRobot(DotAgent):
    def __init__(self, x, y, info: PlanInfo) -> None:
        super().__init__(x, y)
        self.glb_planner = None
        self.loc_planner = LocalPlanner()
        self.cur_goal = None
        self.map_info = None

    def move(self, v: np.double, u: np.double):
        dt = 0.01
        dx = v * dt
        dy = v * dt
        DotAgent.move(self, dx, dy)

    def setupMP(self, info: PlanInfo):
        self.map_info = info

    def plan(self, goal, info: PlanInfo):
        self.map_info = info
        self.cur_goal = goal
        self.glb_planner = DotRRT(
                                start= self.get(),
                                goal= goal,
                                rand_area = self.map_info.rand_area,
                                obstacle_list= self.map_info.obs_list,
                                #play_area=[0, 10, 0, 14],
                                robot_radius=0.8
                            )        
        path = self.glb_planner.planning() 
        return path 

    def execute(self, goal):
        import collections 

        def eucl_dist_2d(p1, p2):
            dx = (p1[0] - p2[0])
            dy = (p1[1] - p2[1])
            return math.sqrt(dx*dx + dy * dy)

        # obtain a path to goal
        path = collections.deque(self.plan(goal, self.map_info))
        path.pop() # ignore the current point
        traj = collections.deque([])
        if path is None:
            print("Cannot find path")
        else:
            print(path)
            # start moving to goal position if path exists
            while len(path) > 0: 
                curGoal = path.pop()

                dTol = eucl_dist_2d([self.x, self.y], curGoal)
                print("Goal:", curGoal, "Dist:", dTol)
                vx, vy = self.loc_planner.plan([self.x, self.y], curGoal)
                print(vx, vy)
                while dTol > 0.5:    
                    vx, vy = self.loc_planner.plan([self.x, self.y], curGoal)
                    self.x += 0.0001 * vx 
                    self.y += 0.0001 * vy
                    traj.append([self.x, self.y])
                    dTol = eucl_dist_2d([self.x, self.y], curGoal)
                    #print("Goal:", curGoal, "Dist:", dTol)
                    print(dTol)

            # Draw final path
            show_animation = True
            if show_animation:
                self.glb_planner.draw_graph()
                #plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
                plt.plot([x for (x, y) in traj], [y for (x, y) in traj], '-r')
                plt.grid(True)
                plt.pause(0.01)  # Need for Mac
                plt.show()




def main(gx=6.0, gy=10.0):
    print("start" + __file__)

    # ====Search Path with RRT====
    obstacle_list = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
                    (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    
    map_info = PlanInfo(obstacle_list, rand_area=[-2, 15])

    # Set Initial parameters
    my_agent = MyRobot(15.0, 14.0, map_info)

    my_agent.setupMP(map_info)
    my_agent.execute([gx, gy])

    print("Finised " + __file__)

if __name__ == '__main__':
    main()
