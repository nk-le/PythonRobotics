import math 
import numpy as np
from enum import Enum 

class ROBOT_TYPE(Enum):
    CIRCLE = 1
    RECTANLE = 2

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = ROBOT_TYPE.RECTANLE

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [5.0, 4.0],
                            [5.0, 5.0],
                            [5.0, 6.0],
                            [5.0, 9.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, ROBOT_TYPE):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value

class Point2D:
    def __init__(self,x, y) -> None:
        self.x = np.double(x)
        self.y = np.double(y)

    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y))

def agentCollision(p1: Point2D, p2: Point2D, tolD: np.double):
    d = Point2D.distance(p1, p2)
    return d < tolD



def main(gx=10.0, gy=10.0, robot_type= ROBOT_TYPE.CIRCLE):
    import matplotlib.pyplot as plt 
    config = Config()

    ob = config.ob
    goal = np.array([gx, gy])
    while True:
        
        if True:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            #plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            #plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            #plot_robot(x[0], x[1], x[2], config)
            #plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check reaching goal
        # dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        # if dist_to_goal <= config.robot_radius:
        #     print("Goal!!")
        #     break

    print("Done")
    # if show_animation:
    #     plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
    #     plt.pause(0.0001)

    plt.show()

#main()



def test_collision():
    p1 = Point2D(0.5, 1.5)
    p2 = Point2D(0.6, 1.6)
    print(Point2D.distance(p1, p2))
    print(agentCollision(p1, p2, 1))
    

test_collision()