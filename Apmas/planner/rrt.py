import numpy as np 
import random
import tool.mathtool as MathTool

class EnvironmentInfo:
    def __init__(self) -> None:
        self.obst_list = None 

class PlannerConfig:
    def __init__(self) -> None:
        self.bound = None
        self.tol_distance = 1e-2
        self.robot_radius = 1
        self.expand_dis = 2
        self.max_iter = 500

class SamplingBased:
    
            
    def __init__(self, config: PlannerConfig, map_info: EnvironmentInfo) -> None:
        self._tree_map = None  
        self._env_info = map_info
        self._config = config

    def sample():
        pass

    def loc_connect():
        pass 

    def check_collision():
        pass 

    def draw_graph(): 
        pass 

    def plan():
        pass

    

class RRT(SamplingBased):
    def sample():
        # if random.randint(0, 100) > self.goal_sample_rate:
        #     rnd = self.Node(
        #         random.uniform(self.min_rand, self.max_rand),
        #         random.uniform(self.min_rand, self.max_rand))
        # else:  # goal point sampling
        #     rnd = self.Node(self.end.x, self.end.y)
        rnd = random.randint(0, 100)
        return rnd
    
    def check_collision():
        pass

    def get_nearest_mode(): 
        pass 

    def steer(nearest_node, rnd_node, dist):
        pass

    def plan(self, animation = False):
        """
        rrt path planning

        animation: flag for animation on or off
        """

        self.node_list = [self.start]
        for i in range(self._config.max_iter):
            # get new sample
            rnd_node = self.sample()
            
            nearest_ind = MathTool.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            # steer new node
            new_node = self.steer(nearest_node, rnd_node, self._config.expand_dis)

            # check feasibility
            if MathTool.check_if_outside_play_area(new_node, self.play_area) and \
               self.check_collision(
                   new_node, self.obstacle_list, self.robot_radius):
                self.node_list.append(new_node)


            # if animation and i % 5 == 0:
            #     self.draw_graph(rnd_node)

            # if self.calc_dist_to_goal(self.node_list[-1].x,
            #                           self.node_list[-1].y) <= self.expand_dis:
            #     final_node = self.steer(self.node_list[-1], self.end,
            #                             self.expand_dis)
            #     if self.check_collision(
            #             final_node, self.obstacle_list, self.robot_radius):
            #         return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path