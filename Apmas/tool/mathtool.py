import math 
import numpy as np
import matplotlib.pyplot as plt 

def plot_circle(x, y, size, color="-b"):  # pragma: no cover
    deg = list(range(0, 360, 5))
    deg.append(0)
    xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
    yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
    plt.plot(xl, yl, color)

def get_nearest_node_index(node_list, rnd_node):
    dlist = [(node.x - rnd_node.x)**2 + (node.y - rnd_node.y)**2
                for node in node_list]
    minind = dlist.index(min(dlist))

    return minind

def check_if_outside_play_area(node, play_area):

    if play_area is None:
        return True  # no play_area was defined, every pos should be ok

    if node.x < play_area.xmin or node.x > play_area.xmax or \
        node.y < play_area.ymin or node.y > play_area.ymax:
        return False  # outside - bad
    else:
        return True  # inside - ok

def check_collision(node, obstacleList, robot_radius):

    if node is None:
        return False

    for (ox, oy, size) in obstacleList:
        dx_list = [ox - x for x in node.path_x]
        dy_list = [oy - y for y in node.path_y]
        d_list = [dx * dx + dy * dy for (dx, dy) in zip(dx_list, dy_list)]

        if min(d_list) <= (size+robot_radius)**2:
            return False  # collision

    return True  # safe

def calc_distance_and_angle(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    d = math.hypot(dx, dy)
    theta = math.atan2(dy, dx)
    return d, theta
