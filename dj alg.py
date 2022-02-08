from random import random, randrange
import math
import datetime
import matplotlib.pyplot as plt

# This class Dijkstra is the main class of the program
class Dijkstra:
    # __init__ Dijkstra object
    def __init__(self, ox, oy, res, rad):
        self.res = res
        self.rad = rad
        self.calc_map(ox, oy)
        self.motion = self.get_motion_model()
    # this private function calc the position bettwin 2 points
    def calc_pos(self, index, min):
        return(index * self.res + min)
    # this private function calc the x any y position
    def calc_xy_index(self, position, min):
        return(round((position - min) / self.res))
    # this private function calc the x any y position
    def calc_index(self, node):
        return(node.y - self.min_y) * self.x_width + (node.x - self.min_x)
    # this private function verify the x any y node
    def verify_node(self, node):
        px = self.calc_pos(node.x, self.min_x)
        py = self.calc_pos(node.y, self.min_y)
        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y or self.obstacle_map[node.x][node.y]:
            return False
        return True
    # this private function calc the final path algoritem
    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_pos(goal_node.x, self.min_x)], [self.calc_pos(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_pos(n.x, self.min_x))
            ry.append(self.calc_pos(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry
    def planning(self, sx, sy, gx, gy):
        start_node = self.Point(self.calc_xy_index(sx, self.min_x),self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Point(self.calc_xy_index(gx, self.min_x),self.calc_xy_index(gy, self.min_y), 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node
        while True:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]
            plt.plot(self.calc_pos(current.x, self.min_x),self.calc_pos(current.y, self.min_y), "xc")
            plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
            if len(closed_set.keys()) % 10 == 0:
                plt.pause(0.00001)
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find Goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            del open_set[c_id]
            closed_set[c_id] = current
            for move_x, move_y, move_cost in self.motion:
                node = self.Point(current.x + move_x, current.y + move_y, current.cost + move_cost, c_id)
                n_id = self.calc_index(node)
                if n_id in closed_set or not self.verify_node(node):
                    continue
                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost >= node.cost:
                        open_set[n_id] = node
        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry
    class Point:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
    def calc_map(self, ox, oy):
        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        self.x_width = round((self.max_x - self.min_x) / self.res)
        self.y_width = round((self.max_y - self.min_y) / self.res)
        self.obstacle_map = [[False for _ in range(self.y_width)]for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_pos(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_pos(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rad:
                        self.obstacle_map[ix][iy] = True
                        break
    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1], [0, 1, 1],[-1, 0, 1],[0, -1, 1],
                 [-1, -1, math.sqrt(2)],[-1, 1, math.sqrt(2)],
                 [1, -1, math.sqrt(2)],[1, 1, math.sqrt(2)]]
        return motion
# this function build the board of the program
def build_board(ox,oy):
    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(11):
        ox.append(i)
        oy.append(40.0)
    for i in range(10):
        ox.append(i + 30)
        oy.append(40)
    for i in range(10):
        ox.append(i)
        oy.append(30.0)
    for i in range(10):
        ox.append(i + 30)
        oy.append(30)
    for i in range(11):
        ox.append(i)
        oy.append(40.0)
    for i in range(10):
        ox.append(10)
        oy.append(i + 30)
    for i in range(10):
        ox.append(30)
        oy.append(i + 30)
    for i in range(10):
        oy.append(i)
        ox.append(40)
    for i in range(8):
        ox.append(60-i)
        oy.append(51)
    oy.append(51)
    ox.append(52)
    oy.append(52)
    ox.append(52)
    for i in range(8):
        ox.append(60 - i)
        oy.append(32)
    oy.append(32)
    ox.append(52)
    oy.append(33)
    ox.append(52)
    for i in range(8):
        ox.append(60 - i)
        oy.append(42)
    oy.append(42)
    ox.append(52)
    oy.append(43)
    ox.append(52)
    for i in range(8):
        ox.append(60 - i)
        oy.append(4)
    oy.append(4)
    ox.append(52)
    oy.append(5)
    ox.append(52)
    for i in range(8):
        ox.append(60 - i)
        oy.append(22)
    oy.append(22)
    ox.append(52)
    oy.append(23)
    ox.append(52)
    for i in range(8):
        ox.append(60 - i)
        oy.append(12)
    oy.append(12)
    ox.append(52)
    oy.append(13)
    ox.append(52)
# this function printing the board of the program
def print_board(ox, oy, k, sx, sy, o, gx, gy, x):
    plt.grid(True)
    plt.plot(ox, oy, k)
    plt.plot(sx, sy, o)
    plt.plot(gx, gy, x)
    plt.title("Dijkstra Algorithm")
    plt.axis("equal")
# this function printing the Dijkstra algoritem of the program
def print_Dijkstra_board(ox, oy, grid_size, rad, sx, sy, gx, gy):
    a = datetime.datetime.now()
    dijkstra = Dijkstra(ox, oy, grid_size, rad)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)
    b = datetime.datetime.now()
    print(b - a)
    plt.title(b - a)
    plt.plot(rx, ry, "-r")
    plt.pause(0.01)
    plt.show()
# this is the main function of the program
def main():
    print('auto-a points or manuel-m?')
    command = input()
    if command == 'a':
        sx = randrange(0, 60, 1)
        sy = randrange(0, 60, 1)
        gx = randrange(0, 60, 1)
        gy = randrange(0, 60, 1)
    else:
        print('Enter start X point:')
        sx = input()
        print('Enter start Y point:')
        sy = input()
        print('Enter end X point:')
        gx = input()
        print('Enter end Y point:')
        gy = input()
    grid_size = 2.0
    rad = 1.0
    ox, oy = [], []
    build_board(ox,oy)
    print_board(ox, oy, ".k", sx, sy, "og", gx, gy, "xb")
    print_Dijkstra_board(ox, oy, grid_size, rad, sx, sy, gx, gy)
main()