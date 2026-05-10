import numpy as np
import math
from matplotlib import pyplot as plt
from PIL import Image
import argparse
import random

from online_motion_planning.Point import Point

def convert_to_path_index(states, edges, smooth_path):

    path_x = [states[i].x for i in smooth_path]
    path_y = [states[i].y for i in smooth_path]

    return path_x, path_y

def fill_path(vertices, edges, edge_goal):
    edges.reverse()
    path = [edge_goal[0][1]]
    next_v = edge_goal[0][0]
    i = 0
    while next_v != 0:
        while edges[i][1] != next_v:
            i += 1
        path.append(edges[i][1])
        next_v = edges[i][0]
        i = 0
    path.append(0)
    edges.reverse()
    path.reverse()
    return vertices, edges, path

def calculate_distance(path, states):
    dist = 0
    for i in range(1, len(path)):
        dist += states[path[i - 1]].dist(states[path[i]])
    
    return dist

class RRT_Star:

    def __init__(self, grid_map, K, delta_q, p, max_distance, q_start, q_goal):
        self.c_space = grid_map
        self.K = K
        self.delta_q = delta_q
        self.p = p
        self.max_distance = max_distance
        self.q_start = q_start
        self.q_goal = q_goal

    def run_RRT_Star(self):
        current_iter = 0
        current_dist = 0
        edge_goal = []
        path_first_time = []
        check_first = False
        path_converge = []
        min_dist = 5
        G_vertex = [self.q_start]
        G_edge = []
        J = [0.0]
        self.log_delta_q = np.log2(self.delta_q)
        self.log_max_distance = np.log2(self.max_distance)
        self.no_improvement_count = 0
        self.max_improvement_count = 100

        c_free_space = []
        for i, j in np.ndindex(self.c_space.shape):
            if self.c_space[i][j] == 0:
                c_free_space.append(Point(i, j))

        for k in range(self.K):
            # print('Iteration: ', k)
            q_rand = self.RAND_CONF(c_free_space, self.p, self.q_goal)
            id, q_near = self.NEAREST_VERTEX(q_rand=q_rand, G=G_vertex)
            q_new = self.NEW_CONF(q_near=q_near, q_rand=q_rand, delta_q=self.delta_q)
            if q_new not in G_vertex and self.IS_FREE_SEGMENT(q_near=q_near, q_new=q_new, c_space=self.c_space, depth=0, max_depth=self.log_delta_q):
                G_vertex, G_edge, J = self.REWIRE_QNEAR_TO_QNEW(G_vertex, G_edge, id, q_near, q_new, J)
                G_vertex, G_edge, J = self.REWIRE_QNEW_FROM_QNEAR(G_vertex, G_edge, q_new, J)
                if check_first == True:
                    _, _, path = fill_path(G_vertex, G_edge ,edge_goal)
                    new_smooth_path = self.smoothing_function(path, G_vertex)
                    new_dist = calculate_distance(new_smooth_path, G_vertex)
                    if new_dist < current_dist:
                        self.no_improvement_count = 0
                        current_dist = new_dist
                    else:
                        self.no_improvement_count += 1
                    if self.no_improvement_count >= self.max_improvement_count:
                        current_iter = k + 1
                        break

                if check_first == False and self.DISTANCE(q_new=q_new, q_goal=self.q_goal) < min_dist:
                        # print('Found path to goal for the first time at iteration: ', k)
                        cost_q_goal = J[G_vertex.index(q_new)] + q_new.dist(self.q_goal)
                        J.append(cost_q_goal)
                        G_vertex.append(self.q_goal)
                        edge_goal = [(len(G_vertex) - 2,len(G_vertex) - 1)]
                        G_edge.append(edge_goal[0])
                        path_first_time = [G_vertex.copy(), G_edge.copy(), k + 1]
                        check_first = True
                        _, _, path = fill_path(path_first_time[0], path_first_time[1], edge_goal)
                        smooth_path = self.smoothing_function(path, path_first_time[0])
                        current_dist = calculate_distance(smooth_path, path_first_time[0])


        if current_iter == 0:
            current_iter = self.K
        path_converge = [G_vertex, G_edge, current_iter]
        return path_first_time, path_converge, edge_goal
    
    def REWIRE_QNEAR_TO_QNEW(self, G_vertex, G_edge, idx, q_near, q_new, J):
        id_min = idx
        cost_q_new = J[idx] + q_near.dist(q_new)
        J.append(cost_q_new)
        Q_near = self.NEAR(G_vertex, q_new)
        for id, q in Q_near:
            if self.IS_FREE_SEGMENT(q, q_new, self.c_space, depth=0, max_depth=self.log_max_distance) and (J[id] + q.dist(q_new) < J[-1]):
                id_min = id
                J[-1] = J[id] + q.dist(q_new)
        
        G_vertex.append(q_new)
        G_edge.append((id_min, len(G_vertex) - 1))

        return G_vertex, G_edge, J
    
    def REWIRE_QNEW_FROM_QNEAR(self, G_vertex, G_edge, q_new, J):
        Q_near = self.NEAR(G_vertex, q_new)
        for id, q in Q_near:
            if self.IS_FREE_SEGMENT(q, q_new, self.c_space, depth=0, max_depth=self.log_max_distance) and (J[-1] + q.dist(q_new) < J[id]):
                J[id] = J[-1] + q.dist(q_new)
                id_parent = self.PARENT(G_edge, id)
                G_edge.remove((id_parent, id))
                G_edge.append((len(G_vertex) - 1, id))
        
        return G_vertex, G_edge, J

    def PARENT(self, G_edge, q_child_idx):
        for edge in G_edge:
            if(edge[1] == q_child_idx):
                return edge[0]
    
    def smoothing_function(self, path, G_vertex):
        smooth_path = [path[-1]]
        current_idx = path[-1]
        first = 0
        while current_idx != path[0]:
            while True:
                segment_dist = G_vertex[current_idx].dist(G_vertex[path[first]])
                # Guard against log2(0) / negative depths for very short segments.
                # Keep at least one recursion level so collision checks remain active.
                max_depth = max(1, int(np.ceil(np.log2(max(segment_dist, 1.0)))))
                is_free = self.IS_FREE_SEGMENT(
                    G_vertex[current_idx],
                    G_vertex[path[first]],
                    self.c_space,
                    depth=0,
                    max_depth=max_depth,
                )
                if is_free:
                    break
                if path[first] + 1  == current_idx:
                    break
                first += 1
            
            current_idx = path[first]
            first = 0
            smooth_path.append(current_idx)

        smooth_path.reverse()
        return smooth_path

    def NEAR(self, G_vertex, q_new):
        Q_near = []
        for id in range(len(G_vertex)):
            point = G_vertex[id]
            if q_new.dist(point) <= self.max_distance:
                Q_near.append((id, point))
        
        return Q_near

    def RAND_CONF(self, c_free_space, p, q_goal):
        if random.random() <= p:
            return q_goal

        else:
            return random.choice(c_free_space)

    def NEAREST_VERTEX(self, q_rand, G):
        q_near = G[0]
        nearest_dist = G[0].dist(q_rand)
        idx = 0
        for i in range(len(G)):
            point = G[i]
            dist = q_rand.dist(point)
            if dist < nearest_dist:
                q_near = point
                idx = i
                nearest_dist = dist

        return idx, q_near

    def NEW_CONF(self, q_near, q_rand, delta_q):
        q_near_vector = q_near.vector(q_rand)
        if q_near_vector.norm() < delta_q:
            return q_rand
        q_near_vector_unit = q_near_vector.unit()
        q_new = q_near.__add__(q_near_vector_unit.scale(self.delta_q))
        return Point(round(q_new.x), round(q_new.y))
    
    def IS_FREE_SEGMENT(self, q_near, q_new, c_space, depth, max_depth):
        if depth >= max_depth:
            return True

        if self.POINT_COLLIDED(q_new, c_space):
            return False
        
        q_mid = (q_near.__add__(q_new)).__truediv__(2)

        if self.POINT_COLLIDED(q_mid, c_space):
            return False

        left_segment_free = self.IS_FREE_SEGMENT(q_near, q_mid, c_space, depth + 1, max_depth)
        if left_segment_free == False:
            return False
        right_segment_free = self.IS_FREE_SEGMENT(q_mid, q_new, c_space, depth + 1, max_depth)
        return right_segment_free
    
    def POINT_COLLIDED(self, q, c_space):
        if q.x < 0 or q.x > c_space.shape[0] or q.y < 0 or q.y > c_space.shape[1]:
            return True
        x = round(q.x)
        y = round(q.y)
        if x == c_space.shape[0] - 1 or y == c_space.shape[1] - 1:
            return c_space[x][y] == 1
        
        return c_space[x][y] == 1 or c_space[x+1][y] == 1 or c_space[x+1][y+1] == 1 or c_space[x][y+1] == 1 # round up error compensation

    def DISTANCE(self, q_new, q_goal):
        return q_goal.dist(q_new)