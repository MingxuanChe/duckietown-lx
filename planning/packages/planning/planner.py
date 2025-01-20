from typing import List
import networkx as nx
import numpy as np
import itertools
import geometry as geo

from aido_schemas import Context, FriendlyPose
from dt_protocols import (
    PlacedPrimitive,
    PlanningQuery,
    PlanningResult,
    PlanningSetup,
    PlanStep,
    Circle,
    Rectangle,
    SimulationResult,
    simulate,
)

__all__ = ["Planner"]

def closest_node(G, target_pos: np.ndarray):
    dist_min = np.inf
    closest_node = None
    
    for node in G.nodes:
        node_pos = node[1]['q'][:2, 2]
        node_pos = np.array(node_pos)
        dist = np.linalg.norm(node_pos - target_pos)
        if dist < dist_min:
            dist_min = dist
            closest_node = node
            
    return closest_node

def normalize_angle(angle):
    # normalize angle to be between 0 and 360
    if angle >= 360:
        angle -= 360
    elif angle < 0:
        angle += 360

def connect_poses(G, ps: PlanningSetup, a: FriendlyPose, b: FriendlyPose):
    # a plan is a list of plan steps
    plan: List[PlanStep] = []

    # Empty Environment 
    if len(ps.environment) == 4:
        # find turn angle and distance difference (a is start b is goal)
        dist_x = b.x - a.x                                          
        dist_y = b.y - a.y                                          
        theta_goal_turn = np.rad2deg(np.atan2(dist_y, dist_x))  
        distance_start_goal = np.sqrt(dist_x**2+dist_y**2)        
        theta_goal_orient = b.theta_deg - theta_goal_turn 
                
        # write steps - for empty environment, turn toward goal position, move forward to x,y coords, orient at goal
        goal_turn = PlanStep(
            duration = abs(theta_goal_turn/ps.max_angular_velocity_deg_s),
            velocity_x_m_s = 0.0,
            angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_turn > 0 else -ps.max_angular_velocity_deg_s
        )
        goal_move = PlanStep(
            duration = distance_start_goal/ps.max_linear_velocity_m_s,
            velocity_x_m_s = ps.max_linear_velocity_m_s,
            angular_velocity_deg_s = 0.0
        )
        goal_orient = PlanStep(
            duration = abs(theta_goal_orient/ps.max_angular_velocity_deg_s),
            velocity_x_m_s = 0.0,
            angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_orient > 0 else -ps.max_angular_velocity_deg_s
        )

        plan.append(goal_turn)
        plan.append(goal_move)
        plan.append(goal_orient)

    # static
    if len(ps.environment) > 4: 
        # for non-empty environment
        start_node = closest_node(G, a)
        goal_node = closest_node(G, b)
        # find path of nodes from start to goal
        node_path = nx.dijkstra_path(G, start_node, goal_node) 
        pose_data = [G.nodes[node]['q'][:2,2] for node in node_path] 
        current_theta = a.theta_deg

        for node in range(1, len(pose_data)):
            dist_x = pose_data[node][0] - pose_data[node-1][0]                                         
            dist_y = pose_data[node][1] - pose_data[node-1][1]                                         
            theta_goal_turn = np.rad2deg(np.atan2(dist_y, dist_x))  
            distance_start_goal = np.sqrt(dist_x**2+dist_y**2)
            
            turn_angle = theta_goal_turn - current_theta
            # normarize turn angle from 0 to 350
            turn_angle = normalize_angle(turn_angle)
        
            step_turn = PlanStep(
                duration = abs(turn_angle/ps.max_angular_velocity_deg_s),
                velocity_x_m_s = 0.0,
                angular_velocity_deg_s = ps.max_angular_velocity_deg_s if turn_angle > 0 else -ps.max_angular_velocity_deg_s
            )
            step_move = PlanStep(
                duration = distance_start_goal/ps.max_linear_velocity_m_s,
                velocity_x_m_s = ps.max_linear_velocity_m_s,
                angular_velocity_deg_s = 0.0
            )

            plan.append(step_turn)
            plan.append(step_move)

            current_theta = theta_goal_turn

            if node == len(pose_data)-1:
                # from current to goal
                theta_goal_orient = b.theta_deg - current_theta  
                theta_goal_orient = normalize_angle(theta_goal_orient)
                
                goal_orient = PlanStep(
                    duration = abs(theta_goal_orient/ps.max_angular_velocity_deg_s),
                    velocity_x_m_s = 0.0,
                    angular_velocity_deg_s = ps.max_angular_velocity_deg_s if theta_goal_orient > 0 else -ps.max_angular_velocity_deg_s
                )

                # add to plan
                plan.append(goal_orient)
    
    return plan


def collision_check(node, env: List[PlacedPrimitive]):
    node_pos = node[1]['q'][:2, 2]
    clearance = 0.2
    
    if env:
        for primitive in env:
            if isinstance(primitive, Circle):
                center = primitive.center
                radius = primitive.radius
                dist = np.linalg.norm(node_pos - center)
                if dist < radius + clearance:
                    return True
            elif isinstance(primitive, Rectangle):
                center = primitive.center
                size = primitive.size
                x_min = center[0] - size[0] / 2
                x_max = center[0] + size[0] / 2
                y_min = center[1] - size[1] / 2
                y_max = center[1] + size[1] / 2
                if x_min - clearance < node_pos[0] < x_max + clearance and y_min - clearance < node_pos[1] < y_max + clearance:
                    return True
    else:
        # empty environment, no collision
        return False

def network(ps: PlanningSetup):
    # grid size
    grid_size = ps.tolerance_xy_m
    
    # find the bounds of the environment
    H = np.floor((abs(ps.bounds.ymin) + ps.bounds.ymax) / grid_size)
    W = np.floor((abs(ps.bounds.xmin) + ps.bounds.xmax) / grid_size)

    # create a graph
    G = nx.MultiDiGraph()
    for i, j in itertools.product(range(H), range(W)):
        node_name = (i, j)
        q = geo.SE2_from_translation_angle((i*grid_size, j*grid_size), 0)
        G.add_node(node_name, q=q)
        
    # add edges
    for i, j in itertools.product(range(H), range(W)):
        for d in [(0, 1), (1, 0), (1, 1), (-1, 1)]:
            i_new = i + d[0]
            j_new = j + d[1]
            if (i_new, j_new) in G:
                # pose of first node
                q1 = G.nodes[(i,j)]['q']
                # pose of second node
                q2 = G.nodes[(i_new, j_new)]['q']
                # relative pose
                relative_pose = geo.SE2.multiply(geo.SE2.inverse(q1), q2)
                # label
                label = geo.SE2.friendly(relative_pose)
                # add the edge with two properties "label" and "relative pose"
                G.add_edge((i,j), (i_new, j_new), label=label, relative_pose=relative_pose)
                # add inverse edges
                rinv = geo.SE2.inverse(relative_pose)
                linv = geo.SE2.friendly(rinv)
                G.add_edge((i_new, j_new), (i,j), label=linv, relative_pose=rinv)        
    return G

class Planner:
    params: PlanningSetup

    def init(self, context: Context):
        context.info("init()")

    def on_received_set_params(self, context: Context, data: PlanningSetup):
        context.info("initialized")
        self.params = data

        # This is the interval of allowed linear velocity
        # Note that min_velocity_x_m_s and max_velocity_x_m_s might be different.
        # Note that min_velocity_x_m_s may be 0 in advanced exercises (cannot go backward)
        max_velocity_x_m_s: float = self.params.max_linear_velocity_m_s
        min_velocity_x_m_s: float = self.params.min_linear_velocity_m_s

        # This is the max curvature. In earlier exercises, this is +inf: you can turn in place.
        # In advanced exercises, this is less than infinity: you cannot turn in place.
        max_curvature: float = self.params.max_curvature

        # these have the same meaning as the collision exercises
        body: List[PlacedPrimitive] = self.params.body
        environment: List[PlacedPrimitive] = self.params.environment

        # these are the final tolerances - the precision at which you need to arrive at the goal
        tolerance_theta_deg: float = self.params.tolerance_theta_deg
        tolerance_xy_m: float = self.params.tolerance_xy_m

        # For convenience, this is the rectangle that contains all the available environment,
        # so you don't need to compute it
        bounds: Rectangle = self.params.bounds

    def on_received_query(self, context: Context, data: PlanningQuery):
        # A planning query is a pair of initial and goal poses
        start: FriendlyPose = data.start
        goal: FriendlyPose = data.target

        # You start at the start pose. You must reach the goal with a tolerance given by
        # tolerance_xy_m and tolerance_theta_deg.
        
        # create a graph of poses
        G = network(self.params)
        
        # remove collided nodes
        collided_nodes = []
        for node in G.nodes:
            if collision_check(node, self.params.environment):
                collided_nodes.append(node)
        
        for node in collided_nodes:
            G.remove_node(node)

        # You need to declare if it is feasible or not
        # feasible = True
        node_s = closest_node(G, start)
        node_g = closest_node(G, goal)
        feasible = True if nx.has_path(G, node_s, node_g) else False

        if not feasible:
            # If it's not feasible, just return this.
            result: PlanningResult = PlanningResult(False, None)
            context.write("response", result)
            return

        # If it is feasible you need to provide a plan.
        plan = connect_poses(G, self.params, start, goal)

        # A plan is a list of PlanStep
        # plan: List[PlanStep] = []

        # # A plan step consists in a duration, a linear and angular velocity.

        # # For now let's just trace a square of side L at maximum velocity.
        # L = 1.0
        # duration_straight_m_s = L / self.params.max_linear_velocity_m_s
        # duration_turn_deg_s = 90.0 / self.params.max_angular_velocity_deg_s
        # # The plan will be: straight, turn, straight, turn, straight, turn, straight, turn

        # straight = PlanStep(
        #     duration=duration_straight_m_s,
        #     angular_velocity_deg_s=0.0,
        #     velocity_x_m_s=self.params.max_linear_velocity_m_s,
        # )
        # turn = PlanStep(
        #     duration=duration_turn_deg_s,
        #     angular_velocity_deg_s=self.params.max_angular_velocity_deg_s,
        #     velocity_x_m_s=0.0,
        # )

        # plan.append(straight)
        # plan.append(turn)
        # plan.append(straight)
        # plan.append(turn)
        # plan.append(straight)
        # plan.append(turn)
        # plan.append(straight)
        # plan.append(turn)

        result: PlanningResult = PlanningResult(feasible, plan)
        context.write("response", result)
