import numpy as np
from MazeProblem import MazeState, MazeProblem, compute_robot_direction
from Robot import UniformCostSearchRobot
from GraphSearch import NodesCollection


def tail_manhattan_heuristic(state: MazeState):
    # TODO (EX 7.2), implement heuristic, delete exception
    forward_cost = state.maze_problem.forward_cost
    tail = state.tail
    goal_tail = state.maze_problem.tail_goal
    # |x0-x1|+|y0-y1|
    manhattan = np.abs(tail[0]-goal_tail[0])+np.abs(tail[1]-goal_tail[1])
    return manhattan*forward_cost


def center_manhattan_heuristic(state: MazeState):
    # TODO (EX 9.2), implement heuristic, delete exception
    forward_cost = state.maze_problem.forward_cost
    tail = state.tail
    head = state.head
    goal_tail = state.maze_problem.tail_goal
    goal_head = state.maze_problem.head_goal
    # center_x = |head_x - tail_x|
    center = ((tail[0] + head[0]) / 2, (tail[1] + head[1]) / 2)
    goal_center = ((goal_tail[0] + goal_head[0]) / 2, (goal_tail[1] + goal_head[1]) / 2)
    manhattan = np.abs(center[0]-goal_center[0])+np.abs(center[1]-goal_center[1])
    return manhattan*forward_cost


class ShorterRobotHeuristic:
    def __init__(self, maze_problem: MazeProblem, k):
        assert k % 2 == 0, "odd must be even"
        assert maze_problem.length - k >= 3, f"it is not possible to shorten a {maze_problem.length}-length robot by " \
                                             f"{k} units because robot length has to at least 3"
        self.k = k
        ################################################################################################################
        # TODO (EX. 13.2): replace all three dots, delete exception
        raise NotImplemented
        shorter_robot_head_goal, shorter_robot_tail_goal = ...
        self.new_maze_problem = MazeProblem(maze_map=...,
                                            initial_head=...,
                                            initial_tail=...,
                                            head_goal=shorter_robot_head_goal,  # doesn't matter, don't change
                                            tail_goal=shorter_robot_tail_goal)  # doesn't matter, don't change
        self.node_dists = ...().solve(..., compute_all_dists=True)
        ################################################################################################################

        assert isinstance(self.node_dists, NodesCollection)

    def _compute_shorter_head_and_tails(self, head, tail):
        # TODO (EX. 13.1): complete code here, delete exception
        raise NotImplemented

    def __call__(self, state: MazeState):
        # TODO (EX. 13.3): replace each three dots, delete exception
        raise NotImplemented
        shorter_head_location, shorter_tail_location = ...
        new_state = MazeState(..., head=..., tail=...)
        if new_state in self.node_dists:
            node = self.node_dists.get_node(new_state)
            return ...
        else:
            return ...  # what should we return in this case, so that the heuristic would be as informative as possible
                        # but still admissible
