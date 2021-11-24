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
        new_head, new_tail = self._compute_shorter_head_and_tails(maze_problem.initial_state.head, maze_problem.initial_state.tail)
        shorter_robot_head_goal, shorter_robot_tail_goal = self._compute_shorter_head_and_tails(maze_problem.head_goal, maze_problem.tail_goal)
        new_maze = maze_problem.maze_map
        new_maze[maze_problem.initial_state.head[0],maze_problem.initial_state.head[1]] = 0
        new_maze[maze_problem.initial_state.tail[0],maze_problem.initial_state.tail[1]] = 0
        new_maze[new_head[0],new_head[1]] = 2
        new_maze[new_tail[0],new_tail[1]] = 1
        new_maze[maze_problem.head_goal[0],maze_problem.head_goal[1]] = 0
        new_maze[maze_problem.tail_goal[0],maze_problem.tail_goal[1]] = 0
        new_maze[shorter_robot_head_goal[0],shorter_robot_head_goal[1]] = 4
        new_maze[shorter_robot_tail_goal[0],shorter_robot_tail_goal[1]] = 3
        self.new_maze_problem = MazeProblem(maze_map=new_maze,
                                            initial_head=new_head,
                                            initial_tail=new_tail,
                                            head_goal=shorter_robot_head_goal,  # doesn't matter, don't change
                                            tail_goal=shorter_robot_tail_goal)  # doesn't matter, don't change
        self.node_dists = UniformCostSearchRobot().solve(self.new_maze_problem, compute_all_dists=True)
        ################################################################################################################

        assert isinstance(self.node_dists, NodesCollection)

    def _compute_shorter_head_and_tails(self, head, tail):
        # TODO (EX. 13.1): complete code here, delete exception
        if head[0] == tail[0]:
            if head[1] > tail[1]:
                new_head = head[0], head[1] - int(self.k / 2)
                new_tail = tail[0], tail[1] + int(self.k / 2)
            else:
                new_head = head[0], head[1] + int(self.k / 2)
                new_tail = tail[0], tail[1] - int(self.k / 2)
        else:
            if head[0] > tail[0]:
                new_head = head[0] - int(self.k / 2), head[1]
                new_tail = tail[0] + int(self.k / 2), tail[1]
            else:
                new_head = head[0] + int(self.k / 2), head[1]
                new_tail = tail[0] - int(self.k / 2), tail[1]
        return np.asarray(new_head), np.asarray(new_tail)

    def __call__(self, state: MazeState):
        # TODO (EX. 13.3): replace each three dots, delete exception
        shorter_head_location, shorter_tail_location = self._compute_shorter_head_and_tails(state.head, state.tail)
        new_state = MazeState(self.new_maze_problem, head=shorter_head_location, tail=shorter_tail_location)
        if new_state in self.node_dists:
            node = self.node_dists.get_node(new_state)
            return node.g_value
        else:
            return 0  # what should we return in this case, so that the heuristic would be as informative as possible
                      # but still admissible
