from Robot import *
from MazeProblem import *
from Animation import Animation
from Heuristics import *
from Utilities import *
from Experiments import *


if __name__ == "__main__":
    # test_robot(BreadthFirstSearchRobot, [0,1,2,3,4,5])
    # test_robot(UniformCostSearchRobot, [0,1,2,3,4,5])
    # test_robot(WAStartRobot, [99], heuristic=tail_manhattan_heuristic)
    # solve_and_display(BreadthFirstSearchRobot, 1)
    # solve_and_display(UniformCostSearchRobot, 99)
    solve_and_display(WAStartRobot, 99, heuristic=tail_manhattan_heuristic)
