import sys, time, math 
sys.path.append("./")
from backend.Testing import initSim, initRobot, trajPlanner
from backend.KoalbyHumanoid.Robot import Joints
from backend.LimbTrajectories.rightLegTraj import * # Old Traj Code, see bottom comment

tj = trajPlanner.TrajPlannerNew([[4, 28, -10, 0], [0, 0, 0, 0], [10, 10, 10, 10]])
fulltraj = tj.getNTraj(13.5, 10, 5)
print("fulltraj")
print(fulltraj)