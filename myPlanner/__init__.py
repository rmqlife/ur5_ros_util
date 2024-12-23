from .myBag import MyBag
from .myBound import MyBound
from .myConfig import MyConfig
from .myIK import *
from .myRobot import MyRobot
from .myRobotWithIK import MyRobotWithIK
from .pose_util import *


def init_robot_with_ik():
    return myRobotWithIK.init_robot()