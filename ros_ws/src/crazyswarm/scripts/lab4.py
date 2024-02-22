import numpy as np
from blocklyTranslations import *
import matplotlib.pyplot as plt
import argparse

Hz = 20

def build_argparser(parent_parsers=[]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        parents=parent_parsers
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    return parser

def rose_curve(groupState, a, n, d):
    crazyflies = groupState.crazyflies
    timeHelper = groupState.timeHelper

    # TODO: Step 1, Parametric Equations of Rose Curve
    # make the appropriate parametric equations
    def fx(t):
        return 0
    
    def fy(t):
        return 0
    
    def fz(t):
        return 0
    
    #TODO: Step 2, Set appropriate start and end times (min and max values of t)
    # You may need to add an if statement...
    start_time = 0
    end_time = 0

    #TODO: Step 3, set appropriate time scaling factor
    time_stretch_factor = 1

    # Stretch end time by time stretch factor
    end_time = end_time * time_stretch_factor

    # Timesteps is an array of evenly spaced numbers from start to end 1/Hz apart
    timesteps = np.arange(start_time, end_time, 1/Hz)

    # Gather the starting positions of each crazyflie
    initialPositions = [cf.position() for cf in crazyflies]

    # Loop through every timestep
    for t in timesteps:
        # Loop through every crazyflie and its initial position at the start of the trajecotry
        for initPos, cf in zip(initialPositions, crazyflies):
            # Transform time back to original range
            t = t / time_stretch_factor

            # Get position from trajectory
            position = (fx(t), fy(t), fz(t))

            # subtract out position of start of trajectory
            position = position - np.array((fx(timesteps[0]), fy(timesteps[0]), fz(timesteps[0])))

            # Add initial position of crazyflie
            position = position + np.array(initPos)

            # Send command to crazyflie
            cf.cmdPosition(position)

        # Sleep for 1/Hz seconds
        timeHelper.sleepForRate(Hz)

    # Tell each crazyflie we are stopping streaming low-level commands
    for cf in crazyflies:
        cf.notifySetpointsStop()


# TODO: Part 3: Your function here:



def main():
    parser = build_argparser()
    if isinstance(args, str):
        args = args.split()
    args, unknown = parser.parse_known_args(args)
    sim = args.sim

    if sim:
        # Use sim version of crazyswarm
        from pycrazyswarm import Crazyswarm
        swarm = Crazyswarm()
        crazyflies = swarm.allcfs.crazyflies
        timeHelper = swarm.timeHelper

        groupState = SimpleNamespace(crazyflies=crazyflies, timeHelper=timeHelper)
    else:
        # Use ROS and Crazyswarm2
        from crazyflie_py import Crazyswarm
        swarm = Crazyswarm()
        crazyflies = swarm.allcfs.crazyflies
        timeHelper = swarm.timeHelper

        groupState = SimpleNamespace(crazyflies=crazyflies, timeHelper=timeHelper) 
    
    takeoff(groupState, height=1, duration=3)
    rose_curve(groupState, 1, 1, 3)
    land(groupState, 0, 3)
    plt.show()

if __name__ == '__main__':
    main()