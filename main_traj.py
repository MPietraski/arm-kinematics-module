from modules.trajectory_generator import *


def main():
    """Main function that runs the simulation"""

    traj = MultiAxisTrajectoryGenerator(method="cubic",
                                        interval=[0,10],
                                        ndof=1,
                                        start_pos=[-30],
                                        final_pos=[60])
    
    # generate trajectory
    t = traj.generate(nsteps=20)

    # plot trajectory
    traj.plot()


if __name__ == "__main__":
    main()