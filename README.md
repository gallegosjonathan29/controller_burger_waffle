# controller_burger_waffle
Kinematic controller.
The following master's thesis presents the theoretical and experimental development
of formation strategy based on the leader-follower method while using the
SBO scheme which represents the separation, direction, and angular deviation
orientation for a set of three mobile robots. It includes the theory and information
needed to confirm the tools used to get the control law and to perform the
experiments.
For the development of the controller, with the leader-follower scheme while
using SBO scheme, the kinematic models of the robots were used. This development
is based on three stages. First on Backstepping techniques where a virtual
control is defined. The second is based on sliding modes where sliding surfaces
were defined. Finally passivity techniques were implemented where an equivalent
stability analysis is constructed. This strategy was designed for managed bounded
disturbances.
In the first experiment, a straight trajectory for the leader was defined which
indirectly obtains the desired trajectory of the follower. In the second experiment,
a comparison of two controllers described in the training strategy is made while
it is compared with the SBO scheme. In the third experiment, the trajectory is
changed into a circular path. The three robots, in the same experiment with the
necessary adjustment in the controllers, are used.
The results, in simulation for the experiments, were used to adjust the controller's
gains with thus avoiding saturation of the controls in real time experiments.
Finally, the proposed controller was validated on an experimental platform. This
platform use ROS (Robot Operating System), programs in python, vision systems
for localization, and computers to be carried out.
