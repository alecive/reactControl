#React Control

This is a repo created with the purpose of providing the iCub with a framework for performing a reaching behavior with whole-body obstacle avoidance. It will be deeply linked with the [peripersonal-space repository](github.com/robotology/peripersonal-space), but for now it is a completely independent module.

**The name is really bad, so if you find a better name I will be glad to update the code accordingly!** For the sake of historic reference, `reactController` comes from the fact that this is a purely iterative control that reacts to changes in the environment (i.e. obstacles to avoid).

## Structure of the repository

Up to now (29-10-2015) there is only one module, called `reactController`.

### Documentation of the `reactController`

#### Dependencies:
    yarp
    iKin
    skinDynLib

#### Features:

 * *Online:* trajectories are computed at runtime.
 * *Reactive:* trajectories are updated as the environment changes or is discovered by the robot sensors.
 * *Comprehensive:* there is one single task to solve, rather than 2 different tasks (end-effector vs. whole-body).
 * *Better than the state of the art:* trajectories are computed by taking into account the whole-body thanks to the skin and the peripersonal-space.

It is an *Iterative Inverse Kinematics Solver + Controller*: the optimization step (i.e. `IPOPT`) will take care of everything in a single step: there is *no decoupling* between Inverse Kinematics and Control. The solution to the task is given in the *joint velocity space* (rather than joint configuration space), that are sent directly to the robot. Importantly, there is no distinction between end-effector and whole-body: the obstacles affect every body part in the same way - by limiting the joint velocities.

