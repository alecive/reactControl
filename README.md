# react-control

<a href="https://zenhub.com"><img src="https://raw.githubusercontent.com/ZenHubIO/support/master/zenhub-badge.png"></a>

This repository contains a controller for the iCub for performing reaching with simultaneous reactive whole-body obstacle avoidance. The module thus encapsulates the functionality of an inverse kinematics solver and the actual controller that generates the commands to reach a position in Cartesian space. The obstacles - perceived visually or through the iCub's sensitive skin - are incorporated on the run as additional constraints.

The reaching controller is standalone in the `reactController` module here, but to acquire the information about the visual/tactile obstacles, it relies on communication with modules from the [peripersonal-space repository](https://github.com/robotology/peripersonal-space).

## Structure of the repository

The core module is the `reactController`.

There are other modules in the repository which have so far an internal/development character. They are `jointVelCtrlIdentSimple`, `reactController-sim`, `reactController-sim-enhanced` under `modules` and there is another internal piece of code in `tests` (which implements the same controller formulation for the optimizer with the simplest possible infrastructure needed to run on the robot). 

### Documentation of the `reactController`

#### Dependencies:
    yarp
    iKin
    skinDynLib
    ctrlLib

#### Principles of operation:

This modules parallels the functionality of the iCub [Cartesian interface] (http://wiki.icub.org/brain/icub_cartesian_interface.html), but both the solver and controller are encapsulated in the single problem formulation. It is an *Iterative Inverse Kinematics Solver + Controller*: the optimization step (i.e. `IPOPT`) will take care of everything in a single step: there is *no decoupling* between Inverse Kinematics and Control. The solution to the task is given in the *joint velocity space* (rather than joint configuration space), that are sent directly to the robot. Importantly, there is no distinction between end-effector and whole-body: the obstacles affect every body part in the same way - by limiting the joint velocities

## Authors

 * [Alessandro Roncone (@alecive)](https://github.com/alecive)
 * [Matej Hoffmann (@matejhof)](https://github.com/matejhof)
 * [Ugo Pattacini (@pattacini)](https://github.com/pattacini)




