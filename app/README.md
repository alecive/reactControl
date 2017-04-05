How to operate reactControl code
=================

## Quick howto

1. start `yarpserver`
2. start `iCub_SIM` or real iCub
3. `reactController` [--verbosity n]
4. `yarp rpc /reactController/rpc:i` , e.g. 
 * `set_relative_xd (0.0 0.0 0.1)` [x,y,z in meters]
 * `set_relative_circular_xd 0.1 0.1`   [radius (m) and speed]
 * stop

## rpc port
`yarp rpc /reactController/rpc:i`

N.B. Thrifted rpc: you can use help or help command

Can be used to specify target for end-effector and to set parameters.

## Streaming targets 
Alternatively, the targets can also be read from a streaming port - `/reactController/streamedWholeBodyTargets:i`,
such as from `reaching-supervisor/particlesCartesianTrajectory:o`

## Visualization
`iCubGui` - to visualize target and also additional control point targets - typically end-effector and elbow - green cubes.

Run `iCubGui` and connect all the joint position ports.

Then connect `/reactController/gui:o` and `/iCubGui/objects`

## Logging

### Init - param.log
The param.log file will be saved to a file in the current dir - where you run reactController.

Format: (TODO verify if it is uptodate)

All angles in deg. 

*  nrActiveDOF minPosDOF_1 maxPosDOF_1 … minPosDOF_n maxPosDOF_n
*  minVelDOF_1 maxVelDOF_1 … minVelDOF_n maxVelDOF_n 
*  -1  trajSpeed    tol [for IPOpt]        globalTol    dT
* 0  0  //these were boundSmoothnessFlag and boundSmoothnessValue
* 1 (controlMode == "velocity")         /     2 controlMode == "positionDirect"
* 0     0     0    used to be ipOptMemoryOn, ipOptFilterOn, filterTc  
* 1   (stiffInteraction)    /     0
* 1 (additionalControlPoints)    /     0 

## Continuous logging    
Logged every iteration.

This is streamed to a port: `/reactController/data:o`

Needs to be logged using `yarpdatadumper --name /data/reactCtrl --txTime --rxTime`

`yarp connect /reactController/data:o /data/reactCtrl`

For complete logging using the iCub, see https://github.com/robotology/react-control/tree/master/app/scripts/reactCtrl_log_icub.xml


