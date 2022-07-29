#!/usr/bin/python3
import time
import yarp
import numpy as np
import re
    
def sampleTargetOnCircle(center, radius, res=10):
    points = []

    for theta in np.deg2rad(np.arange(-180, 180, res)):
        x = center[0]
        y = center[1] + radius * np.cos(theta)
        z = center[2] + radius * np.sin(theta)
        points.append((x, y, z))

    return np.array(points)


def randomMovements(rpc_client, inport, use_cart=False, seed=0):
    np.random.seed(seed)
    return_to_home = False
    x = np.round(np.arange(-0.3, -0.05, 0.1),2)
    y = np.round(np.arange(-0.2, 0.05, 0.1),2)
    z = np.round(np.arange(-0.05, 0.35, 0.1),2)
    poses = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    print("Size is ", poses.shape)

    feasibility = [False] * poses.shape[0]
    indexes = np.random.permutation(poses.shape[0])
    feasible_poses = []
    count = 0
    for idx in indexes:
        pos = poses[idx]
        result = yarp.Bottle()
        result.clear()
        if use_cart:
            result.addFloat64(pos[0])
            result.addFloat64(pos[1])
            result.addFloat64(pos[2])
            rpc_client.write(result)
        
        else:
            result.addString("set_xd")
            l = result.addList()
            l.addFloat64(pos[0])
            l.addFloat64(pos[1])
            l.addFloat64(pos[2])
            send_command(rpc_client,result)
        
       # print(result.toString())
        
        poseStr = None
        while not poseStr:
            poseStr = read_once(inport)
        
        posErr = np.sqrt((pos[0]*1000-int(poseStr[0]))**2 + (pos[1]*1000-int(poseStr[1]))**2 + (pos[2]*1000-int(poseStr[2]))**2)
        print(pos, idx, count, np.round(posErr,2))
        count += 1
        if (posErr < 10): # 1 cm
            feasibility[idx] = True
            feasible_poses.append(pos)
        if return_to_home:
            result = yarp.Bottle()
            result.clear()
            result.addString("go_home")
        
            send_command(rpc_client,result)
            poseStr = None
            while not poseStr:
                poseStr = read_once(inport)
            print(' '.join(poseStr[:3]))
        


    print(count, len(feasible_poses))
    print(np.array(feasible_poses))


def streamedTargets(rpc_client, inport, use_cart=False, use_right=False, seed=0):
    np.random.seed(seed)
    return_to_home = False
    x = np.round(np.arange(-0.3, -0.05, 0.1),2)
    y = np.round(np.arange(-0.2, 0.05, 0.1),2)
    z = np.round(np.arange(-0.05, 0.35, 0.1),2)
    poses = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    print("Size is ", poses.shape)
    indexes = np.random.permutation(poses.shape[0])
    count = 0
    for idx in indexes:
        pos = poses[idx]
        result = yarp.Bottle()
        result.clear()
        if use_cart:
            result.addFloat64(pos[0])
            result.addFloat64(-pos[1] if use_right else pos[1])
            result.addFloat64(pos[2])
            rpc_client.write(result)
        
        else:
            result.addString("set_xd")
            l = result.addList()
            l.addFloat64(pos[0])
            l.addFloat64(-pos[1] if use_right else pos[1])
            l.addFloat64(pos[2])
            send_command(rpc_client,result)
        print(pos)
        start = time.time()
        while time.time()-start < 2:
            pass
        

def classicScenario(rpc_client, inport, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.12]]
    for pos in poses:
        result = yarp.Bottle()
        result.clear()
        result.addString("set_xd")
        l = result.addList()
        l.addFloat64(pos[0])
        l.addFloat64(-pos[1] if use_right else pos[1])
        l.addFloat64(pos[2])
        send_command(rpc_client,result)
        poseStr = None
        
        while not poseStr:
            poseStr = read_once(inport)
    
    result = yarp.Bottle()
    result.clear()
    result.addString("set_relative_circular_xd")
    result.addFloat64(0.08)
    result.addFloat64(0.2)
        
    send_command(rpc_client,result)
    '''
    if use_right:
        poses[-1][1] *= -1
    circ_poses = sampleTargetOnCircle(poses[-1], 0.08,1)
    index = 0
    radius = 0.08
    frequency = 0.2
    '''
    start = time.time()
    pos = [0,0,0]
    
    while time.time()-start < 60:
        pass
        '''
        # pos[0]=poses[-1][0]
        # pos[1]=poses[-1][1] + radius*np.cos(2.0*np.pi*frequency*(time.time()-start));
        # pos[2]=poses[-1][2] + radius*np.sin(2.0*np.pi*frequency*(time.time()-start));
        pos = circ_poses[index]
        index += 1
        if index == len(circ_poses):
            index = 0
        result = yarp.Bottle()
        result.clear()
        result.addString("set_xd")
        l = result.addList()
        l.addFloat64(pos[0])
        l.addFloat64(pos[1]) # if use_right else pos[1])
        l.addFloat64(pos[2])
        send_command(rpc_client,result)
        yarp.delay(0.1)
        '''
    result = yarp.Bottle()
    result.clear()
    result.addString("stop")
    send_command(rpc_client,result)
    
    result = yarp.Bottle()
    result.clear()
    result.addString("hold_position")
    send_command(rpc_client,result)


def visualScenario(rpc_client, inport, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1]]#, 
  #  [-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00],
  #  [-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00],
  #  [-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00],
  #  [-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00]]
 #   [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1],
 #   [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1]]
    for pos in poses:
        result = yarp.Bottle()
        result.clear()
        result.addString("set_xd")
        l = result.addList()
        l.addFloat64(pos[0])
        l.addFloat64(-pos[1] if use_right else pos[1])
        l.addFloat64(pos[2])

        send_command(rpc_client, result)
        poseStr = None
        
        while not poseStr:
            poseStr = read_once(inport)
        print(pos)
        
    result = yarp.Bottle()
    result.clear()
    result.addString("hold_position")
    send_command(rpc_client, result)


def send_command(rpc_client, command):
    ans = yarp.Bottle()     
    #print("Command sent: ", command.toString())
    rpc_client.write(command, ans)
    #print ("Reply received: ", ans.toString())
    return ans.toString()


def read_once(inport):
    bottle = yarp.Bottle()
    bottle.clear()
    # read the messge
    inport.read(bottle)
    #print("Received ", bottle.toString())
    return bottle.toString().split(' ')


if __name__=="__main__": 
    yarp.Network.init() # Initialise YARP
    conf = open('../app/conf/reactController.ini')
    for line in conf.readlines(): 
        if (line.startswith('part')):
            part = re.findall(r"(?<=\()\w+", line)[0]
            break

    conf.close()
    use_cart = False
    use_right = (part == "right")
    port_name = "/testCart/target:i" if use_cart else "/reactController/rpc:i"
    port_name2 = "/testCart/finished:o" if use_cart else "/reactController/finished:o" 
    
    inport = yarp.Port()
    # activate ports
    inport.open("/reader")
    yarp.Network.connect(port_name2, inport.getName())

    outPort = yarp.Port() if use_cart else yarp.RpcClient()

    outPort.open('/pokus:o')
    yarp.Network.connect(outPort.getName(), port_name)
    

   # randomMovements(outPort, inport, use_cart, 0)
   # streamedTargets(outPort, inport, use_cart, use_right, 10)
    classicScenario(outPort, inport, use_right)
   # visualScenario(outPort, inport, use_right)

    # close the network
    yarp.Network.fini()
