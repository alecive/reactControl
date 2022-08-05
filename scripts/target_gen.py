#!/usr/bin/python3
import time
import yarp
import numpy as np
import re
    

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
        if use_cart:
            result = setXD(pos[0], pos[1], pos[2], True)
            rpc_client.write(result)
        
        else:
            result = setXD(pos[0], pos[1], pos[2], False)
            send_command(rpc_client,result)
        
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
    x = np.round(np.arange(-0.3, -0.05, 0.05),2)
    y = np.round(np.arange(-0.2, 0.05, 0.05),2)
    z = np.round(np.arange(-0.05, 0.35, 0.05),2)
    poses = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    print("Size is ", poses.shape)
    indexes = np.random.permutation(poses.shape[0])
    count = 0
    for idx in indexes:
        pos = poses[idx]
        if use_cart:
            result = setXD(pos[0], -pos[1] if use_right else pos[1], pos[2], True)
            rpc_client.write(result)
        
        else:
            result = setXD(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
            send_command(rpc_client,result)
        
        print(pos)
        start = time.time()
        while time.time()-start < 2:
            pass


def p2pMovement(rpc_client, t=180, use_right=False): # TODO: test with left arm
    points = np.genfromtxt('P2P.txt', delimiter='\t')
    result.clear()
    start = time.time()
    for point in points:
        while (time.time() - start) < t * point[0]:
            pass
        print(point, (time.time() - start)/180)
        result.clear()
        result.addString("set_6d")
        l = result.addList()
        l.addFloat64(point[1])
        l.addFloat64(point[2])
        l.addFloat64(point[3])
        k = result.addList()
        k.addFloat64(point[4])
        k.addFloat64(point[5])
        k.addFloat64(point[6])
        k.addFloat64(point[7])
        send_command(rpc_client,result)
        poseStr = None
        
        while not poseStr:
            poseStr = read_once(inport)


def lemniscateMovement(rpc_client, client, inport, center, t=150, use_right=False): # TODO test with left arm
    points = np.genfromtxt('lemniscate.txt', delimiter='\t')
    if (use_right):
        center[1] *= -1
    result = setXD(center[0], center[1], center[2], False)
    send_command(rpc_client,result)
    poseStr = None    
    while not poseStr:
        poseStr = read_once(inport)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client,result)
    gain = 0.13
    start = time.time()
    for point in points:
        while(time.time() - start) < t * point[0]:
            pass
        x = center[0] + gain * point[1]
        y = center[1] + gain * point[2]
        z = center[2] + gain * point[3]
        print(x, y, z)
        result = setXD(x, y, z, True)
        client.write(result)
    

def setXD(x, y, z, streaming=False):
    bot = yarp.Bottle()
    bot.clear()
    if streaming:
        bot.addFloat64(x)
        bot.addFloat64(y)
        bot.addFloat64(z)
    else:
        bot.addString("set_xd")
        l = bot.addList()
        l.addFloat64(x)
        l.addFloat64(y)
        l.addFloat64(z)
    return bot
    

def holdPosition(rpc_client):
    result = yarp.Bottle()
    result.clear()
    result.addString("hold_position")
    send_command(rpc_client,result)


def stopMovement(rpc_client):
    result = yarp.Bottle()
    result.clear()
    result.addString("stop")
    send_command(rpc_client,result)
    print("Stop sent")


def circularMovement(rpc_client, client, inport, center, radius, frequency, t=60, use_right=False):
    if (use_right):
        center[1] *= -1
    result = setXD(center[0], center[1], center[2], False)
    send_command(rpc_client,result)
    poseStr = None    
    while not poseStr:
        poseStr = read_once(inport)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client,result)

    start = time.time()
    while time.time()-start < t:
        x = center[0]
        y = center[1] + radius*np.cos(2.0*np.pi*frequency*(time.time()-start))
        z = center[2] + radius*np.sin(2.0*np.pi*frequency*(time.time()-start))
        print(x,y,z)
        result = setXD(x, y, z, True)
        client.write(result)
        small_start = time.time()
        while time.time()-small_start < 0.02:
            pass


def classicScenario(rpc_client, inport, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.12]]
    for pos in poses:
        result = setXD(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
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
    start = time.time()
    pos = [0,0,0]
    
    while time.time()-start < 60:
        pass


def visualScenario(rpc_client, inport, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1]]
    for pos in poses:
        result = setXD(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
        send_command(rpc_client, result)
        poseStr = None
        
        while not poseStr:
            poseStr = read_once(inport)
        print(pos)


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
    rpcport_name = "/testCart/target:i" if use_cart else "/reactController/rpc:i"
    inport_name = "/testCart/finished:o" if use_cart else "/reactController/finished:o" 
    
    streamedPort = "/reactController/streamedTargets:i"
    inport = yarp.Port()
    inport.open("/reader")    
    yarp.Network.connect(inport_name, inport.getName())

    outPort = yarp.Port()
    outPort.open('/pokus:o')
    yarp.Network.connect(outPort.getName(), streamedPort)

    outPortRpc = yarp.RpcClient()
    outPortRpc.open('/pokusRpc:o')
    yarp.Network.connect(outPortRpc.getName(), rpcport_name)
    
    stopMovement(outPortRpc)    

    move_type = 5
    start = time.time()
    while time.time()-start < 1:
        pass
    if move_type == 0:
        visualScenario(outPortRpc, inport, use_right)
        holdPosition(outPortRpc)
    elif move_type == 1:
        classicScenario(outPortRpc, inport, use_right)
    elif move_type == 2:
        streamedTargets(outPortRpc, inport, use_cart, use_right, 10)
    elif move_type == 3:
        p2pMovement(outPortRpc, 180, use_right)
    elif move_type == 4:
        circularMovement(outPortRpc, outPort, inport, [-0.299, -0.074, 0.1], 0.08, 0.2, 60, use_right)
    elif move_type == 5:
        lemniscateMovement(outPortRpc, outPort, inport, [-0.28, -0.17,0.09], 15, use_right)
    # else:
    #     randomMovements(outPortRpc, inport, use_cart, 0)

    start = time.time()
    while time.time()-start < 1:
        pass 
    if move_type > 0:
        stopMovement(outPortRpc)   
    # close the network
    yarp.Network.fini()
