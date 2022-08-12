#!/usr/bin/python3
import time
import yarp
import numpy as np
import re
import sys
    

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


def streamedTargets(rpc_client, inport, timeout=4, use_cart=False, use_right=False, seed=0):
    np.random.seed(seed)
    return_to_home = False
    x = np.round(np.arange(-0.3, -0.15, 0.05),2) #x = np.round(np.arange(-0.3, -0.05, 0.05),2)
    y = np.round(np.arange(-0.2, 0.0, 0.05),2) #y = np.round(np.arange(-0.2, 0.05, 0.05),2)
    z = np.round(np.arange(0.05, 0.25, 0.05),2) #z = np.round(np.arange(-0.05, 0.35, 0.05),2)
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
        while time.time()-start < timeout:
            pass


def p2pMovement(rpc_client, reps=10, period=9, use_right=False):
    points_right = [[-0.232, 0.258, 0.021, -0.1477, -0.7912, 0.5933, 3.0637], 
                [-0.264, 0.032, 0.034, -0.112, 0.9935, 0.01514, 3.1355]]
    points_left = [[-0.232, -0.258, 0.021, -0.1477, 0.5933, -0.7912, 3.0637], 
                [-0.264, -0.032, 0.034, -0.112, 0.01514, 0.9935, 3.1355]]
    points = points_right if use_right else points_left
    ite = 0
    start = time.time()
    for i in range(reps):
        for point in points:
            while (time.time() - start) < period * ite:
                pass
            ite += 1
            result = yarp.Bottle()
            result.clear()
            result.addString("set_6d")
            l = result.addList()
            l.addFloat64(point[0])
            l.addFloat64(point[1])
            l.addFloat64(point[2])
            k = result.addList()
            k.addFloat64(point[3])
            k.addFloat64(point[4])
            k.addFloat64(point[5])
            k.addFloat64(point[6])
            send_command(rpc_client,result)
            #poseStr = None
            
            #while not poseStr:
            #    poseStr = read_once(inport)


def lemniscateMovement(rpc_client, client, inport, center, step=0.15, reps=10, use_right=False):
    points = [[0.0,  0.000, -0.000], [0.0,  0.037, -0.037], [0.0,  0.075, -0.074], [0.0,  0.113, -0.110], [0.0,  0.152, -0.145], [0.0,  0.192, -0.179], 
            [0.0,  0.233, -0.211], [0.0,  0.276, -0.241], [0.0,  0.321, -0.269], [0.0,  0.367, -0.293], [0.0,  0.415, -0.314], [0.0,  0.464, -0.331], 
            [0.0,  0.515, -0.344], [0.0,  0.567, -0.351], [0.0,  0.620, -0.353], [0.0,  0.672, -0.350], [0.0,  0.723, -0.340], [0.0,  0.773, -0.323], 
            [0.0,  0.820, -0.301], [0.0,  0.864, -0.272], [0.0,  0.903, -0.237], [0.0,  0.937, -0.196], [0.0,  0.964, -0.152], [0.0,  0.984, -0.103], 
            [0.0,  0.996, -0.052], [0.0,  0.996,  0.052], [0.0,  0.984,  0.103], [0.0,  0.964,  0.152], [0.0,  0.937,  0.196], [0.0,  0.903,  0.237], 
            [0.0,  0.864,  0.272], [0.0,  0.820,  0.301], [0.0,  0.773,  0.323], [0.0,  0.723,  0.340], [0.0,  0.672,  0.350], [0.0,  0.620,  0.353], 
            [0.0,  0.567,  0.351], [0.0,  0.515,  0.344], [0.0,  0.464,  0.331], [0.0,  0.415,  0.314], [0.0,  0.367,  0.293], [0.0,  0.321,  0.269], 
            [0.0,  0.276,  0.241], [0.0,  0.233,  0.211], [0.0,  0.192,  0.179], [0.0,  0.152,  0.145], [0.0,  0.113,  0.110], [0.0,  0.075,  0.074], 
            [0.0,  0.037,  0.037], [0.0,  0.000,  0.000], [0.0, -0.037, -0.037], [0.0, -0.075, -0.074], [0.0, -0.113, -0.110], [0.0, -0.152, -0.145], 
            [0.0, -0.192, -0.179], [0.0, -0.233, -0.211], [0.0, -0.276, -0.241], [0.0, -0.321, -0.269], [0.0, -0.367, -0.293], [0.0, -0.415, -0.314], 
            [0.0, -0.464, -0.331], [0.0, -0.515, -0.344], [0.0, -0.567, -0.351], [0.0, -0.620, -0.353], [0.0, -0.672, -0.350], [0.0, -0.723, -0.340], 
            [0.0, -0.773, -0.323], [0.0, -0.820, -0.301], [0.0, -0.864, -0.272], [0.0, -0.903, -0.237], [0.0, -0.937, -0.196], [0.0, -0.964, -0.152], 
            [0.0, -0.984, -0.103], [0.0, -0.996, -0.052], [0.0, -0.996,  0.052], [0.0, -0.984,  0.103], [0.0, -0.964,  0.152], [0.0, -0.937,  0.196], 
            [0.0, -0.903,  0.237], [0.0, -0.864,  0.272], [0.0, -0.820,  0.301], [0.0, -0.773,  0.323], [0.0, -0.723,  0.340], [0.0, -0.672,  0.350], 
            [0.0, -0.620,  0.353], [0.0, -0.567,  0.351], [0.0, -0.515,  0.344], [0.0, -0.464,  0.331], [0.0, -0.415,  0.314], [0.0, -0.367,  0.293], 
            [0.0, -0.321,  0.269], [0.0, -0.276,  0.241], [0.0, -0.233,  0.211], [0.0, -0.192,  0.179], [0.0, -0.152,  0.145], [0.0, -0.113,  0.110], 
            [0.0, -0.075,  0.074], [0.0, -0.037,  0.037]]
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
    i = 0
    for rep in range(reps):
        for point in points:
            while(time.time() - start) < i * step:
                pass
            i += 1
            x = center[0] + gain * point[0]
            y = center[1] + gain * point[1]
            z = center[2] + gain * point[2]
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


def classicScenario(rpc_client, inport, timeout=30, use_right=False):
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
    
    while time.time()-start < timeout:
        pass


def visualScenario(rpc_client, inport, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1]]
    poses = [[0.1,0.0,0.0], [0.05,0.0,0.0], [0.1,0.0,0.0]]
    for pos in poses:
        result = setXD(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
        send_command(rpc_client, result)
        poseStr = None
        
        while not poseStr:
            poseStr = read_once(inport)
        print(poseStr)
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
    conf = open('app/conf/reactController.ini')
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

    move_type = int(sys.argv[1])
    start = time.time()
    while time.time()-start < 1:
        pass
    if move_type == 0:
        visualScenario(outPortRpc, inport, use_right)
        holdPosition(outPortRpc)
    elif move_type == 1:
        classicScenario(outPortRpc, inport, 30, use_right)
    elif move_type == 2:
        streamedTargets(outPortRpc, inport, 3, use_cart, use_right, 10)
    elif move_type == 3:
        p2pMovement(outPortRpc, reps=5, period=6, use_right=use_right)
    elif move_type == 4:
        circularMovement(outPortRpc, outPort, inport, [-0.299, -0.074, 0.1], 0.08, 0.2, t=30, use_right=use_right)
    elif move_type == 5:
        lemniscateMovement(outPortRpc, outPort, inport, [-0.28, -0.17,0.09], step=0.1, reps=5, use_right=use_right)
    # else:
    #     randomMovements(outPortRpc, inport, use_cart, 0)

    start = time.time()
    while time.time()-start < 1:
        pass 
    if move_type > 0:
        stopMovement(outPortRpc)   
    # close the network
    yarp.Network.fini()
