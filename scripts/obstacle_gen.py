#!/usr/bin/python3
import time
import yarp
import numpy as np
import re
import sys

guiPort = yarp.Port()    
dataPort = yarp.Port()

def setXD(x, y, z, r, conf=0.1):
    bot = yarp.Bottle()
    bot.clear()
    peopleList = bot.addList()
    partList = peopleList.addList()
    partList.addFloat64(x)
    partList.addFloat64(y)
    partList.addFloat64(z)
    partList.addFloat64(r)
    partList.addFloat64(conf)

    return bot

def setNeoObs(x,y,z):
    bot = yarp.Bottle()
    bot.clear()
    bot.addFloat64(x)
    bot.addFloat64(y)
    bot.addFloat64(z)

    obj = yarp.Bottle()
    obj.addString("object");
    obj.addString("o1");
    obj.addFloat64(20.0);
    obj.addFloat64(20.0);
    obj.addFloat64(20.0);
    obj.addFloat64(1000*x);
    obj.addFloat64(1000*y);
    obj.addFloat64(1000*z);
    obj.addFloat64(0.0);
    obj.addFloat64(0.0);
    obj.addFloat64(0.0);
    obj.addInt32(0);
    obj.addInt32(255);
    obj.addInt32(0);
    obj.addFloat64(0.7);
    guiPort.write(obj);

    return bot


def genStaticObstacle(client, center, t=60, useNEO=False, use_right=False):
    start = time.time()
    center[1] = -center[1] if use_right else center[1]
    while time.time()-start < t:
        x = center[0]
        y = center[1]
        z = center[2]
        print(x,y,z)
        result = setNeoObs(x,y,z) if useNEO else setXD(x, y, z, 0.02)
        databot = yarp.Bottle()
        databot.clear()
        databot.addFloat64(x)
        databot.addFloat64(y)
        databot.addFloat64(z)
        dataPort.write(databot)
        client.write(result)
        small_start = time.time()
        while time.time()-small_start < 0.02:
            pass

def genDynamicObstacle(client, start_pos, vel, normal, count=5, t=60, useNEO=False, use_right=False):
    start_pos[1] = -start_pos[1] if use_right else start_pos[1]
    normal[1] = -normal[1] if use_right else normal[1]
    end_pos = [start_pos[0] + vel*normal[0]*t, start_pos[1] + vel*normal[1]*t, start_pos[2] + vel*normal[2]*t]
    for i in range(count):
        for d in [1,-1]:    
            st = start_pos if d == 1 else end_pos
            start = time.time()
            while time.time()-start < t:
                x = st[0] + d * vel * (time.time()-start) * normal[0]
                y = st[1] + d * vel * (time.time()-start) * normal[1]
                z = st[2] + d * vel * (time.time()-start) * normal[2]
                print([x,y,z])
                result = setNeoObs(x,y,z) if useNEO else setXD(x, y, z, 0.02)
                databot = yarp.Bottle()
                databot.clear()
                databot.addFloat64(x)
                databot.addFloat64(y)
                databot.addFloat64(z)
                dataPort.write(databot)
                client.write(result)
                small_start = time.time()
                while time.time()-small_start < 0.02:
                    pass

def genFallingObstacle(client, start_pos, vel, count=5, t=60, useNEO=False, use_right=False):
    start_pos[1] = -start_pos[1] if use_right else start_pos[1]
    for i in range(count):
        start = time.time()
        while time.time()-start < t:
            x = start_pos[0]
            y = start_pos[1]
            z = start_pos[2] - vel * (time.time()-start)
            print([x,y,z])
            result = setNeoObs(x,y,z) if useNEO else setXD(x, y, z, 0.02)
            databot = yarp.Bottle()
            databot.clear()
            databot.addFloat64(x)
            databot.addFloat64(y)
            databot.addFloat64(z)
            dataPort.write(databot)
            client.write(result)
            small_start = time.time()
            while time.time()-small_start < 0.02:
                pass
        while(time.time()-start < t+1):
            print([0,0,0])
            pass


def genProximityObstacle(client, count=5, period=5, t=1, use_right=False):
    start = time.time()
    while time.time() - start < 4:
        pass
    step = 1/(50*t)
    for i in range(count):
        start = time.time()
        mag = 1
        while time.time()-start < t:
            bot = yarp.Bottle()
            bot.clear()
            bot.addFloat64(4 if use_right else 1)
            bot.addFloat64(-0.02)
            bot.addFloat64(0)
            bot.addFloat64(-0.01 if use_right else 0.01)
            bot.addFloat64(0)
            bot.addFloat64(0)
            bot.addFloat64(-1 if use_right else 1)
            for j in range(6):
                bot.addFloat64(0)
            bot.addFloat64(mag)
            mag -= step
            if mag < 0:
                mag = 0
            client.write(bot)            
            small_start = time.time()
            while time.time()-small_start < 0.02:
                pass
        while(time.time()-start < period):
            pass


def main():
    yarp.Network.init() # Initialise YARP
    useNEO = int(sys.argv[1])
    outport_name = "/reactController/neo_obstacles:i" if useNEO else "/visuoTactileWrapper/sensManager:i"
    conf = open('app/conf/reactController.ini')
    part = "left"
    for line in conf.readlines():
        if line.startswith('part'):
            part = re.findall(r"(?<=\()\w+", line)[0]
            break

    conf.close()
    use_right = (part == "right")
    start = time.time()
    outPort = yarp.Port()
    outPort.open('/obstacles:o')
    dataPort.open('/data_obstacles:o')
    guiPort.open('/gui_obst:o')
    yarp.Network.connect(outPort.getName(), outport_name)
    yarp.Network.connect(guiPort.getName(), "/iCubGui/objects")
    proxPort = yarp.Port()
    proxPort.open('/prox_obst:o')
    yarp.Network.connect(proxPort.getName(), "/reactController/proximity_events:i")

    while time.time() - start < 10:
        pass

    obs_type = int(sys.argv[2])
    
    # close the network
    if obs_type == 1:
        genDynamicObstacle(outPort, [-0.319, 0.084, 0.1], 0.08, [0, -1, 0], 3, 3, useNEO, use_right) # exp 1
    elif obs_type == 3:
        genDynamicObstacle(outPort, [-0.119, -0.404, 0.05], 0.05, [0, 1, 0], 10, 3, useNEO, use_right) # exp 3
    elif obs_type == 4:
        genDynamicObstacle(outPort, [-0.319, 0.104, 0.05], 0.05, [0, -1, 0], 5, 3, useNEO, use_right) # exp 4
    elif obs_type == 5:
        genDynamicObstacle(outPort, [-0.319, 0.104, 0.05], 0.05, [0, -1, 0], 5, 3, useNEO, use_right) # exp 4
    elif obs_type == 8:
        genDynamicObstacle(outPort, [-0.159, 0.054, 0.15], 0.05, [1, 0, 0], 5, 3, useNEO, use_right) # exp 4
    elif obs_type == 10:
        genProximityObstacle(proxPort, 5, 5, 3, use_right)
    elif obs_type == 11:
        genProximityObstacle(proxPort, 5, 5, 2, use_right)
    elif obs_type == 100:
        genFallingObstacle(outPort, [-0.249, -0.074, 0.35], 0.1, 3, 4, useNEO, use_right) # exp 4
    elif obs_type == 0:
        genStaticObstacle(outPort, [-0.099,0.0,0.2], 30, useNEO, use_right)
    yarp.Network.fini()

if __name__=="__main__": 
    main()