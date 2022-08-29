#!/usr/bin/python3
import time
import yarp
import numpy as np
import re
import sys

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


def genStaticObstacle(client, center, t=60, use_right=False):
    start = time.time()
    center[1] = -center[1] if use_right else center[1]
    while time.time()-start < t:
        x = center[0]
        y = center[1]
        z = center[2]
        print(x,y,z)
        result = setXD(x, y, z, 0.02)
        client.write(result)
        small_start = time.time()
        while time.time()-small_start < 0.02:
            pass

def genDynamicObstacle(client, start_pos, vel, normal, count=5, t=60, use_right=False):
    start_pos[1] = -start_pos[1] if use_right else start_pos[1]
    normal[1] = -normal[1] if use_right else normal[1]
    for i in range(count):
        start = time.time()
        while time.time()-start < t:
            x = start_pos[0] + vel * (time.time()-start) * normal[0]
            y = start_pos[1] + vel * (time.time()-start) * normal[1]
            z = start_pos[2] + vel * (time.time()-start) * normal[2]
            print(x,y,z)
            result = setXD(x, y, z, 0.02)
            client.write(result)
            small_start = time.time()
            while time.time()-small_start < 0.02:
                pass


def main():
    yarp.Network.init() # Initialise YARP
    outport_name = "/visuoTactileWrapper/sensManager:i"
    conf = open('app/conf/reactController.ini')
    part = "left"
    for line in conf.readlines():
        if line.startswith('part'):
            part = re.findall(r"(?<=\()\w+", line)[0]
            break

    conf.close()
    use_right = (part == "right")

    outPort = yarp.Port()
    outPort.open('/obstacles:o')
    yarp.Network.connect(outPort.getName(), outport_name)
    # genStaticObstacle(outPort, [-0.299, -0.094, 0.1], 30, use_right)
    genDynamicObstacle(outPort, [-0.299, -0.074, 0.4], 0.1, [0, 0, -1], 5, 5, use_right )
    # close the network
    yarp.Network.fini()

if __name__=="__main__": 
    main()