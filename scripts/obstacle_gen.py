#!/usr/bin/python3
import time
import yarp
import re
import sys

guiPort = yarp.Port()
dataPort = yarp.Port()


def set_xd(x, y, z, r, conf=0.1):
    bot = yarp.Bottle()
    bot.clear()
    people_list = bot.addList()
    part_list = people_list.addList()
    part_list.addFloat64(x)
    part_list.addFloat64(y)
    part_list.addFloat64(z)
    part_list.addFloat64(r)
    part_list.addFloat64(conf)

    return bot


def set_neo_obs(x, y, z):
    bot = yarp.Bottle()
    bot.clear()
    bot.addFloat64(x)
    bot.addFloat64(y)
    bot.addFloat64(z)

    obj = yarp.Bottle()
    obj.addString("object")
    obj.addString("o1")
    obj.addFloat64(20.0)
    obj.addFloat64(20.0)
    obj.addFloat64(20.0)
    obj.addFloat64(1000 * x)
    obj.addFloat64(1000 * y)
    obj.addFloat64(1000 * z)
    obj.addFloat64(0.0)
    obj.addFloat64(0.0)
    obj.addFloat64(0.0)
    obj.addInt32(255)
    obj.addInt32(0)
    obj.addInt32(0)
    obj.addFloat64(0.7)
    guiPort.write(obj)

    return bot


def gen_static_obstacle(client, center, t=60, use_neo=False, use_right=False):
    start = time.time()
    while time.time() - start < 10:
        pass
    start = time.time()
    center[1] = -center[1] if use_right else center[1]
    while time.time() - start < t:
        x = center[0]
        y = center[1]
        z = center[2]
        print(x, y, z)
        result = set_neo_obs(x, y, z) if use_neo else set_xd(x, y, z, 0.02)
        databot = yarp.Bottle()
        databot.clear()
        databot.addFloat64(x)
        databot.addFloat64(y)
        databot.addFloat64(z)
        dataPort.write(databot)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def gen_dynamic_obstacle(client, start_pos, vel, normal, count=5, t=60, use_neo=False, use_right=False):
    start_pos[1] = -start_pos[1] if use_right else start_pos[1]
    normal[1] = -normal[1] if use_right else normal[1]
    end_pos = [start_pos[0] + vel * normal[0] * t, start_pos[1] + vel * normal[1] * t,
               start_pos[2] + vel * normal[2] * t]
    for i in range(count):
        for d in [1, -1]:
            st = start_pos if d == 1 else end_pos
            start = time.time()
            while time.time() - start < t:
                x = st[0] + d * vel * (time.time() - start) * normal[0]
                y = st[1] + d * vel * (time.time() - start) * normal[1]
                z = st[2] + d * vel * (time.time() - start) * normal[2]
                print([x, y, z])
                result = set_neo_obs(x, y, z) if use_neo else set_xd(x, y, z, 0.02)
                databot = yarp.Bottle()
                databot.clear()
                databot.addFloat64(x)
                databot.addFloat64(y)
                databot.addFloat64(z)
                dataPort.write(databot)
                client.write(result)
                small_start = time.time()
                while time.time() - small_start < 0.02:
                    pass

def gen_dynamic_1way_obstacle(client, start_pos, vel, normal, count=5, t=60, use_neo=False, use_right=False):
    start_pos[1] = -start_pos[1] if use_right else start_pos[1]
    normal[1] = -normal[1] if use_right else normal[1]
    end_pos = [start_pos[0] + vel * normal[0] * t, start_pos[1] + vel * normal[1] * t,
               start_pos[2] + vel * normal[2] * t]
    for i in range(count):
        start = time.time()
        while time.time() - start < t:
            x = start_pos[0] + vel * (time.time() - start) * normal[0]
            y = start_pos[1] + vel * (time.time() - start) * normal[1]
            z = start_pos[2] + vel * (time.time() - start) * normal[2]
            print([x, y, z])
            result = set_neo_obs(x, y, z) if use_neo else set_xd(x, y, z, 0.02)
            databot = yarp.Bottle()
            databot.clear()
            databot.addFloat64(x)
            databot.addFloat64(y)
            databot.addFloat64(z)
            dataPort.write(databot)
            client.write(result)
            small_start = time.time()
            while time.time() - small_start < 0.02:
                pass
                

def gen_falling_obstacle(client, start_pos, vel, count=5, t=60, use_neo=False, use_right=False):
    start_pos[1] = -start_pos[1] if use_right else start_pos[1]
    for i in range(count):
        start = time.time()
        while time.time() - start < t:
            x = start_pos[0]
            y = start_pos[1]
            z = start_pos[2] - vel * (time.time() - start)
            print([x, y, z])
            result = set_neo_obs(x, y, z) if use_neo else set_xd(x, y, z, 0.02)

            databot = yarp.Bottle()
            databot.clear()
            databot.addFloat64(x)
            databot.addFloat64(y)
            databot.addFloat64(z)
            dataPort.write(databot)
            client.write(result)
            small_start = time.time()
            while time.time() - small_start < 0.02:
                pass
        while time.time() - start < t + 1:
            print([0, 0, 0])
            pass


def gen_proximity_obstacle(client, count=5, period=5, t=1, use_right=False):
    start = time.time()
    while time.time() - start < 5:
        pass
    step = 1 / (50 * t)/5
    for i in range(count):
        start = time.time()
        mag = 0.5
        while time.time() - start < t:
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
            while time.time() - small_start < 0.02:
                pass
        while time.time() - start < period:
            pass


def main():
    yarp.Network.init()  # Initialise YARP
   # print(sys.argv[1], bool(sys.argv[1]), int(sys.argv[1]), bool(int(sys.argv[1])))
    use_neo = int(sys.argv[1])
    outport_name = "/reactController/neo_obstacles:i" if use_neo else "/visuoTactileWrapper/sensManager:i"
    if not use_neo:
        outport_name_data = "/reactController/sensManager:i"
    conf = open('app/conf/reactController.ini')
    part = "left"
    for line in conf.readlines():
        if line.startswith('part'):
            part = re.findall(r"(?<=\()\w+", line)[0]
            break
    print(outport_name)
    conf.close()
    use_right = (part == "right")
    start = time.time()
    out_port = yarp.Port()
    out_port.open('/obstacles:o')
    dataPort.open('/data_obstacles:o')
    guiPort.open('/gui_obst:o')
    yarp.Network.connect(out_port.getName(), outport_name)
    if not use_neo:
        yarp.Network.connect(out_port.getName(), outport_name_data)
    yarp.Network.connect(guiPort.getName(), "/iCubGui/objects")
    prox_port = yarp.Port()
    prox_port.open('/prox_obst:o')
    yarp.Network.connect(prox_port.getName(), "/reactController/proximity_events:i")

    while time.time() - start < 10:
        pass

    obs_type = int(sys.argv[2])

    if obs_type == 100:
        gen_static_obstacle(out_port, [-0.349, -0.22, 0.1], 60, use_neo, use_right)
    elif obs_type == 101:
        gen_static_obstacle(out_port, [-0.27, -0.14, 0.06], 60, use_neo, use_right)
    elif obs_type == 102:
        gen_static_obstacle(out_port, [-0.27, -0.11, 0.06], 60, use_neo, use_right)
    # elif obs_type == 13:
    #     gen_dynamic_1way_obstacle(out_port, [-0.28, 0.044, 0.02], 0.05, [0, -1, 0], 15, 8, use_neo, use_right)  # exp 3
    # elif obs_type == 14:
    #     gen_dynamic_1way_obstacle(out_port, [-0.43, -0.164, 0.02], 0.06, [1, 0, 0], 15, 9, use_neo, use_right)  # exp 3
    # elif obs_type == 15:
    #     gen_dynamic_1way_obstacle(out_port, [-0.2, -0.22, 0.3], 0.08, [0, 0, -1], 15, 8, use_neo, use_right)  # exp 3
    elif obs_type == 16:
        gen_dynamic_1way_obstacle(out_port, [-0.27, 0.052, 0.0], 0.06, [0, -1, 0], 6, 8, use_neo, use_right)  # exp 3
    elif obs_type == 17:
        gen_dynamic_1way_obstacle(out_port, [-0.43, -0.164, 0.02], 0.06, [1, 0, 0], 6, 8, use_neo, use_right)  # exp 3
    elif obs_type == 18:
        gen_dynamic_1way_obstacle(out_port, [-0.2, -0.2, 0.28], 0.06, [0, 0, -1], 6, 8, use_neo, use_right)  # exp 3
    # elif obs_type == 0:
    #     gen_static_obstacle(out_port, [-0.099, 0.0, 0.2], 30, use_neo, use_right)
    yarp.Network.fini()


if __name__ == "__main__":
    main()
