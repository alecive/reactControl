#!/usr/bin/python3
import copy
import time
import yarp
import numpy as np
import re
import sys
import json


def both_arms_circular(rpc_client, client, inp, pars):
    center = pars["center"]
    center[1] *= -1  # prvni je prava
    offset = -0.15
    p1 = [center[0], center[1] + pars["radius"], center[2]]
    p2 = [center[0], center[1] + pars["radius"] + offset, center[2]]
    result = set_both_xd(p1, p2, False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass

    start = time.time()
    while time.time() - start < pars["time"]:
        x = center[0]
        y = center[1] + pars["radius"] * np.cos(2.0 * np.pi * pars["freq"] * (time.time() - start))
        z = center[2] + pars["radius"] * np.sin(2.0 * np.pi * pars["freq"] * (time.time() - start))
        print(x, y, z)
        result = set_both_xd([x, y, z], [x, y + offset, z], True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass

def bimanual_task(rpc_client, client, inp, pars):
    center = pars["center"]
    center[1] *= -1  # prvni je prava
    pars["radius"] *= 0.5
    center[1] -= 0.04
    offset = -0.10 # -0.15
    p1 = [center[0], center[1] + pars["radius"], center[2]]
    p2 = [center[0], center[1] + pars["radius"] + offset, center[2]]
    result = set_both_xd(p1, p2, False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass

    start = time.time()
    while time.time() - start < pars["time"]:
        x = center[0]
        y = center[1] + pars["radius"] * np.cos(2.0 * np.pi * pars["freq"] * (time.time() - start))
        z = center[2] + pars["radius"] * np.sin(2.0 * np.pi * pars["freq"] * (time.time() - start))
        print(x, y, z)
        result = set_both_xd([x, y, z], [x, y + offset, z], True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def both_arms_lemniscate(rpc_client, client, inp, pars):
    center = pars["lemni_center"]
    center[1] *= -1
    offset = -0.15
    p1 = [center[0], center[1], center[2]]
    p2 = [center[0], center[1] + offset, center[2]]
    result = set_both_xd(p1, p2, False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass
    gain = 0.13
    start = time.time()
    i = 0
    for rep in range(pars["lemni_reps"]):
        for point in pars["points"]:
            while (time.time() - start) < i * pars["lemni_step"]:
                pass
            i += 1
            x = center[0] + gain * point[0]
            y = center[1] + gain * point[1]
            z = center[2] + gain * point[2]
            print(x, y, z)
            result = set_both_xd([x, y, z], [x, y + offset, z], True)
            client.write(result)



def both_arms_circular_colls(rpc_client, client, inp, pars):
    center = pars["center"]
    center2 = copy.deepcopy(pars["center"])
    pars["radius"] *= 1.25
    center[1] *= -1  # prvni je prava
    result = set_both_xd([center[0], center[1] + pars["radius"], center[2]],
                         [center2[0], center2[1] - pars["radius"], center2[2]], False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass

    start = time.time()
    while time.time() - start < pars["time"]:
        x = center[0]
        y = center[1] + pars["radius"] * np.cos(2.0 * np.pi * pars["freq"] * (time.time() - start))
        z = center[2] + pars["radius"] * np.sin(2.0 * np.pi * pars["freq"] * (time.time() - start))
        x2 = center2[0]
        y2 = center2[1] - pars["radius"] * np.cos(2.0 * np.pi * pars["freq"] * (time.time() - start))
        z2 = center2[2] + pars["radius"] * np.sin(2.0 * np.pi * pars["freq"] * (time.time() - start))
        print(x, y, z)
        result = set_both_xd([x, y, z], [x2, y2, z2], True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def both_arms_holdpos(rpc_client, inp, timeout=30):
    center = [-0.299, -0.174, 0.1]
    center[1] *= -1
    result = set_both_xd([center[0], center[1], center[2]], [center[0], -center[1], center[2]])
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("hold_position")
    send_command(rpc_client, result)
    start = time.time()
    while time.time() - start < timeout:
        pass


def set_both_xd(p1, p2, streaming=False):
    bot = yarp.Bottle()
    bot.clear()
    if streaming:
        for p in p1:
            bot.addFloat64(p)

        for p in p2:
            bot.addFloat64(p)
    else:
        bot.addString("set_both_xd")
        m = bot.addList()
        for p in p1:
            m.addFloat64(p)
        k = bot.addList()
        for p in p2:
            k.addFloat64(p)

    return bot


def streamed_exp(rpc_client, pars, use_right=False):
    np.random.seed(pars["seed"])
    x = np.round(np.arange(pars["xmin"], pars["xmax"], pars["xstep"]), 2)  # np.round(np.arange(-0.3, -0.05, 0.05),2)
    y = np.round(np.arange(pars["ymin"], pars["ymax"], pars["ystep"]), 2)  # np.round(np.arange(-0.2, 0.05, 0.05),2)
    y = -y if use_right else y
    z = np.round(np.arange(pars["zmin"], pars["zmax"], pars["zstep"]), 2)  # np.round(np.arange(-0.05, 0.35, 0.05),2)
    poses = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    rot = [[-0.1477, -0.7912, 0.5933, 3.0637], [-0.112, 0.9935, 0.01514, 3.1355]] if use_right \
        else [[-0.1477, 0.5933, -0.7912, 3.0637], -0.112, 0.01514, 0.9935, 3.1355]
    print("Size is ", poses.shape)
    indexes = np.random.permutation(poses.shape[0])
    for idx in indexes:
        pos = poses[idx]
        rot_idx = np.random.randint(0, 1)
        # result = set_xd(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
        result = yarp.Bottle()
        result.clear()
        result.addString("set_6d")
        m = result.addList()
        m.addFloat64(pos[0])
        m.addFloat64(pos[1])
        m.addFloat64(pos[2])
        k = result.addList()
        k.addFloat64(rot[rot_idx][0])
        k.addFloat64(rot[rot_idx][1])
        k.addFloat64(rot[rot_idx][2])
        k.addFloat64(rot[rot_idx][3])
        send_command(rpc_client, result)

        print(pos)
        start = time.time()
        while time.time() - start < pars["timeout"]:
            pass


def p2p_exp(client, pars, use_right=False):
    points = pars["points_right"] if use_right else pars["points_left"]
    ite = 0
    start = time.time()
    for i in range(pars["p2p_reps"] + 1):
        for point in points:
            while (time.time() - start) < pars["p2p_period"] * ite:
                pass
            ite += 1
            result = yarp.Bottle()
            result.clear()
            result.addString("set_6d")
            m = result.addList()
            m.addFloat64(point[0])
            m.addFloat64(point[1])
            m.addFloat64(point[2])
            k = result.addList()
            k.addFloat64(point[3])
            k.addFloat64(point[4])
            k.addFloat64(point[5])
            k.addFloat64(point[6])
            send_command(client, result)
            if i == pars["p2p_reps"]:
                while (time.time() - start) < pars["p2p_period"] * ite:
                    pass
                break


def lemniscate_exp(rpc_client, client, inp, pars, use_right=False):
    center = pars["lemni_center"]
    if use_right:
        center[1] *= -1
    result = set_xd(center[0], center[1], center[2], False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass
    gain = 0.13
    start = time.time()
    i = 0
    for rep in range(pars["lemni_reps"]):
        for point in pars["points"]:
            while (time.time() - start) < i * pars["lemni_step"]:
                pass
            i += 1
            x = center[0] + gain * point[0]
            y = center[1] + gain * point[1]
            z = center[2] + gain * point[2]
            print(x, y, z)
            result = set_xd(x, y, z, True)
            client.write(result)


def set_xd(x, y, z, streaming=False):
    bot = yarp.Bottle()
    bot.clear()
    if streaming:
        bot.addFloat64(x)
        bot.addFloat64(y)
        bot.addFloat64(z)
    else:
        bot.addString("set_xd")
        m = bot.addList()
        m.addFloat64(x)
        m.addFloat64(y)
        m.addFloat64(z)
    return bot


def holdpos(rpc_client):
    result = yarp.Bottle()
    result.clear()
    result.addString("hold_position")
    send_command(rpc_client, result)


def stop_move(rpc_client):
    result = yarp.Bottle()
    result.clear()
    result.addString("stop")
    send_command(rpc_client, result)
    print("Stop sent")


def circular_exp(rpc_client, client, inp, pars, use_right=False):
    center = pars["center"]
    if use_right:
        center[1] *= -1
    result = set_xd(center[0], center[1] + pars["radius"], center[2], False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass

    start = time.time()
    while time.time() - start < pars["time"]:
        x = center[0]
        y = center[1] + pars["radius"] * np.cos(2.0 * np.pi * pars["freq"] * (time.time() - start))
        z = center[2] + pars["radius"] * np.sin(2.0 * np.pi * pars["freq"] * (time.time() - start))
        print(x, y, z)
        result = set_xd(x, y, z, True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def linmov_exp(rpc_client, client, inp, pars, use_right=False):
    stop = pars["stop"]
    step = pars["step"]
    start_pos = pars["start"]
    if use_right:
        start_pos[1] *= -1
        stop *= -1
        step *= -1
    result = set_xd(start_pos[0], start_pos[1], start_pos[2], False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("set_streaming_xd")
    send_command(rpc_client, result)
    small_start = time.time()
    while time.time() - small_start < 0.1:
        pass

    y = start_pos[1]
    while (y < stop and not use_right) or (y > stop and use_right):
        y += step
        result = set_xd(start_pos[0], y, start_pos[2], True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass

    small_start = time.time()
    while time.time() - small_start < 2:
        pass
    result = set_xd(start_pos[0], start_pos[1], start_pos[2], False)
    send_command(client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)


def classic_exp(rpc_client, inp, timeout=30, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.12]]
    for pos in poses:
        result = set_xd(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
        send_command(rpc_client, result)
        pose_str = None

        while not pose_str:
            pose_str = read_once(inp)

    result = yarp.Bottle()
    result.clear()
    result.addString("set_relative_circular_xd")
    result.addFloat64(0.08)
    result.addFloat64(0.2)

    send_command(rpc_client, result)
    start = time.time()

    while time.time() - start < timeout:
        pass


def holdpos_exp(rpc_client, inp, timeout=30, use_right=False):
    center = [-0.299, -0.174, 0.1]
    if use_right:
        center[1] *= -1
    result = set_xd(center[0], center[1], center[2], False)
    send_command(rpc_client, result)
    pose_str = None
    while not pose_str:
        pose_str = read_once(inp)
    result.clear()
    result.addString("hold_position")
    send_command(rpc_client, result)
    start = time.time()
    while time.time() - start < timeout:
        pass


def visual_exp(rpc_client, inp, use_right=False):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.1],
             [-0.299, -0.074, 0.1]]
    for pos in poses:
        result = set_xd(pos[0], -pos[1] if use_right else pos[1], pos[2], False)
        send_command(rpc_client, result)
        pose_str = None

        while not pose_str:
            pose_str = read_once(inp)
        print(pose_str)
        print(pos)


def send_command(rpc_client, command):
    ans = yarp.Bottle()
    # print("Command sent: ", command.toString())
    rpc_client.write(command, ans)
    # print ("Reply received: ", ans.toString())
    return ans.toString()


def read_once(inp):
    bottle = yarp.Bottle()
    bottle.clear()
    # read the messge
    inp.read(bottle)
    # print("Received ", bottle.toString())
    return bottle.toString().split(' ')


def main(move_type):
    with open("scripts/sample.json") as fp:
        params = json.load(fp)
    yarp.Network.init()  # Initialise YARP
    conf = open('app/conf/reactController.ini')
    part = "left"
    for line in conf.readlines():
        if line.startswith('part'):
            part = re.findall(r"(?<=\()\w+", line)[0]
            break

    conf.close()
    use_right_ = (part == "right")
    rpcport_name = "/reactController/rpc:i"
    inport_name = "/reactController/finished:o"

    streamedport = "/reactController/streamedTargets:i"
    inport = yarp.Port()
    inport.open("/reader")
    yarp.Network.connect(inport_name, inport.getName())

    outport = yarp.Port()
    outport.open('/pokus:o')
    yarp.Network.connect(outport.getName(), streamedport)

    outport_rpc = yarp.RpcClient()
    outport_rpc.open('/pokusRpc:o')
    yarp.Network.connect(outport_rpc.getName(), rpcport_name)

    stop_move(outport_rpc)

    start = time.time()
    while time.time() - start < 1:
        pass
    if move_type == 0:
        visual_exp(outport_rpc, inport, use_right_)
        holdpos(outport_rpc)
    elif move_type == 1:
        holdpos_exp(outport_rpc, inport, 18, use_right_)
    elif move_type == 2:
        streamed_exp(outport_rpc, params["randomTargets"], use_right_)
    elif move_type == 3:
        p2p_exp(outport_rpc, params["p2p"], use_right=use_right_)
    elif move_type == 4:
        circular_exp(outport_rpc, outport, inport, params["circle"], use_right=use_right_)
    elif move_type == 5:
        lemniscate_exp(outport_rpc, outport, inport, params["lemniscate"], use_right=use_right_)
    elif move_type == 6:
        linmov_exp(outport_rpc, outport, params["linMov"], use_right_)
    elif move_type == 7:
        both_arms_circular(outport_rpc, outport, inport, params["circle"])
    elif move_type == 8:
        both_arms_circular_colls(outport_rpc, outport, inport, params["circle"])
    elif move_type == 9:
        both_arms_lemniscate(outport_rpc, outport, inport, params["lemniscate"])
    elif move_type == 10:
        both_arms_holdpos(outport_rpc, inport, 30)
    elif move_type == 11:
        bimanual_task(outport_rpc, outport, inport, params["circle"])
    
    start = time.time()
    while time.time() - start < 2:
        pass
    if move_type > 0:
        stop_move(outport_rpc)
        # close the network
    yarp.Network.fini()


if __name__ == "__main__":
    main(int(sys.argv[1]))
