#!/usr/bin/python3
import copy
import time
import yarp
import numpy as np
import re
import sys
import json


#reactController --robot icub --part "(right left)" --trajSpeed 0.02 --vMax 10
def bimanual_task_p2p(rpc_client, client, inp, pars, use_right=False):
    points = [[-0.299, 0.1,0.05], [-0.289, 0.05,0.15], [-0.299, 0.0,0.05]]
    idxs = 4* [1,2,1,0]
    offset = -0.10
    p1 = [points[0][0], points[0][1], points[0][2]]
    p2 = [points[0][0], points[0][1] + offset, points[0][2]]
    result = set_both_xd(p1, p2, m_arm_constr=True, streaming=False)
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
    
    for idx in idxs:
        x = points[idx][0]
        y = points[idx][1]
        z = points[idx][2]
        result = set_both_xd([x, y, z], [x, y + offset, z], m_arm_constr=True, streaming=True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 3:
            pass

def bimanual_task(rpc_client, client, inp, pars, use_right=False):
    center = pars["center"]
    center[1] = -0.05
    offset = 0.10  # 0.15
    if use_right:
        center[1] *= -1
        offset *= -1
    pars["radius"] *= 0.7
    pars["freq"] *= 2
    p1 = [center[0], center[1] + pars["radius"], center[2]]
    p2 = [center[0], center[1] + pars["radius"] + offset, center[2]]
    result = set_both_xd(p1, p2, m_arm_constr=True, streaming=False)
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
        # print(x, y, z)
        result = set_both_xd([x, y, z], [x, y + offset, z], m_arm_constr=True, streaming=True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def both_arms_circular_colls_avoid(rpc_client, client, inp, pars, use_right=False):
    center = pars["center"]
    center2 = copy.deepcopy(pars["center"])
    if use_right:
        center[1] *= -1
    result = set_both_xd([center[0], center[1] + pars["radius"], center[2]],
                         [center2[0], center2[1] - pars["radius"], center2[2]], m_arm_constr=True, streaming=False)
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
        # print(x, y, z)
        result = set_both_xd([x, y, z], [x2, y2, z2], m_arm_constr=True, streaming=True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def both_arms_circular_colls_small(rpc_client, client, inp, pars, use_right=False):
    center = pars["center"]
    center2 = copy.deepcopy(pars["center"])
    if use_right:
        center[1] *= -1
    result = set_both_xd([center[0], center[1] + pars["radius"], center[2]],
                         [center2[0], center2[1] - pars["radius"], center2[2]], m_arm_constr=True, streaming=False)
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
        # print(x, y, z)
        result = set_both_xd([x, y, z], [x2, y2, z2], m_arm_constr=True, streaming=True)
        client.write(result)
        small_start = time.time()
        while time.time() - small_start < 0.02:
            pass


def both_arms_holdpos(rpc_client, inp, timeout=30, use_right=False):
    center = [-0.299, -0.174, 0.1]
    if use_right:
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


def set_both_xd(p1, p2, m_arm_constr=True, streaming=False):
    bot = yarp.Bottle()
    bot.clear()
    if streaming:
        for p in p1:
            bot.addFloat64(p)

        for p in p2:
            bot.addFloat64(p)
        bot.addInt32(m_arm_constr)
    else:
        bot.addString("set_p_both_xd")
        m = bot.addList()
        for p in p1:
            m.addFloat64(p)
        k = bot.addList()
        for p in p2:
            k.addFloat64(p)
        bot.addInt32(m_arm_constr)
    # print(bot.toString())
    return bot


def streamed_exp(rpc_client, pars, seed, use_right=False):
    np.random.seed(pars["seed"] if seed == 0 else seed)
    x = np.round(np.arange(pars["xmin"], pars["xmax"], pars["xstep"]), 2)  # np.round(np.arange(-0.3, -0.05, 0.05),2)
    y = np.round(np.arange(pars["ymin"], pars["ymax"], pars["ystep"]), 2)  # np.round(np.arange(-0.2, 0.05, 0.05),2)
    y = -y if use_right else y
    z = np.round(np.arange(pars["zmin"], pars["zmax"], pars["zstep"]), 2)  # np.round(np.arange(-0.05, 0.35, 0.05),2)
    poses = np.array(np.meshgrid(x, y, z)).T.reshape(-1, 3)
    rot = [[-0.1477, -0.7912, 0.5933, 3.0637], [-0.112, 0.9935, 0.01514, 3.1355]] if use_right \
        else [[-0.1477, 0.5933, -0.7912, 3.0637], -0.112, 0.01514, 0.9935, 3.1355]
    print("Size is ", poses.shape)
    print(poses)
    indexes = np.random.permutation(poses.shape[0])
    for idx in indexes:
        pos = poses[idx]
        rot_idx = np.random.randint(0, 2)
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


def main(move_type, config_file, seed):
    with open(config_file) as fp:
        params = json.load(fp)
    yarp.Network.init()  # Initialise YARP
    conf = open('app/conf/reactController.ini')
    part = "left"
    for line in conf.readlines():
        if line.startswith('part'):
            part = re.findall(r"(?<=\()\w+", line)[0]
            break
    print(part, part == "right")
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
        streamed_exp(outport_rpc, params["randomTargets"], seed, use_right_)
    elif move_type == 3:
        p2p_exp(outport_rpc, params["p2p"], use_right=use_right_)
    elif move_type == 4:
        circular_exp(outport_rpc, outport, inport, params["circle"], use_right=use_right_)
    elif move_type == 5:
        both_arms_circular_colls_small(outport_rpc, outport, inport, params["circle_small"], use_right_)
    elif move_type == 6:
        both_arms_circular_colls_avoid(outport_rpc, outport, inport, params["circle"], use_right_)
    elif move_type == 12:
        both_arms_holdpos(outport_rpc, inport, 30, use_right_)
    elif move_type == 13:
        bimanual_task(outport_rpc, outport, inport, params["circle"], use_right_)
    elif move_type == 14:
        bimanual_task_p2p(outport_rpc, outport, inport, params["circle"], use_right_)

    start = time.time()
    while time.time() - start < 2:
        pass
    if move_type > 0:
        stop_move(outport_rpc)
        # close the network
    yarp.Network.fini()


if __name__ == "__main__":
    filename = "scripts/params-noobs.json"
    seed = 0
    if (len(sys.argv) > 2):
        filename = sys.argv[2]
        if (len(sys.argv) > 3):
            seed = int(sys.argv[3])
    
    main(int(sys.argv[1]), filename, seed)
