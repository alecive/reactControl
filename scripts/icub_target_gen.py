#!/usr/bin/python3
import socket, re, sys
import time

def get_addr(s):
    m = re.match("registration name [^ ]+ ip ([^ ]+) port ([0-9]+) type tcp",s)
    return (m.group(1),int(m.group(2))) if m else None
 
# get a single line of text from a socket
def getline(sock):
    result = ""
    while result.find('\n')==-1:
        result = result + sock.recv(1024).decode()
    result = re.sub('[\r\n].*','',result)
    return result
 
# send a message and expect a reply
def comm(addr,message):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(addr)
    sock.send(b'CONNACK extern\n')
    getline(sock)
    sock.send(b'd\n%s\n' % message.encode())
    result = getline(sock)
    sock.close()
    return result

def read_once(addr):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(addr)
    sock.send(b'CONNECT extern\nr\n')
    data = sock.recv(1024)
    data = sock.recv(1024).decode().split('\r\n')
    data = data[1].split()
    return data


def randomMovements(query, query_read, use_cart=False):
    import numpy as np
    return_to_home = True
    x, y, z = np.meshgrid(np.round(np.arange(-0.4, 0.0, 0.05),2), np.round(np.arange(-0.2, 0.1, 0.05),2), np.round(np.arange(-0.2, 0.4, 0.05),2), indexing='ij')
    x, y, z = np.meshgrid(np.round(np.arange(-0.3, 0.0, 0.05),2), np.round(np.arange(-0.2, 0.1, 0.05),2), np.round(np.arange(-0.1, 0.4, 0.05),2), indexing='ij')

    x = x.flatten()
    y = y.flatten()
    z = z.flatten()
    print("Size is ", len(x))
    feasibility = [False] * len(x)
    indexes = np.random.randint(0, len(x), size=len(x))
    feasible_poses = []
    count = 0
    for idx in indexes:
        pos = [x[idx], y[idx], z[idx]]
        message = f"{pos[0]} {pos[1]} {pos[2]}" if use_cart else f"set_xd ({pos[0]} {pos[1]} {pos[2]})"
        comm(query,message)
        poseStr = None
        print(message)
        while not poseStr:
            poseStr = read_once(query_read)
        #print(' '.join(poseStr[:3]))
        posErrSq = (pos[0]*1000-int(poseStr[0]))**2 + (pos[1]*1000-int(poseStr[1]))**2 + (pos[2]*1000-int(poseStr[2]))**2
        print(count, posErrSq)
        count += 1
        if (posErrSq < 400): # 2 cm
            feasibility[idx] = True
            feasible_poses.append(pos)
        if return_to_home:
            message = "go_to_home"
            comm(query,message)
            poseStr = None
            print(message)
            while not poseStr:
                poseStr = read_once(query_read)
            print(' '.join(poseStr[:3]))
        


    print(count, len(feasible_poses))
    print(feasible_poses)


def classicScenario(query, query_read):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.1]]
    for pos in poses:
        message = f"set_xd ({pos[0]} {pos[1]} {pos[2]})"
        comm(query,message)
        poseStr = None
        print(message)
        while not poseStr:
            poseStr = read_once(query_read)

    message = f"set_relative_circular_xd 0.08 0.2"
    comm(query,message)
    start = time.time()
    while time.time()-start < 60:
        pass
    message = f"stop"
    comm(query,message)
    
    message = f"hold_position"
    comm(query,message)


def visualScenario(query, query_read):
    poses = [[-0.299, -0.174, 0.05], [-0.299, -0.174, 0.15], [-0.299, -0.174, 0.00], [-0.299, -0.174, 0.1], [-0.299, -0.074, 0.1]]
    for pos in poses:
        message = f"set_xd ({pos[0]} {pos[1]} {pos[2]})"
        comm(query,message)
        poseStr = None
        print(message)
        while not poseStr:
            poseStr = read_once(query_read)

    message = f"hold_position"
    comm(query,message)



if __name__=="__main__": 
     
    use_cart = False
    name_server = ('localhost', 10000)
     
    port_name = "/testCart/target:i" if use_cart else "/reactController/rpc:i"
    port_name2 = "/testCart/finished:o" if use_cart else "/reactController/finished:o" 
    query = get_addr(comm(name_server,"query %s"%port_name))
    print ("Talking to", port_name, "here:", query)
    query_read = get_addr(comm(name_server,"query %s"%port_name2))
    print ("Listening from", port_name2, "here:", query_read)
    
    # randomMovements(query, query_read, use_cart)
    classicScenario(query, query_read)

        
