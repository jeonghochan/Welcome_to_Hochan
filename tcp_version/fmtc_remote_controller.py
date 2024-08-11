# Simple TCP Test
# Mobile (5G) <-> Public IP 147.47.239.109:5778

from http import server
import socket

import pickle
import time 
import sys
print ("TCPServer Waiting for client on port 5000")

PAIR_KEY = "vdcl1234"


def is_alive(msg):
    # msg pattern is "alive:uuid"
    # send back
    if msg.startswith("alive:"):
        return msg
        

while 1:
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(("", 5778))

        server_socket.listen(5)
        client_socket, address = server_socket.accept()
        print ("Incoming Connection Request", address)
        
        # Auth
        data = client_socket.recv(1024)
        data = data.decode()
        if data == PAIR_KEY:
            print("Auth Success")
            client_socket.send("OK".encode()) 
            time.sleep(1) 
        else:
            print("Auth Failed, reset connection")
            client_socket.send("FAIL".encode())  
            client_socket.close()
            server_socket.close()
            continue
        
        
        while 1:
            data_pack = dict({"ts" : time.time(), "data" : "Hello World", "control_request" : False})

            data = pickle.dumps(data_pack)

            client_socket.send(data)
            client_socket.recv(1024)
            
            time.sleep( 1.0 / 60.0)
            
            #data = client_socket.recv(512).decode()
    except KeyboardInterrupt:
        server_socket.close()
        print("Bye")
        sys.exit(0)

    except Exception as ex:
        print("Connection Closed... Standby")
        #server_socket.bind(("", 5778))
        time.sleep(2)
        client_socket.close()
        server_socket.close()


print("SOCKET closed... END")