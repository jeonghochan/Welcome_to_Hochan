# FMTC Remote Client
# Mobile (5G) <-> Public IP 147.47.239.109:5778

from csv import excel
from datetime import datetime
import socket
import configparser
import os, sys, time
import threading
import pickle
import uuid
import datetime
# read cfg file
config = configparser.ConfigParser()
config.read('network.cfg')

SERVER_IP = config.get('network', 'SERVER_IP')
PORT = config.get('network', 'PORT')
LIFECYCLE = float(config.get('network', 'LIFECYCLE'))
PAIR_KEY = "vdcl1234"


# non-blocking socket
class network_client:
    connection_status = False
    current_job = ""
    last_recv_message = ""

    
    def __init__(self):
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
    def auth(self):
        # send pairkey to server
        self.current_job = "On Authentication Process"
        self.send(PAIR_KEY)
        s, data = self.receive()
        # self.current_job = f"Authentication Result: {data}"
        time.sleep(1)
        if data == b"OK":
            time.sleep(1)
            return True
        elif data == b"FAIL":
            time.sleep(1)
            return False
    
    ## alive health check by echoing back the received message
    def is_alive(self):
        if self.connection_status:
            _uuid = str(uuid.uuid4())
            self.send(f'alive:{_uuid}')
        
    def loop(self):
        while True:
            
            if not self.connection_status:
                self.get_connection()

                
            try:
                t1 = time.time()
                s_code, data = self.receive()
                if s_code == 0:
                    
                    if data:
                        data = pickle.loads(data)
                
                    
                        
                        
                        
                        self.safety_check(t1, data)
                    
                elif s_code == 1:
                    data = data
                    
                self.is_alive()
                    
                self.last_recv_message = data
                    
                # else:
                #     self.current_job = print("\033[91mNo Data!, Not expected\033[0m")
                #     self.close()
                #     break
            except Exception as ex:
                print(ex)
                self.connection_status = False
                continue # For reconnect

    def safety_check(self, t1, data):
        # check if the received message is too old
        t2 = time.time()
        if t2 - t1 > 0.02:
            self.current_job = f": {t2-t1} sec"




    def get_connection(self):
        # try to connect server, if port is used, wait 1 sec and try again, until connection is established
        while not self.connection_status:
            try:
                self.current_job = "connecting to FMTC Server"
                self.client_socket.connect((SERVER_IP, int(PORT)))
                
                if not (self.auth()):
                    self.current_job = "Authentication Failed"
                    self.client_socket.shutdown(socket.SHUT_RDWR)

                    raise Exception("Authentication Failed")
                
                self.connection_status = True
                self.connection_start_time = time.time()
            # Socket is used, kill the occupied process by lsof 
            except Exception as ex: 
                #print("Socket is used, kill the occupied process by lsof")
                #os.system("lsof -i :{}".format(PORT))
                #os.system("kill -9 $(lsof -t -i:{})".format(PORT))
                print(ex)

                time.sleep(3)
                self.__init__()
                
        self.current_job = "connection established"
        return self.connection_status
    
    def get_connection_uptime(self):
        if self.connection_status:
            return round(time.time() - self.connection_start_time, 0)
        else:   
            return 0

    def send(self, msg):
        self.client_socket.send(msg.encode())

    def receive(self):
        try:
            data = self.client_socket.recv(1024)
            return 0, data # Pickle
        except:
            data = self.client_socket.recv(1024).decode()
            return 1, data # Normal message
    def close(self):
        self.client_socket.close()

class car_handler:
    # Print control status and connection status

    def __init__(self):
        self.control_status = False # Default is On-Device
        self.car_fail_signal = False # From ROS-VDCL Logic
        self.connection_status = False # Default is not connected
        
        # Make network thread
        self.network_inst = network_client()
        self.network_thread = threading.Thread(target=self.network_inst.loop)
        self.network_thread.daemon = True
        self.network_thread.start()
        

        # Make ROS thread
        self.ros_comm = ros_comm()
        
        
        
    
        # Some ROS Init
        
        
    def _print(self):
        # Print colored control status and connection status, red is false, green is true
        # and flush the print buffer
        datatime = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"[FMTC Remote Client]        [{datatime}] ")
        print("")
        print("[CAR Side INFO]")
        if self.control_status:
            print("     \033[92m[Control Status] Remote\033[0m")
        else:
            # Orange color
            print("     \033[93m[Control Status] On-Device\033[0m")
        
        if self.car_fail_signal:
            print("     \033[91m[Car State] Fault\033[0m")
        else:
            print("     \033[92m[Car Fail Signal] Normal\033[0m")
            
        print("[Network INFO]")
        if self.network_inst.connection_status:
            print("     \033[92m[Connection Status] Connected\033[0m")
            print("     \033[92m[Connection Uptime] {}s\033[0m".format(self.network_inst.get_connection_uptime()))
            
        else:
            print("     \033[91m[Connection Status] Disconnected\033[0m")
            print("     \033[91m[Connection Uptime] 0s\033[0m")
            
        print("[Job]")
        print("     {}".format(self.network_inst.current_job))
        
        print("[Last Message]")
        print("     {}".format(self.network_inst.last_recv_message))
            
        
    def _flush(self):
        # Flush Terminal
        os.system('cls' if os.name == 'nt' else 'clear')
        
        
    def _control(self):
        pass

    def loop(self):
        # while loop, if get ctrl+c, close the socket and give ros topic a last message
        # and exit the program
        try:
            while True:
                
                self._print()
                self._control()
                time.sleep( LIFECYCLE )
                self._flush()
                
        except KeyboardInterrupt:
            self.network_inst.close()
            self.ros_comm.close()
            print("Bye")
            sys.exit()
            
        
        
        
class ros_comm:
    
    car_data = dict({
        "Vx" : 0,
        "SAS_angle" : 0,
        "MDPS_Module_Status" : 0,
        "ACC_Module_Status" : 0,
        "RTK" : [0,0],
        "FAIL_SIGNAL" : 0,
    })
   
    
    def __init__(self):
        # ROS Init
        self.control_status = False
        pass
        
    def sNode(self):
        # ROS Subscriber Node
        pass
    
    def close(self):
        # TODO : Give /remote_cotrol/availabilty : False
        pass
    
    
if __name__ == "__main__":
    ch = car_handler()
    ch.loop()