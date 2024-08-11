import requests
SERVER_IP = "http://1.249.212.211"
s_data = dict({"uid": "cc7a91473bf04b59afb9bd91123665f8", "msg": {"ctrl_fault" : 0, "ctrl_request" : 0, "ctrl_remote" : 0, "ctrl_mode" : 1} })
resp = requests.post(url=SERVER_IP + str("/ctrl_status"), json=s_data).json()
print(resp)
import time
while 1: 
    time.sleep(1)
    resp = requests.post(url=SERVER_IP + str("/forVCS")).json()
    print(resp)
