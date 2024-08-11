'''
FMTC Remote v2 TCP
Shinkansan
'''

from collections import deque
from queue import Queue
from doctest import UnexpectedException
from unicodedata import name
import pygame
import cv2
import numpy as np
import configparser
import matplotlib.pyplot as plt
from matplotlib.backends.backend_agg import FigureCanvasAgg
import time
import threading
import sys, os
import socket
import csv
import copy
from datetime import datetime
import pickle

# Haptic

import platform
import requests

import util.camera_viz as hud

pygame.init() 

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

CAMERA_MODE = 'gst'
VIDEO_COMM_MODE = 'udp' # udp or tcp
IS_DARWIN = True if platform.system() == 'Darwin' else False

SERVER_IP = "http://1.249.212.211"
SERVER_CTRL_IP = "http://1.249.212.211:9912"

CAR_MODE = {0: "수동주행", 1: "자율주행", 2: "Remote"}

height = 1080
width = 1920
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
pygame.display.set_caption("FMTC Remote HUB v2")
done= False
clock= pygame.time.Clock()

config = configparser.ConfigParser()    
config.read('network.cfg', encoding='utf-8') 



ANGLE_SCALE = 300
AUTO_CENTER = True # g29 
if AUTO_CENTER:
    import g29_adapter
PAIR_KEY = "vdcl1234"

### Graph Plot에 사용되는 변수들
name_to_arr_idx = { "user_gas": 0,
                    "user_brake": 1,
                    "computer_gas": 2,
                    "computer_brake": 3,
                    "v_ego": 4,
                    "v_pid": 5,
                    "angle_steers_des": 6,
                    "angle_steers": 7,
                    "network_ping_delay": 8,
                    "network_bandwidth": 9}

plot_arr = np.zeros((100, len(name_to_arr_idx.values())))

plot_xlims = [(0, plot_arr.shape[0]), (0, plot_arr.shape[0]), (0, plot_arr.shape[0])]
plot_ylims = [(0, 110), (-ANGLE_SCALE, ANGLE_SCALE), (0., 500.)]
plot_names = [["user_gas", "user_brake", "computer_gas", "computer_brake"],
            ["v_ego", "v_pid", "angle_steers_des", "angle_steers"],
                ["network_ping_delay", "network_bandwidth"]]
plot_colors = [["b", "g", "r", "y"],
                ["b", "g", "y", "r"],
                ["b", "g", "r", "y"]]
plot_styles = [["-", "-", "-", "-", "-"],
                ["-", "-", "-", "-"],
                ["-", "-", "-", "-"]]

###


objects = []
class Button(): # UI에 버튼 추가해주는거
    font = pygame.font.Font('./assets/hhm.ttf', 20)
    def __init__(self, x, y, width, height, buttonText='Button', onclickFunction=None, onePress=False):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.onclickFunction = onclickFunction
        self.onePress = onePress

        self.fillColors = {
            'normal': '#efefef',
            'hover': '#666666',
            'pressed': '#333333',
        }

        self.buttonSurface = pygame.Surface((self.width, self.height))
        self.buttonRect = pygame.Rect(self.x, self.y, self.width, self.height)

        self.buttonSurf = self.font.render(buttonText, True, (20, 20, 20))

        self.alreadyPressed = False

        objects.append(self)

    def process(self):

        mousePos = pygame.mouse.get_pos()
        
        self.buttonSurface.fill(self.fillColors['normal'])
        if self.buttonRect.collidepoint(mousePos):
            self.buttonSurface.fill(self.fillColors['hover'])

            if pygame.mouse.get_pressed(num_buttons=3)[0]:
                self.buttonSurface.fill(self.fillColors['pressed'])

                if self.onePress:
                    self.onclickFunction()

                elif not self.alreadyPressed:
                    self.onclickFunction()
                    self.alreadyPressed = True

            else:
                self.alreadyPressed = False

        self.buttonSurface.blit(self.buttonSurf, [
            self.buttonRect.width/2 - self.buttonSurf.get_rect().width/2,
            self.buttonRect.height/2 - self.buttonSurf.get_rect().height/2
        ])
        screen.blit(self.buttonSurface, self.buttonRect)

class main:
    alert1_font = pygame.font.Font("./assets/hhm.ttf", 30)
    alert2_font = pygame.font.Font("./assets/hhm.ttf", 15)
    cur_time_font = pygame.font.Font("./assets/hhm.ttf", 20)

    # image_recv_size = (1920, 360)



    img = np.zeros((480, 640, 3), dtype='uint8')
    loop_enable = True
    joy_connected = False
    first_setup = False
    on_logging = False
    camera_feed_online = False
    
    def __init__(self):
        self.control_request_accept = False
        self.brake_pressed_time = 0
        self.widthOfScreen, self.heightOfScreen = width, height
        self.fmtc_logo = pygame.image.load("./assets/fmtc_logo.png")
        self.fmtc_logo = pygame.transform.smoothscale(self.fmtc_logo, [self.fmtc_logo.get_width()/2, self.fmtc_logo.get_height()/2])

        # load instruction image 
        self.instruction_paddle_img = pygame.image.load("./assets/press_paddle.png")
        self.instruction_paddle_img = pygame.transform.smoothscale(self.instruction_paddle_img, [self.instruction_paddle_img.get_width()/3.5, self.instruction_paddle_img.get_height()/3.5])
        
        if CAMERA_MODE == 'panorama':
            self.image_recv_size = (1920, 360)
            self.camera_surface = pygame.surface.Surface(self.image_recv_size, 0, 24).convert()
        else:
            self.image_recv_size = (int(self.widthOfScreen * 0.6), int(self.heightOfScreen *0.6))
            self.camera_surface = pygame.surface.Surface(self.image_recv_size, 0, 24).convert()
        
        
        
        self.draw_plots = self.init_plots(plot_arr, name_to_arr_idx, plot_xlims, plot_ylims, plot_names, plot_colors, plot_styles)
        self.error_log = Queue(maxsize = 4)
        self.sN = feed_provider()
        
        self.camera_thread = threading.Thread(target = self.camera_loop, args=())
        self.camera_thread.start()
        
        self.nE = network_eval()
        
        self.start_thread = threading.Thread(target=self.start_setup, args=()) 
        
        self.joyThread = threading.Thread(target=self.get_joy_and_send, args=())

        if AUTO_CENTER : self.g29_auto_center_daemon = g29_adapter.g29_lib()

        self.alarm_sound = pygame.mixer.Sound("./alarm2.mp3")
        self.sad_cnt2 = 0
        
        pygame.joystick.init()
        self.axis_info = {"steer": 0,"cluth": 0,"acc": 0,"brk": 0}
        self.axes = 0
        joystick_count = pygame.joystick.get_count()
        self.joyAvailable = True if joystick_count > 0 else False
        for i in range(joystick_count):
            self.joystick = pygame.joystick.Joystick(i)
            self.joystick.init()
            self.axes = self.joystick.get_numaxes()
        
        pass
    
    def get_joy_and_send(self): # 조이스틱 핸들 페달 값 받아서 차로 전송
        while 1:
            try:
                clock.tick(65) # 30hz
                st = time.time()
                tmp = []
                for i in range(self.axes):
                    axis = self.joystick.get_axis(i)*100
                    a = str(axis)
                    tmp.append(a)
                    # sock.sendto(a.encode(),(host,port))
                    # print(sys.getsizeof(a.encode()))
                    # if i == 0:print("Axis {} value: {:>6.3f}".format(i, axis))

                # get button press
                buttons = self.joystick.get_numbuttons()
                if buttons > 0:
                    self.sN.sub_data["RightPaddle_pressed"] = bool(self.joystick.get_button(5))
                    self.sN.sub_data["LeftPaddle_pressed"] = bool(self.joystick.get_button(4))

                if self.axes != 0: 
                    self.joy_connected = True
                    self.axis_info = {"steer":tmp[0],"cluth":tmp[1],"acc":tmp[2],"brk":tmp[3]}
                    
                    self.sN.sub_data["joy_angle"] = float(tmp[0])
                    self.sN.sub_data["accel_pressed"] = (100 - float(tmp[2])) > 6e-3
                    self.sN.sub_data["brake_pressed"] = (100 - float(tmp[3])) > 6e-3
                    
                    if self.sN.sub_data["brake_pressed"] and self.sN.sub_data["accel_pressed"]:
                        # Brake Override
                        tmp[2] = 100
                    
                    
                    self.send_cmd =  {"axis0":tmp[0],"axis1":tmp[1],"axis2":tmp[2],"axis3":tmp[3]} # steer, cluth, acc, brk
                    
                    self.sN.cmd_deque.append(self.send_cmd)
                    self.sN.cmd_live = self.send_cmd
                    
                    #self.cmd_socket.sendto(pickle.dumps(self.send_cmd),(config["network"]["car_my_ip"], int(config["network"]["fmtc_to_car_cmd"])))
                else: 
                    self.error_log.put("No Joystick device", "Control Unavailable")
                    self.send_cmd =  {"axis0":tmp[0],"axis1":tmp[1],"axis2":100,"axis3":100}

                et = time.time() - st
                self.sN.sub_data["CMD_tick"] = et

                if AUTO_CENTER: self.g29_auto_center_daemon.loop()
            except Exception as ex:
                self.error_log.put(["Joystick Fault", str(ex)])

            
            
            
    
    def bg_init(self): # 메인 배경 초기화
        black_margin = self.heightOfScreen - 100
        screen.fill(pygame.Color("black"), rect=(0, 0, self.widthOfScreen, black_margin))
        screen.fill(pygame.Color("white"), rect=(0, black_margin, self.widthOfScreen, self.heightOfScreen))
        self.text_on_screen("FMTC Remote HUB", self.cur_time_font, 5, 5)
        self.text_on_screen(datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f'), self.cur_time_font, self.widthOfScreen - 280, 5)
        screen.blit(self.fmtc_logo, (0, black_margin+10))
        #pygame.draw.rect(screen, GREEN, (self.widthOfScreen - 500, black_margin + 5, self.widthOfScreen - 100 , black_margin + 10))
        info_font = pygame.font.Font("./assets/hhm.ttf", 15)
        
        self.camButton = Button(300, self.heightOfScreen - 70,130, 40, 'Setup', self.start_check, False)
        self.logButton = Button(500, self.heightOfScreen - 70,130, 40, 'Logging', self.start_log_btn, False)
        

        
        #         "controlState" : True,
        # "vEgo_in_ms" : 0.0,
        # "sas_angle" : 0.0,
        # "joy_angle" : 0.0,
        # "car_online" : False,
        
        ### Info title drawing
        INFO_SPACING = 20
        write_x = self.widthOfScreen - 550
        write_y = 540
        lines = [
            info_font.render("정상동작 : ENABLED", True, GREEN if self.first_setup else BLACK),
            info_font.render("Saving Log...", True, GREEN if self.on_logging else BLACK),
            info_font.render("차량 영상 전송 상태 : 정상", True, GREEN if self.camera_feed_online else BLACK),
            info_font.render("MDPS Module ENABLED", True, GREEN if self.sN.sub_data["MDPS Module Avail"] else BLACK),
            info_font.render("ACC Module ENABLED", True, GREEN if self.sN.sub_data["ACC Module Avail"] else BLACK),
            info_font.render("종합 연결 상태 신호 ", True, GREEN if self.sN.sub_data["ACC Module Avail"] else RED),
            None,
            info_font.render("CAR CONTROL STATE: " + CAR_MODE[int(self.sN.sub_data["ctrl_mode"])], True, YELLOW),
            info_font.render("SPEED: " + str(round(self.sN.sub_data["vEgo_in_ms"], 1)) + " km/h" + " / MAX speed set : " + str(self.sN.max_speed_in_kph), True, YELLOW),
            info_font.render("LONG MPC SOURCE: " + str("RESERVED"), True, YELLOW),
            None,
            info_font.render("Car SAS Angle: " + str(round(self.sN.sub_data["sas_angle"], 1)) + " deg", True, YELLOW),
            info_font.render("Joy Angle: " + str(round(self.sN.sub_data["joy_angle"], 3)) + " deg", True, YELLOW),
            info_font.render("Accel Pressed", True, GREEN if self.sN.sub_data["accel_pressed"] else YELLOW),
            info_font.render("Brake Pressed", True, GREEN if self.sN.sub_data["brake_pressed"] else YELLOW),
            info_font.render("RL Paddle Pressed", True, GREEN if self.sN.sub_data["RightPaddle_pressed"] and self.sN.sub_data["LeftPaddle_pressed"] else YELLOW),
            info_font.render("RTK UTM: " + str(self.sN.sub_data["RTK_X"]) + ", " + str(self.sN.sub_data["RTK_Y"]), True, YELLOW),
            None,
            info_font.render("MDPS Emulation STATE", True, GREEN if AUTO_CENTER else RED),
            info_font.render("Control Request State: " + str(bool(self.sN.sub_data["ctrl_request"])), True, GREEN if self.sN.sub_data["ctrl_request"] else RED),
        ]
    
        for i, line in enumerate(lines):
            if line is not None:
                screen.blit(line, (write_x, write_y + i * INFO_SPACING))
                
    def camera_loop(self):
        while(1):
            try:


                if int(self.sN.number_of_camera) == 1 and self.sN.camera_init:
                    img = self.sN.get_camera_img("fc")
                    if CAMERA_MODE == 'panorama':
                        img = cv2.resize(img, dsize= self.image_recv_size)
                    else:
                        img = cv2.resize(img, dsize= self.image_recv_size)

                    try:
                        img = hud.hud(self.sN.sub_data["sas_angle"], round(self.sN.sub_data["vEgo_in_ms"], 1), "D", None, img)
                    except Exception as e:
                        print(e)

                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                    self.camera_feed_online = True
                    pygame.surfarray.blit_array(self.camera_surface, img.swapaxes(0, 1))


                else:
                    pass
            except Exception as ex:
                self.camera_feed_online = False
                self.error_log.put(["Video Feed Error", str(ex)])
                self.sN.del_camera()
                self.sN.establishing_camera()

                
            
    def graph_draw(self):
        plot_arr[:-1] = plot_arr[1:]
        if not self.nE.network_delay.empty(): 
            delay_ms = self.nE.network_delay.queue.pop()
            self.sN.sub_data["delay_ms"] = delay_ms

            if delay_ms:
                plot_arr[-1, name_to_arr_idx["network_ping_delay"]] = delay_ms
            else:
                self.error_log.put("ERROR!", "Car Connection Interrupted")
        if not self.nE.network_bw.empty():
            #screen.blit
            bw = self.nE.network_bw.queue.pop()
            self.sN.sub_data["inOctets"] = bw
            if bw:
                plot_arr[-1, name_to_arr_idx["network_bandwidth"]] = bw / 1000 # b -> kb

        if self.joy_connected:        
            plot_arr[-1, name_to_arr_idx["angle_steers_des"]] = float(self.axis_info["steer"])
            plot_arr[-1, name_to_arr_idx["user_gas"]] = 100 - round(float(self.axis_info["acc"]))
            plot_arr[-1, name_to_arr_idx["user_brake"]] = 100 - round(float(self.axis_info["brk"]))
        
        plot_arr[-1, name_to_arr_idx["angle_steers"]] = -self.sN.sub_data["sas_angle"]    

        if self.on_logging:
            self.log_inst.get_ts_and_data()
        

        screen.blit(self.draw_plots(plot_arr), (self.widthOfScreen - 550, 30))
        

    very_sad_cnt = 0
    def start_log_btn(self):
        if self.on_logging and self.very_sad_cnt > 2:
            self.on_logging = False
            self.log_inst.close()
            self.very_sad_cnt = 0
        elif not self.on_logging and self.very_sad_cnt > 1:
            self.very_sad_cnt = 0
            self.log_inst = log_manager(self.sN)
            self.on_logging = True
        self.very_sad_cnt += 1

    def send_vms_reply(self, uid, remote_accept):
        try:
            if remote_accept:
                s_data = dict({"uid": uid, "msg": {"ctrl_remote" : 1} })
                resp = requests.post(url=SERVER_IP + str("/ctrl_status"), json=s_data).json()
                fmtc_remote.error_log.put(["MY CONTROL", "Control with care"])
            else:
                s_data = dict({"uid": uid, "msg": {"ctrl_remote" : 0} })
                resp = requests.post(url=SERVER_IP + str("/ctrl_status"), json=s_data).json()
                fmtc_remote.error_log.put(["CAR CONTROL", "Now in Car Control"])
        except:
            fmtc_remote.error_log.put(["VMS Reply Error", "VMS Server Connection Error"])

        

            
    def error_log_draw(self):
        if self.error_log.qsize() == 4:
            self.error_log.queue.popleft()
        temp_error = copy.deepcopy(self.error_log.queue)

        SPACING = 40
        for idx, error in enumerate(temp_error): 
            title_font = pygame.font.Font("./assets/hhm.ttf", 20 + idx*3)  
            desc_font = pygame.font.Font("./assets/hhm.ttf", 10 + idx * 3)  
            screen.blit(title_font.render(error[0], True, (255, 0, 0)), (self.camera_surface.get_rect()[0]+35, self.camera_surface.get_rect()[1] + 30 + (idx * SPACING)))
            screen.blit(desc_font.render(error[1], True, (255, 0, 0)), (self.camera_surface.get_rect()[0]+35, self.camera_surface.get_rect()[1] + 55 + (idx * SPACING)))

    def dialogue_draw(self):
        # make a dialogue box  and draw it on pygame
        # draw a overlay rectangle with center of the screen
        # "Control Request Received", "Press brake for 1s to accept"
        self.widthOfScreen, self.heightOfScreen = pygame.display.get_surface().get_size()

        if bool(self.sN.sub_data["ctrl_request"]) == True and bool(self.sN.sub_data["control_request_accept"]) == False:
            # if reqest is received and not accepted
            # draw a overlay rectangle with center of the screen with 50% opacity, below the camera surface 

            # draw a dialogue box with whole width
            dialogue_box = pygame.Surface((self.widthOfScreen, 200))

            # orange color and blinking with time.time's 1st digit is odd
            if int(time.time()) % 2 == 0:
                # redish orange
                dialogue_box.fill((255, 69, 0))
            else:
                dialogue_box.fill((255, 165, 0))
            # center of the screen
            dialogue_box.set_alpha(100)
            screen.blit(dialogue_box, (0, self.heightOfScreen/2 - 100))
            # draw a text
            dialogue_font = pygame.font.Font("./assets/hhm.ttf", 20)
            dialogue_title_font = pygame.font.Font("./assets/hhm.ttf", 30, bold=True)
            screen.blit(dialogue_title_font.render("[Attention] Control Request Received", True, (255, 255, 255)), (self.widthOfScreen/2 - 400 + 20, self.heightOfScreen/2 - 100 + 20))
            screen.blit(dialogue_font.render("Pull Both Paddles for 2s to accept", True, (255, 255, 255)), (self.widthOfScreen/2 - 400 + 20, self.heightOfScreen/2 - 100 + 50))

            # paddle instruction image
            screen.blit(self.instruction_paddle_img, (self.widthOfScreen/2 - 650, self.heightOfScreen/2 + - 100))

            # make a block ascii text based progress bar
            # draw a progress bar with "=" with 10 spaces |====================      |
            progress_bar = "["
            for i in range(20):
                if i < self.brake_pressed_time:
                    progress_bar += "=="
                else:
                    progress_bar += " "
            progress_bar += "]"
            screen.blit(dialogue_font.render(progress_bar, True, (0, 0, 0)), (self.widthOfScreen/2 - 400 + 20, self.heightOfScreen/2 - 100 + 80))

            # play sound
            if self.sad_cnt2 == 0:
                self.alarm_sound.play(loops=-1)

                self.sad_cnt2 += 1
             
            if self.sN.sub_data["RightPaddle_pressed"] and self.sN.sub_data["LeftPaddle_pressed"]:
                # accumulate time
                self.brake_pressed_time += 1
                
                if  self.brake_pressed_time > 11:
                    # Done, now control request is accepted draw
                     
                    self.sN.sub_data["control_request_accept"] = True
                    self.send_vms_reply(self.sN.sub_data["vehicle_uid"], True)

                    self.brake_pressed_time = 0
            else:
                self.brake_pressed_time = 0
        else:
            self.alarm_sound.stop()
            self.sad_cnt2 = 0
           
    def start_check(self):
        if not self.first_setup:
            self.first_setup = True
            self.start_thread.start()
        else:
            print("Button Ignored : Already set up")
            
    def start_setup(self): #차량 통신 시작 
        try:
            self.first_setup = True
            if self.joyAvailable:
                self.joyThread.start()
            else:
                print("No joystick detected, disable joy thread")

            self.error_log.put(["Connect to Chassis", ""])
            self.sN.establishing_info_connection()
            self.error_log.put(["Establishing Camera...", "work as server"])
            self.sN.establishing_camera(test=False)


        except Exception as ex:
            self.error_log.put(["Error on Init!", str(ex)])
        pass

    def reset_setup(self): #차량 통신 종료
        self.first_setup = False
        # find thread and kill it
        # check if thread is alive
        try:
            if self.joyThread.is_alive():
                self.joyThread.kill()
                print("Joy Thread Killed")
        except:
            print("No Joy Thread to kill")
        try:
            if self.sN.camera_daemon.is_alive():
                self.sN.camera_daemon.kill()
                print("Camera Thread Killed")
        except:
            print("No Camera Thread to kill")
        try:
            if self.sN.info_thread.is_alive():
                self.sN.info_thread.kill()
                print("Info Connection Thread Killed")
        except:
            print("No Info Connection Thread to kill")

            
      
    def ui_loop(self):
        clock.tick(30)
        self.widthOfScreen, self.heightOfScreen = pygame.display.get_surface().get_size()
        # Default
        
        self.bg_init()
        if self.sN.camera_init : screen.blit(self.camera_surface, (30, 30))
        if self.sN.camera_init == False: self.text_on_screen("Awaiting Video Feed... [Press Setup]", self.cur_time_font, 840, 400)
        
        # Pretty UI
        self.graph_draw()
        self.error_log_draw()
        self.dialogue_draw()
        self.camButton.process()
        self.logButton.process()
        
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.loop_enable = False
                os.kill(os.getpid(), 9)
                pygame.display.quit()
                pygame.quit()
                sys.exit()
        self.control_hud()
        pygame.display.flip()
        #self.widthOfScreen, self.heightOfScreen = pygame.display.get_surface().get_size()

    def control_hud(self):
        # Draw a rectangle with 50% opacity
        control_hud = pygame.Surface((self.image_recv_size[0], 50))
        control_hud.fill((255, 255, 255))
        control_hud.set_alpha(90)

        # Draw a text
        control_hud_font = pygame.font.Font("./assets/hhm.ttf", 20)
        control_hud_info = pygame.font.Font("./assets/hhm.ttf", 10)
        control_hud_title_font = pygame.font.Font("./assets/hhm.ttf", 30)
        # print(self.sN.sub_data["ctrl_request"])
        if not self.sN.sub_data.get("control_request_accept"):
            if self.sN.sub_data.get("ctrl_request"):
                # gray rectangle
                control_hud.fill((128, 128, 128))
                screen.blit(control_hud, (30, self.image_recv_size[1] + 30))
                # text "Incoming Control Reqest from UID"
                screen.blit(control_hud_font.render("Incoming Control Requests from {}".format(self.sN.sub_data["vehicle_alias"]), True, (255, 255, 255)), (30 + self.image_recv_size[0]//2 - 300, self.image_recv_size[1] + 30 + 15)) 
        else:
            # green rectangle
            control_hud.fill((0, 255, 0))
            screen.blit(control_hud, (30, self.image_recv_size[1] + 30))
            # text "Control Accepted"
            
            screen.blit(control_hud_font.render("Control Vehicle [{}]".format(self.sN.sub_data["vehicle_alias"]), True, (255, 255, 255)), (30 + self.image_recv_size[0]//2 - 50, self.image_recv_size[1] + 30 + 15))
            screen.blit(control_hud_info.render("Pull RL Paddle for 2s to takeover RM -- {}%".format(round((self.brake_pressed_time /201)*100)), True, (255, 255, 255)), (30 + self.image_recv_size[0]//2 - 50, self.image_recv_size[1] + 30 + 35))      
            
        
            # Pull RL Paddle for 2s to takeover RM
            if self.brake_pressed_time < 20:
                if self.sN.sub_data.get("RightPaddle_pressed") and self.sN.sub_data.get("LeftPaddle_pressed"):
                    self.brake_pressed_time += 1
                else:
                    self.brake_pressed_time = 0
            else:
                self.send_vms_reply(self.sN.sub_data["vehicle_uid"], False)
                self.sN.sub_data["control_request_accept"] = False
                self.brake_pressed_time = 0
                
        
        
    def text_on_screen(self, content, font, w, h):
        ''' Renders content with font '''
        fps_surface = font.render(content, 1, pygame.Color("white"))
        text_width = fps_surface.get_width()
        if w + text_width > screen.get_width():
            start = 0
            for word in content.split():
                wr = font.render(word + " ", 1, pygame.Color("white"))
                if w + start + wr.get_width() and screen.get_width():
                    screen.blit(wr, (w + start, h))
                    start += wr.get_width()
                else:
                    start = 0
                    h = h + 20
                    screen.blit(wr, (w + start, h))
                    start += wr.get_width()
        else:
            screen.blit(fps_surface, (w, h))
            

    def init_plots(self, arr, name_to_arr_idx, plot_xlims, plot_ylims, plot_names, plot_colors, plot_styles):
        color_palette = { "r": (1, 0, 0),
                            "g": (0, 1, 0),
                            "b": (0, 0, 1),
                            "k": (0, 0, 0),
                            "y": (1, 1, 0),
                            "p": (0, 1, 1),
                            "m": (1, 0, 1)}
        if IS_DARWIN:
            dpi = 50
            fig = plt.figure(figsize=(250 / dpi, 250 / dpi), dpi=dpi)
        else:
            dpi = 50
            fig = plt.figure(figsize=(550 / dpi, 550 / dpi), dpi=dpi)
        canvas = FigureCanvasAgg(fig)

        fig.set_facecolor((0, 0, 0))

        axs = []
        for pn in range(len(plot_ylims)):
            ax = fig.add_subplot(len(plot_ylims), 1, len(axs)+1)
            ax.set_xlim(plot_xlims[pn][0], plot_xlims[pn][1])
            ax.set_ylim(plot_ylims[pn][0], plot_ylims[pn][1])
            ax.patch.set_facecolor((0.4, 0.4, 0.4))
            axs.append(ax)

        plots, idxs, plot_select = [], [], []
        for i, pl_list in enumerate(plot_names):
            for j, item in enumerate(pl_list):
                plot, = axs[i].plot(arr[:, name_to_arr_idx[item]],
                                label=item,
                                color=color_palette[plot_colors[i][j]],
                                linestyle=plot_styles[i][j])
                plots.append(plot)
                idxs.append(name_to_arr_idx[item])
                plot_select.append(i)
            axs[i].set_title(", ".join(f"{nm} ({cl})"
                                    for (nm, cl) in zip(pl_list, plot_colors[i])), fontsize=10)
            axs[i].tick_params(axis="x", colors="white")
            axs[i].tick_params(axis="y", colors="white")
            axs[i].title.set_color("white")

            if i < len(plot_ylims) - 1:
                axs[i].set_xticks([])

        canvas.draw()

        def draw_plots(arr):
            for ax in axs:
                ax.draw_artist(ax.patch)
            for i in range(len(plots)):
                plots[i].set_ydata(arr[:, idxs[i]])
                axs[plot_select[i]].draw_artist(plots[i])

            raw_data = canvas.buffer_rgba()
            plot_surface = pygame.image.frombuffer(raw_data, canvas.get_width_height(), "RGBA").convert()
            return plot_surface

        return draw_plots
    
class network_eval:
    def __init__(self):

        self.network_delay = Queue(maxsize = 100)
        self.network_bw = Queue(maxsize = 100)

        self.car_ip = config["network"]["car_my_ip"]
        self.delay_daemon = threading.Thread(target =  self.get_delay, args=())
        self.delay_daemon.start()

        #self.bw_daemon = threading.Thread(target =  self.get_bw, args=())
        #self.bw_daemon.start()
        
    def get_bw(self): # 통신 대역폭 측정
        my_pid = os.getpid()
        magic_cmd = f'''cat /proc/{my_pid}/net/netstat''' + ''' |  awk '(f==0) {name=$1; i=2; while ( i<=NF) {n[i] = $i; i++ }; f=1; next} \
(f==1){ i=2; while ( i<=NF){ printf "%s%s = %d", name, n[i], $i; i++}; f=0} '
'''

        while 1:
            time.sleep(0.1)
            ret = os.popen(magic_cmd).readlines()
            for idx, item in enumerate(ret):
                if "IpExt:InOctets" in item:
                    inBytes = item.split("=")[-1].strip()
                    self.network_bw.put(inBytes)
                elif "IpExt:OutOctets" in item:
                    outBytes = item.split("=")[-1].strip()
                    
                
        ## TODO : check this scripts works on ubuntu pc

            
        
    def get_delay(self): # 차량 통신 지연 측정
        while 1:
            time.sleep(0.1)
            ret = os.popen(f"ping {self.car_ip} -c 1").readlines()
            if ret[1] == "\n": 
                self.network_delay.put(-1)
            else:
                delay = ret[1].split("=")[-1].split()[0]
                self.network_delay.put(delay)
                

class log_manager:

    def __init__(self, sN, save_topics=None):
        if save_topics == None:
            self.save_topics = [*list(sN.sub_data.keys())]
        else:
            self.save_topics = save_topics
        self.sN = sN
        self.open_and_write()

    def open_and_write(self):
        datetimeStr = datetime.now().strftime("%Y%m%d%H%M%S")
        self.csv_buffer = open(f'log_{datetimeStr}.csv', 'w', newline='')
        fieldnames = ['timestamp', *self.save_topics]
        self.writer = csv.DictWriter(self.csv_buffer, fieldnames=fieldnames)
        self.writer.writeheader()

    def get_ts_and_data(self): #로그 시간 데이터 생성 timestamp
        ref_ts = time.time()
        df = {'timestamp' : ref_ts}
        df.update(self.sN.sub_data)

        self.writer.writerow(df)

    def close(self):
        self.csv_buffer.close()


    
        

class feed_provider:
    
    sub_data = {
        "controlState" : True,
        "vEgo_in_ms" : 0.0,
        "sas_angle" : 0.0,
        "joy_angle" : 0.0,
        "car_online" : False,
        "MDPS Module Avail" : False,
        "ACC Module Avail" : False,
        "CMD_tick" : 0.0,
        "accel_pressed" : False,
        "brake_pressed" : False,
        "RTK_X" : 0.0,
        "RTK_Y" : 0.0,
        "delay_ms" : 0.0,
        "inOctets" : 0.0,
        "Ax_cmd" : 0.0,
        "ESTOP" : 0.0,
        "ctrl_request" : False,
        "control_request_accept" : False,
        "ctrl_mode" : 0, # 0 : normal, 1 : auto, 2 : remote
        "vehicle_uid" : 0,
        "vehicle_alias" : "None",
        "RightPaddle_pressed" : False,
        "LeftPaddle_pressed" : False,

        
    }
    
    sub_camera = {}
    
    def gst_pipeline_maker(self, port): #안쓰는 함수
        
        gst_pipeline = f'''
        'udpsrc port={port} caps = "application/x-rtp, encoding-name=H264, payload=96"'
        ' ! rtpjitterbuffer latency=0'
        ' ! rtph264depay'
        ' ! avdec_h264'
        ' ! decodebin'
        ' ! videoconvert'
        ' ! autovideosink sync=false'
        ' ! appsink'
        '''
        
        return gst_pipeline
        
    def __init__(self):
        self.car_my_ip = config["network"]["car_my_ip"]
        self.fmtc_my_ip = config["network"]["fmtc_my_ip"]
        self.udp_rtsp_port = int(config["network"]["udp_rtsp_port"])
        self.udp_info_port = int(config["network"]["car_to_fmtc_info"])
        
        self.tcp_rtsp_port = int(config["network"]["tcp_rtsp_port"])
        self.fmtc_car_port = int(config["network"]["fmtc_car_port"])
        
        self.number_of_camera = config["camera"]["number_of_camera"]
        self.camera_alias = config["camera"]["camera_alias"]
        self.resolution = config["camera"]["resolution"]

        self.max_speed_in_kph = config["control"]["max_speed_in_kph"]
        
        self.maxlength = 65540
        
        self.camera_init = False
        
        self.cmd_deque = deque(maxlen=1)
        self.cmd_live = {}
        self.recv_deque = deque(maxlen=1)

        self.vms_daemon_worker = threading.Thread(target=self.vms_daemon, daemon=True)
        self.vms_daemon_worker.start()

    def camera_connection_worker(self):
        print(self.gst_pipeline_maker(5118))
        #cap = cv2.VideoCapture(self.gst_pipeline_maker(5118), cv2.CAP_GSTREAMER)
        
        fmtc_remote.error_log.put(["Camera pipeline wait", VIDEO_COMM_MODE])
        if VIDEO_COMM_MODE == 'udp':
            self.cap = cv2.VideoCapture(
            'udpsrc port=5118 caps = "application/x-rtp, encoding-name=H264, payload=96" buffer-size=100000'
            ' ! rtpjitterbuffer latency=0'
            ' ! rtph264depay'
            ' ! avdec_h264'
            ' ! decodebin'
            ' ! videoconvert'
            ' ! appsink', cv2.CAP_GSTREAMER)
        
        elif VIDEO_COMM_MODE == 'tcp':
            self.cap = cv2.VideoCapture('tcpserversrc host=1.249.212.151 port=5118  \
            ! gdpdepay  \
            ! rtph264depay  \
            ! h264parse  \
            ! avdec_h264 \
            ! videoconvert \
            ! appsink', cv2.CAP_GSTREAMER)
            
            
        # elif VIDEO_COMM_MODE == 'tcp':
        #     self.cap = cv2.VideoCapture('tcpserversrc host=127.0.0.1 port=4566  \
        #     ! gdpdepay  \
        #     ! rtph264depay  \
        #     ! nvv4l2decoder  \
        #     ! nvvidconv \
        #     ! video/x-raw, format=(string)BGRx \
        #     ! videoconvert \
        #     ! video/x-raw,format=BGR \
        #     ! appsink drop=1', cv2.CAP_GSTREAMER)

        # self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if self.cap.read()[0]:
            self.sub_camera.update({"fc" : self.cap})
            self.camera_init = True
            fmtc_remote.error_log.put(["Camera Done", ""])
        else:
            fmtc_remote.error_log.put(["GST Open Error", ""])
            self.camera_init = False
            #raise UnexpectedException("Cant Open GST Video")
    
    def del_camera(self):
        self.camera_init = False
        self.cap.release()
        cv2.destroyAllWindows()
        


    def vms_daemon(self):
        while True:
            try:
                active_vehicle = requests.post(url=SERVER_IP + str("/forVVS"), timeout=2).json()
                # print(active_vehicle)
                resp = requests.post(url=SERVER_IP + str("/forVCS"), timeout=2).json()
                # print(resp)
                t_data = {"ctrl_request" : 0 , "ctrl_mode" : 0, "remote" : 0}
                
                for item in resp:
                    if int(item[2]) == 1:
                        self.sub_data.update({"vehicle_uid" : item[0]})
                        self.sub_data.update({"vehicle_alias" : active_vehicle.get(item[0])})
                        t_data["ctrl_request"] = int(item[2]) # idx 0 is uid
                        t_data["ctrl_mode"] = int(item[3])
                        t_data["remote"] = int(item[4])
                        
                self.sub_data.update({"ctrl_request" : t_data["ctrl_request"]})
                self.sub_data.update({"ctrl_mode" : t_data["ctrl_mode"]})

                if t_data["ctrl_request"] == 0 and bool(self.sub_data["control_request_accept"]) == True:
                    # This for re-entering control mode when it's fixed and re-occured
                    self.sub_data.update({"control_request_accept" : False})

                #self.sub_data.update({"remote__accept_reply" : t_data["remote"]})

            except Exception as e:
                print(e)
                pass

            time.sleep(0.5)
            
        pass


      
    def establishing_camera(self, test=False): #카메라 영상 데이터 취득
        if test: 
            cap = cv2.VideoCapture("./assets/test.mp4")
            if cap.read()[0]:
                self.sub_camera.update({"fc" : cap})
                self.camera_init = True
            else:
                raise UnexpectedException("Cant Open Test Video")
                self.camera_init = False
            
        else:
            # check camera daemon is alive thread
            # check camera_daemon is instance of threading.Thread
            try:
                if isinstance(self.camera_daemon, threading.Thread):
                    if self.camera_daemon.is_alive():
                        # kill camera daemon
                        print("Trying to kill camera daemon")
                        self.camera_daemon.join()
                        print("Camera Daemon is killed")
                        self.camera_init = False
            except:
                pass
        
            self.camera_daemon = threading.Thread(target=self.camera_connection_worker, args=())
            self.camera_daemon.start()

            
    def tcp_daemon(self):
        while 1:
            try:
                server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server_socket.bind(("", self.fmtc_car_port))
                fmtc_remote.error_log.put(["TCP Server Waiting", ""])
                print ("TCP Server Waiting")
                server_socket.listen(5)
                
                client_socket, address = server_socket.accept()
                print ("Incoming Connection Request", address)
                
                # Auth
                data = client_socket.recv(1024)
                data = data.decode()
                if data == PAIR_KEY:
                    print("Auth Success")
                    client_socket.send("OK".encode()) 
                    fmtc_remote.error_log.put(["Accepting Car", ""])
                    time.sleep(1) 
                else:
                    print("Auth Failed, reset connection")
                    self.sub_data['car_online'] = False
                    client_socket.send("FAIL".encode())  
                    client_socket.close()
                    server_socket.close()
                    continue
                
                self.sub_data['car_online'] = True
                
                while 1:

                    if len(self.cmd_deque) == 0:
                        #data_pack = dict({"ts" : time.time(), "data" : "Hello World", "control_request" : False, "control_accel" : 100, "control_brake" : 100, "control_swa" :  0}) 
                        cmd_data = {"axis0":0,"axis1":0,"axis2":100,"axis3":100}
                        pass
                    else:
                        cmd_data = self.cmd_live #self.cmd_deque.popleft()

                    cmd_data = self.control_limit_check(cmd_data)
                    data_pack = dict({"ts" : time.time(), "data" : "Hello World", "control_request" : True, "control_accel" : cmd_data['axis2'], "control_brake" : cmd_data['axis3'], "control_swa" :  cmd_data['axis0']})
                    # self.send_cmd =  {"axis0":tmp[0],"axis1":tmp[1],"axis2":tmp[2],"axis3":tmp[3]} # steer, cluth, acc, brk
                    data = pickle.dumps(data_pack, protocol=2)
                

                    if self.sub_data['ctrl_mode'] == 2 and bool(self.sub_data['control_request_accept']) == True:
                        client_socket.send(data)

                    else:
                        cmd_data = {"axis0":0,"axis1":0,"axis2":100,"axis3":100}
                        data_pack = dict({"ts" : time.time(), "data" : "Hello World", "control_request" : False, "control_accel" : cmd_data['axis2'], "control_brake" : cmd_data['axis3'], "control_swa" :  cmd_data['axis0']})
                        client_socket.send(data)
                    
                    tcp_recv_dict = client_socket.recv(1024)
                    self.unpack_tcp_recv(tcp_recv_dict)
                    
                    time.sleep( 1.0 / 60.0 )

                    

            except KeyboardInterrupt:
                server_socket.close()
                print("Bye")
                sys.exit(0)

            except Exception as ex:
                print("Connection Closed... Standby", str(ex))
                self.sub_data['car_online'] = False
                #server_socket.bind(("", 5778))
                time.sleep(2)
                try:
                    client_socket.close()
                    server_socket.close()
                except:
                    pass
                
    def control_limit_check(self, data):
        # acc range : 100 ~ 0, 100 is no press, 0 is full press

        # 2023/07 added regulation
        # if car over max speed, give 0 acc
        if float(self.sub_data["vEgo_in_ms"]) > float(self.max_speed_in_kph): # despite vEgo variable name is ms unit, it is actually km/h
            data['axis2'] = 100

            # if int(time.time()) % 5 == 0: fmtc_remote.error_log.put(["Over Max Speed", str(self.sub_data["vEgo_in_ms"])])

        return data

                
    def unpack_tcp_recv(self, info_data):
        info_pickle = pickle.loads(info_data)   
        if info_pickle:
            try:
                self.sub_data["vEgo_in_ms"] = float(info_pickle["Vx"])
                self.sub_data["sas_angle"] = float(info_pickle["SAS_angle"])
                self.sub_data["MDPS Module Avail"] = info_pickle["MDPS_Enable_Status"]
                self.sub_data["ACC Module Avail"] = info_pickle["ACC_Enable_Status"]
                self.sub_data["RTK_X"] = info_pickle["gps_x"] # UTM 형식임
                self.sub_data["RTK_Y"] = info_pickle["gps_y"]
                self.sub_data["ESTOP"] = info_pickle["Emergency_Stop_Status"]
                self.sub_data["Ax_cmd"] = info_pickle["Ax_Cmd"]
            except Exception as ex:
                fmtc_remote.error_log.put(["Error on unpickling info data", str(ex)])
        pass

            
    ## TODO add Checksum
    def establishing_info_connection(self): #차량 정보 수신
        
        self.info_thread = threading.Thread(target = self.tcp_daemon, args=())
        self.info_thread.start()
        
    # ## TODO Add Packet loss statistics Car MDPS CMD Failure case
    # def info_sock_daemon(self): # 차량 정보 해석 쓰레드
    #     while 1:
    #         if self.sock2:
    #             info_data, addr = self.sock2.recvfrom(self.maxlength)
    #             info_pickle = pickle.loads(info_data)
                
    #             if info_pickle:
    #                 try:
    #                     self.sub_data["vEgo_in_ms"] = float(info_pickle["Vx"])
    #                     self.sub_data["sas_angle"] = float(info_pickle["sas_angle"])
    #                     self.sub_data["MDPS Module Avail"] = info_pickle["MDPS_Module_Stat"]
    #                     self.sub_data["ACC Module Avail"] = info_pickle["ACC_Module_Stat"]
    #                     self.sub_data["RTK_X"] = info_pickle["RTK"][0] # UTM 형식임
    #                     self.sub_data["RTK_Y"] = info_pickle["RTK"][1]
    #                 except Exception as ex:
    #                     fmtc_remote.error_log.put(["Error on unpickling info data", str(ex)])
        
                      
            
    def get_camera_img(self, alias='fc'): # 카메라 영상 데이터를 자료구조형식으로 변경
        if self.camera_init:
            return self.sub_camera.get(alias).read()[1]
        else:
            return -1
        
            
        
    
if __name__ == "__main__":
    fmtc_remote = main()
    try:
        while fmtc_remote.loop_enable:
            fmtc_remote.ui_loop()
    except:
        pass
    
