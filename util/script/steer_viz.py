import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
LF = 2.7 / 2            # Front distance from the center of mass of the car.
LB = 2.7 / 2            # Back distance from the center of mass of the car.
LW = 1.82 / 2                  # Half of the width of the vehicle.
axcolor = 'lightgoldenrodyellow'
from matplotlib.widgets import Slider, Button, RadioButtons
fig, ax = plt.subplots()
vx_car_in_ms = 0.0
str_car_in_deg = 0.0
l, = plt.plot([0], [0], lw=2, color='red')
img = plt.imread("./assets/test_snap.png")

#plt.axis([0, 1, -10, 10])
axfreq = plt.axes([0.25, 0.1, 0.65, 0.03])
axamp = plt.axes([0.25, 0.15, 0.65, 0.03])

sfreq = Slider(axfreq, 'Spexed', 0.1, 30.0, valinit=vx_car_in_ms)
samp = Slider(axamp, 'Steer Deg', -30.0, 30.0, valinit=str_car_in_deg)
plt.subplots_adjust(left=0.25, bottom=0.25)

class Calibration:
    def __init__(self, num_px, rpy, intrinsic, height=0.0):
        self.intrinsic = intrinsic
        view_from_road = self.eulerAnglesToRotationMatrix(rpy)
        self.extrinsics_matrix = np.hstack((view_from_road, [[0], [height], [0]]))[:,:3]
        
        #self.zoom = _CALIB_BB_TO_FULL[num_px][0, 0]
        self.zoom = 1 #3852/1920

    def car_space_to_ff(self, x, y, z):
        car_space_projective = np.column_stack((x, y, z)).T

        ep = self.extrinsics_matrix.dot(car_space_projective)
        # ep = np.array([-0.009018723852932453, -0.9999593496322632, 0.00012400432024151087, 0.0, 
        #          -0.013748353347182274, 0.0, -0.9999054670333862, 1.2200000286102295, 
        #          0.9998648166656494, -0.009019576013088226, -0.013747794553637505, 0.0]).reshape((3, 4))
        kep = self.intrinsic.dot(ep)
        return (kep[:-1, :] / kep[-1, :]).T #
        
    def car_space_to_bb(self, x, y, z):
        pts = self.car_space_to_ff(x, y, z)
        return pts / self.zoom

    def eulerAnglesToRotationMatrix(self, theta) : #RPY

        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])

        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])

        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])

        R = np.dot(R_z, np.dot( R_y, R_x )) # .dot(np.diag([1, -1, -1]))
        # XYZ ZXY
        device_frame_from_view_frame = np.array([
        [ 0.,  0.,  1.],
        [ 1.,  0.,  0.],
        [ 0.,  1.,  0.],

        ])
        view_frame_from_device_frame = device_frame_from_view_frame.T
        view_from_road = view_frame_from_device_frame.dot(R)
        return view_from_road
    
focal = 567.0
print(img.shape)
full_size = (img.shape[1], img.shape[0])
_INTRINSICS = np.array([
                            [focal, 0., 3852 / 2],
                            [0., focal, 2410 / 2],
                            [0., 0., 1.]
                            ])

e_cam = Calibration(1, [-0.00012310677266214043, 0.013748787343502045, -0.009019698016345501], _INTRINSICS)



def update(val):
    sas = samp.val
    spd = sfreq.val
    ax.cla()
    #print()
    x, y = get_traj2(spd, sas)
    #print("org", x, y)
    ax.imshow(img)
    z = np.ones_like(x).astype(np.float32)
    pts  = e_cam.car_space_to_bb(x, y, z)
    pts = np.round(pts).astype(np.int32)
    # translation



    x = pts[: ,0]
    y = pts[:, 1]
    
    # y = np.subtract(img.shape[0], y)
    # x = np.add(-300, x)
    
    ax.scatter(x, y, color='red')

    ax.axis([0, img.shape[1], img.shape[0], 0])
    

sfreq.on_changed(update)
samp.on_changed(update)

def get_traj(ctrl_vel_in_ms, ctrl_angle_in_deg, ):
    prev_phi = 0
    pos_x, pos_y = [0], [0]
    vel = ctrl_vel_in_ms
    steering = np.deg2rad(ctrl_angle_in_deg)

    # Calculate heading of the vehicle.
    
    # max 20m view
    target_len = 20 # in meters
    elspaed_t = target_len / vel
    resolution = target_len * 2 # 0.5m
    print(elspaed_t)

    for len_m in range(0, resolution):
        beta = np.arctan(LB * np.tan(steering) / (LB + LF))
        phi = prev_phi + ((elspaed_t / resolution) * vel * np.cos(beta) * np.tan(steering)) / (LB + LF)
        prev_phi = phi

        # Calculate vehicles's position.
        pos_x.append(pos_x[-1] + (elspaed_t / resolution) * vel * np.cos(beta + phi))
        pos_y.append(pos_y[-1] + (elspaed_t / resolution) * vel * np.sin(beta + phi))
        
    #print(pos_x, pos_y)
    return pos_x, pos_y

def get_traj2(ctrl_vel_in_ms, ctrl_angle_in_deg, ): # time invariant
    prev_phi = 0
    pos_x, pos_y = [0], [0]
    vel = ctrl_vel_in_ms
    steering = np.deg2rad(ctrl_angle_in_deg)

    # Calculate heading of the vehicle.
    
    # max 20m view
    target_len = 7 # secs
    elspaed_m =  min(vel * target_len, 40)
    resolution = 40 # 0.5m
    print(elspaed_m)

    for len_m in np.linspace(0, resolution):
        beta = np.arctan(LB * np.tan(steering) / (LB + LF))
        phi = prev_phi + (elspaed_m / resolution * np.cos(beta) * np.tan(steering)) / (LB + LF)
        prev_phi = phi

        # Calculate vehicles's position.
        pos_x.append(pos_x[-1] + elspaed_m / resolution * np.cos(beta + phi))
        pos_y.append(pos_y[-1] + elspaed_m / resolution * np.sin(beta + phi))
        
    #print(pos_x, pos_y)
    return pos_x, pos_y

        

plt.show()