import ctypes
lib = ctypes.cdll.LoadLibrary('./g29_lib_shinkansan.so')
import threading
'''
std::string m_device_name;
double m_loop_rate;
double m_max_torque;
double m_min_torque;
double m_brake_position;
double m_brake_torque_rate;
double m_auto_centering_max_torque;
double m_auto_centering_max_position;
double m_eps;
bool m_auto_centering;
'''
class g29_param(ctypes.Structure):
    _fields = [("m_device_name", ctypes.c_char_p),
                ("m_loop_rate", ctypes.c_double),
                ("m_max_torque", ctypes.c_double),
                ("m_min_torque", ctypes.c_double),
                ("m_brake_position", ctypes.c_double),
                ("m_brake_torque_rate", ctypes.c_double),
                ("m_auto_centering_max_torque", ctypes.c_double),
                ("m_auto_centering_max_position", ctypes.c_double),
                ("m_eps", ctypes.c_double),
                ("m_auto_centering", ctypes.c_bool)
                
                ]

class g29_lib(object):
    def __init__(self):
        #lib.g29_ff.argtypes = [ctypes.c_void_p]
        lib.g29_ff.restype = ctypes.c_void_p

        lib.g29_loop.argtypes = [ctypes.c_void_p]
        lib.g29_loop.restype = ctypes.c_void_p

        self.obj = lib.g29_ff()


    def loop(self):

        lib.g29_loop(self.obj)

