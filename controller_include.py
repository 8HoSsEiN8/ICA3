# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16
# import ach
# import sys

DEF_1 = 0
DEF_2 = 1
DEF_3 = 2


CONTROLLER_REF_NAME              = 'controller-ref-chan'

class CONTROLLER_REF(Structure):
    _pack_ = 1
    _fields_ = [("x",    c_int16),
                ("y",    c_int16)]
#class HUBO_REF(Structure):
#    _pack_ = 1
#    _fields_ = [("ref",    c_double*HUBO_JOINT_COUNT),
#                ("mode",   c_int16*HUBO_JOINT_COUNT),
#                ("comply", c_ubyte*HUBO_JOINT_COUNT)]
