# Project: Python wrapper for C++ class Odom
# Created Date: October 15th 2021
import ctypes

lib = ctypes.cdll.LoadLibrary('./build/libmylib.so')

class Odo(object):

    def __init__(self):
        lib.Dist_corr.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_int]
        lib.Angle_corr.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_int]

        self.obj = lib.Odom_constructor()

    def distance(self, dist_inp, dir_inp):
        return lib.Dist_corr(self.obj, dist_inp, dir_inp)
    
    def angle(self, angl_inp, dir_inp):
        return lib.Angle_corr(self.obj, angl_inp, dir_inp)



o = Odo()
# Call the o.distance(dist_inp, dir_inp) OR o.angle(angl_inp, dir_inp)


