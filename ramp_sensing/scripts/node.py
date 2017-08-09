#!/usr/bin/env python
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy import linalg, optimize
import functools

xs = np.r_[57, 57, 60, 60, 59, 59, 58, 57, 57, 56]
ys = np.r_[39, 36, 36, 38, 39, 44, 43, 44, 42, 41]

def main():
    print 'In main'

    # Each list is a row in the matrix
    a = np.array([ [3,1], [1,2]])
    b = np.array([9,8])

    basename = 'circle'

    method = 'algebraic'

# Centroid
    x_m = np.mean(xs)
    y_m = np.mean(ys)

# Calculation of the reduced coordinates
    u = xs - x_m
    v = ys - y_m

    
# linear system defining the center in reduced coordinates (uc, vc):
#    Suu * uc +  Suv * vc = (Suuu + Suvv)/2
#    Suv * uc +  Svv * vc = (Suuv + Svvv)/2
    Suv  = sum(u*v)
    Suu  = sum(u**2)
    Svv  = sum(v**2)
    Suuv = sum(u**2 * v)
    Suvv = sum(u * v**2)
    Suuu = sum(u**3)
    Svvv = sum(v**3)


# Solving the linear system
    A = np.array([ [ Suu, Suv ], [Suv, Svv]])
    B = np.array([ Suuu + Suvv, Svvv + Suuv ])/2.0
    uc, vc = linalg.solve(A, B)

    xc_1 = x_m + uc
    yc_1 = y_m + vc


# Calculation of all distances from the center (xc_1, yc_1)
    Ri_1      = np.sqrt((xs-xc_1)**2 + (ys-yc_1)**2)
    R_1       = np.mean(Ri_1)
    residu_1  = sum((Ri_1-R_1)**2)
    residu2_1 = sum((Ri_1**2-R_1**2)**2)

    print 'xc_1: %d' % xc_1
    print 'yc_1: %d' % yc_1
    print Ri_1
    print 'R_1: %d' % R_1
    print 'residu_1: %d' % residu_1
    print 'residu2_1: %d' % residu2_1

    
    # Now, apply geometric method 
    method_2 = 'leastsq'


    center_estimate = x_m, y_m
    center_2, ier = optimize.leastsq(f_2, center_estimate)

    xc_2, yc_2 = center_2
    Ri_2       = calc_R(xc_2, yc_2)
    R_2        = Ri_2.mean()
    residu_2   = sum((Ri_2 - R_2)**2)
    residu2_2  = sum((Ri_2**2-R_2**2)**2)
    ncalls_2   = f_2.ncalls

    print 'xc_2: %d' % xc_2
    print 'yc_2: %d' % yc_2
    print Ri_2
    print 'R_2: %d' % R_2
    print 'residu_2: %d' % residu_2
    print 'residu2_2: %d' % residu2_2
    print 'ncalls_2: %d' % ncalls_2


# Decorator to count functions calls
def countcalls(fn):
    "decorator function count function calls "

    @functools.wraps(fn)
    def wrapped(*args):
        wrapped.ncalls +=1
        return fn(*args)

    wrapped.ncalls = 0
    return wrapped

def calc_R(xc, yc):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    return np.sqrt((xs-xc)**2 + (ys-yc)**2)

@countcalls
def f_2(c):
    """ calculate the algebraic distance between the 2D points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c)
    return Ri - Ri.mean()




    a = np.array(xs, ys, np.ones(10))
    x = linalg.solve(a,b)
    print x

    print 'Exiting normally'



if __name__ == '__main__':
    main()
