#main.py
import multiprocessing
import time
import numpy as np
import sys
import getopt
# import scipy.io as sio

def calc():
    x=233
    for i in xrange(1000000000):
        x=x+1
        x=x-1

def writeln(x,y):
    aa=x*10+y
    print(aa)
    return(aa)

def func1(x):
    calc()
    c1=0
    d1=np.zeros(233,int)
    for i in xrange(1):
        d1[c1]=writeln(1,i)
        c1+=1
        #time.sleep(1)
    # sio.savemat('11.mat',{'dd':d1})

def func2(x):
    calc()
    c2=0
    d2=np.zeros(233,int)
    for i in xrange(5):
        d2[c2]=writeln(2,i)
        c2+=1
        #time.sleep(1)
    sio.savemat('22.mat',{'dd':d2})

def func3(x):
    calc()
    c3=0
    d3=np.zeros(233,int)
    for i in xrange(5):
        d3[c3]=writeln(3,i)
        c3+=1
        #time.sleep(1)
    sio.savemat('33.mat',{'dd':d3})

def func4(x):
    calc()
    c4=0
    d4=np.zeros(233,int)
    for i in xrange(5):
        d4[c4]=writeln(4,i)
        c4+=1
        #time.sleep(1)
    sio.savemat('44.mat',{'dd':d4})

def f1(x):
    print x
    for i in xrange(100000000):
        # print r[x]
        x += 1

    return x

from random import *

if __name__ == "__main__":
    a = [1,2,3]
    b = [1,2,3]
    c = [a,b,b]
    d = [b,a,a]
    print a == b
    print c == d
    # print hash(a)
    # print hash(b)
    # print randint(0,0)
    # opts, args = getopt.getopt(sys.argv[2:], "t:s:")
    # print opts
    # print args
    # cores = multiprocessing.cpu_count()
    # cores = 7
    # pool = multiprocessing.Pool(processes=cores)
    #
    # tt = time.time()
    # tc = time.clock()
    # print tt,tc
    #
    # r = range(cores)
    # print(cores)
    # results = []
    # for i in range(cores):
    #     results.append(pool.apply_async(f1, (i,)))
    #
    # pool.close()
    # pool.join()
    # for res in results:
    #     print res.get()
    #
    # print time.time() - tt
    # print time.clock() - tc
    # print "Sub-process(es) done."
