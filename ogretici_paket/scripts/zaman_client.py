#!/usr/bin/env python

import sys
import rospy
from ogretici_paket.srv import GecenZaman

def istekte_bulun(x):
    rospy.wait_for_service('zaman')
    try:
        sure_hesapla = rospy.ServiceProxy('zaman', GecenZaman)
        cevap = sure_hesapla(x)
        return cevap.gecen_sure
    except rospy.ServiceException:
        pass

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = int(sys.argv[1])
    else:
        sys.exit(1)
    print ("Gidilecek mesafe: %d metre"%(x))
    print ("Hedefe varana kadar gecen sure: %d saniye."%(istekte_bulun(x)))