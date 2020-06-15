#!/usr/bin/env python
import rospy
from ogretici_paket.msg import BataryaDurum

def batarya_fonksiyonu(mesaj):
    rospy.loginfo("Robotun sarji: %s", mesaj.batarya)
    
def mesaj_dinle():

    rospy.init_node('abone_dugumu', anonymous=True)
    rospy.Subscriber("batarya", BataryaDurum, batarya_fonksiyonu)
    rospy.spin()

if __name__ == '__main__':
    mesaj_dinle()
