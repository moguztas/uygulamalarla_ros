#!/usr/bin/env python
import rospy
from ogretici_paket.msg import BataryaDurum

def mesaj_yayinla():
    pub = rospy.Publisher('batarya', BataryaDurum, queue_size=10)
    rospy.init_node('yayinci_dugumu', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        durum = "%86"
        rospy.loginfo(durum)
        pub.publish(durum)
        rate.sleep()

if __name__ == '__main__':
    try:
        mesaj_yayinla()
    except rospy.ROSInterruptException:
        pass
