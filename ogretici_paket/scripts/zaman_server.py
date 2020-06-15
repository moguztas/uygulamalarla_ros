#!/usr/bin/env python

from ogretici_paket.srv import GecenZaman
import rospy

def gecen_zaman_fonksiyonu(istek):
    robot_hiz = 0.5
    print ("Robot: %d metre ileriye gidecek."%(istek.hedef_konum))
    sure = istek.hedef_konum / robot_hiz
    return sure

def cevap_gonder():
    rospy.init_node('server_dugumu')
    s = rospy.Service('zaman', GecenZaman, gecen_zaman_fonksiyonu)
    print("Hedef konum bilgisi bekleniyor...")
    rospy.spin()

if __name__ == "__main__":
    cevap_gonder()