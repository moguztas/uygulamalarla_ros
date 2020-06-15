#!/usr/bin/env python
# coding=utf-8
"""
Uygulama Adı      : Hedef Noktaya Varma Uygulaması
Uygulama Amacı    : Robotun odom konusundan konumunu, hedef_konum konusundan gideceği hedefi alıp, 
                    hedef nokta ile robot arasındaki mesafe farkını hesaplayarak, 
                    servis cevabına göre hız değeri üretmesi.
"""
# Gerekli kütüphanelerin eklenmesi
# hedefe_git.srv : Oluşturulan servis dosyasını ekler.
# rospy          : Python kütüphaneleri ile çalışmak için gereklidir.
from hedefe_git.srv import VarisDurum
import rospy

# Servis callback fonksiyonu: gelen isteğin değerlendirilmesini sağlar ve uygun cevap döner.
def servis_callback(istek):
    if istek.fark <= 0.1:
        return True
    else:
        return False

# Hedefe varılıp varılmadığını kontrol eden servisin tanımlanması
def varis_servisi():
    rospy.init_node('varis_durum')
    rospy.Service('varis', VarisDurum, servis_callback)
    rospy.spin()

# ROSInterruptException istisnası olmadığı müddet boyunca oluşturulan fonksiyonu test et.
if __name__ == "__main__":
    try:
        varis_servisi()
    except rospy.ROSInterruptException:
        pass