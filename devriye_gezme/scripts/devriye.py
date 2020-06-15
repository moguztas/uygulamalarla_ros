#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Uygulama Adı     : Devriye Gezme Uygulaması
Uygulama Amacı   : Pakette bulunan parametreleri okuyarak, robotun devriye gezmesini sağlamak
"""
# Gerekli kütüphanelerin eklenmesi.
# rospy                 : Python kütüphaneleri ile çalışmak için gereklidir.
# geometry_msgs.msg     : Hız üretmek için kullanılacak mesaj tipi paketidir.
# math                  : Mutlak değer almak için (fabs) kullanılan pakettir.
import rospy
from geometry_msgs.msg import Twist
from math import fabs

# Devriye gezmeyi sağlayacak ana fonksiyon
def volta_at():
    # Yeni düğüm başlatılıyor (volta)
    rospy.init_node('volta', anonymous=True)
    # Konu (cmd_vel) yayınlanıp değişkende tutuluyor (hiz_yayinla)
    hiz_yayinla = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # hiz_mesaj Twist tipinde verileri tutuyor.
    hiz_mesaj = Twist()
    # Robot hızı tanımlanıyor.
    robot_hiz = 0.25
    # Devriye gezme ile alakalı parametreler .yaml dosyasında alınıyor.
    volta_uzunluk = rospy.get_param("/VoltaUzunluk")
    volta_sayisi = rospy.get_param("/VoltaSayisi")
    # Atılan her turu saymak için bir değişken tanımlanıyor.
    volta_sayici = 0
    # Bilgilendirme mesajı ekrana bastırılıyor.
    rospy.loginfo("Robot devriye gezmeye başladı...")
    # Tur sayısı parametreden gelen sayıya eşit olana kadar tur atması sağlanıyor.
    while volta_sayici <= volta_sayisi:
        # Mesafe hesabı yapmak için, anlık zaman saniye cinsinden tutuluyor.
        t0 = rospy.Time.now().to_sec()
        # Gezilen mesafe başlangıçta sıfırlanıyor.
        gezilen_mesafe = 0
        # Eğer tur sayıcının değeri çift ise pozitif hız, tek ise negatif hız veriliyor.
        if volta_sayici %2 == 0:
            hiz_mesaj.linear.x = robot_hiz
        else:
            hiz_mesaj.linear.x = -robot_hiz
        # Gezilen mesafe tur uzunluğuna eşit olana kadar hareket etmesi sağlanıyor.    
        while gezilen_mesafe < volta_uzunluk:
            # hiz_mesaj değeri yayınlanıyor.
            hiz_yayinla.publish(hiz_mesaj)
            # Mesafe hesabı yapmak için, anlık zaman saniye cinsinden tutuluyor.
            t1 = rospy.Time.now().to_sec()
            # Gezilen mesafe x = v*t formülü ile hesaplanıyor.
            gezilen_mesafe = fabs(robot_hiz*(t1-t0))
        # Her tur başında robot hızı sıfırlanıp, yayınlanıyor.
        hiz_mesaj.linear.x = 0
        hiz_yayinla.publish(hiz_mesaj)
        # Tur sayıcısı 1 artırılıyor.
        volta_sayici = volta_sayici + 1
    # ROS devre dışı bırakılıyor ve bilgi mesajı yazılıyor.
    rospy.loginfo("Devriye gezme tamamlandı.")
    rospy.is_shutdown()

# ROSInterruptException istisnası olmadığı müddet boyunca oluşturulan fonksiyonu test et.
if __name__ == '__main__':
    try:
        volta_at()
    except rospy.ROSInterruptException:
        pass