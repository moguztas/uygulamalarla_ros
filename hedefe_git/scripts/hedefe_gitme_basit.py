#!/usr/bin/env python
# coding=utf-8
"""
Uygulama Adı      : Hedef Noktaya Varma Uygulaması - Basit Versiyon
Uygulama Amacı    : Robotun odom konusundan konumunu, başlangıçta tanımlanan değişkenden gideceği hedefi alıp, 
                    hedef nokta ile robot arasındaki mesafe farkını hesaplayarak, uygun hız değeri üretmesi.
"""
# Gerekli kütüphanelerin eklenmesi
# geometry_msgs.msg         : Twist ve Point mesajlarını ekler.
# nav_msgs.msg              : Odometry mesajını ekler.
# rospy                     : Python kütüphaneleri ile çalışmak için gereklidir.
# math                      : Matematiksel işlemleri yapan kütüphanedir.
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
import math

# HedefeGitme sınıfı oluşturuluyor ve tüm işlemler bu sınıfın özellikleri (attribute) ve yöntemleri (method) olarak tanımlanıyor.
class HedefeGit():
    # Başlangıç özellik değerleri ayarlanıyor.
    def __init__(self):
        self.kontrol = False
        self.guncel_konum = Point()
        self.hedef_konum = Point()
        self.hedef_konum.x = 4.0
        self.hedef_konum.y = 0.0
        self.hedefe_uzaklik = 0.0
        self.ana_fonksiyon()
    
    # odom konusuna abone olunuyor. cmd_vel konusunun yayımlanması için tanımlayıcı oluşturuluyor.
    def ana_fonksiyon(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        hiz_yayinla = rospy.Publisher('/cmd_vel', Twist, queue_size=10)   
        hiz_mesaj = Twist()
        rate = rospy.Rate(10)
        # ROS kapanana kadar, döngü hızında bu işlemlerin yapılması sağlanıyor.
        while not rospy.is_shutdown():
            # Eğer kontrol değeri True ise, hedefe varılmadığı için, pozitif veya negatif hız mesajı yayınlanır.
            if self.kontrol:
                if self.hedef_konum.x > self.guncel_konum.x:
                    hiz_mesaj.linear.x = 0.1
                    hiz_yayinla.publish(hiz_mesaj)
                else:
                    hiz_mesaj.linear.x = -0.1
                    hiz_yayinla.publish(hiz_mesaj)
            # Eğer kontrol değeri False ise, hedefe varıldığı için, robotun hızı 0.0 olarak güncellenir.
            else:
                hiz_mesaj.linear.x = 0.0
                hiz_yayinla.publish(hiz_mesaj)

            rate.sleep()
    # Yayınlanan odom konusunda robotun konum değerleri ve hedefe uzaklığı güncellenir. Değerler ekrana bastırılır.
    def odom_callback(self, deger):
        self.guncel_konum.x = deger.pose.pose.position.x
        self.guncel_konum.y = deger.pose.pose.position.y
        self.hedefe_uzaklik = math.sqrt(pow(self.guncel_konum.y - self.hedef_konum.y, 2) + pow(self.guncel_konum.x - self.hedef_konum.x, 2))
        print("Robotun güncel x konumu: {} m\nHedefe uzaklık: {}".format(self.guncel_konum.x, self.hedefe_uzaklik))
        if self.hedefe_uzaklik <= 0.1:
            self.kontrol = False
        else:
            self.kontrol = True

# ROSInterruptException istisnası olmadığı müddet boyunca düğüm başlatılır ve örnek oluştur.
if __name__ == '__main__':
    try:
        rospy.init_node('hedefe_gitme')
        dugum = HedefeGit()
    except rospy.ROSInterruptException:
        pass
