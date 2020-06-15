#!/usr/bin/env python
# coding=utf-8
"""
Uygulama Adı      : Hedef Noktaya Varma Uygulaması
Uygulama Amacı    : Robotun odom konusundan konumunu, hedef_konum konusundan gideceği hedefi alıp, 
                    hedef nokta ile robot arasındaki mesafe farkını hesaplayarak, 
                    servis cevabına göre hız değeri üretmesi.
"""
# Gerekli kütüphanelerin eklenmesi
# hedefe_git.srv     : Oluşturulan servis dosyasını ekler.
# hedefe_git.msg     : Oluşturulan mesaj dosyasını ekler.
# geometry_msgs.msg  : Twist ve Point mesajlarını ekler.
# nav_msgs.msg       : Odometry mesajını ekler.
# rospy              : Python kütüphaneleri ile çalışmak için gereklidir.
# math               : Matematiksel işlemleri yapan kütüphanedir.
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from hedefe_git.srv import VarisDurum
from hedefe_git.msg import Mesafe
from math import sqrt

# HedefeGitme sınıfı oluşturuluyor ve tüm işlemler bu sınıfın özellikleri (attribute) ve yöntemleri (method) olarak tanımlanıyor.
class HedefeGitme():
    # Başlangıç özellik değerleri ayarlanıyor.
    def __init__(self):
        self.guncel_konum = Point()
        self.hedef_konum = Point()
        self.hedef_konum_x = 0.0
        self.hedef_konum.y = 0.0
        self.hedefe_uzaklik = 0.0
        self.kontrol = False
        self.ana_fonksiyon()
    # hedef_konum ve odom konularına abone olunuyor. cmd_vel konusunun yayımlanması için tanımlayıcı oluşturuluyor.
    def ana_fonksiyon(self):
        rospy.Subscriber('/hedef_konum', Mesafe, self.hedef_konum_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        hiz_yayinla = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        hiz_mesaj = Twist()
        # Döngü hızı ayarlanıyor.
        rate = rospy.Rate(10)
        # ROS kapanana kadar, döngü hızında bu işlemlerin yapılması sağlanıyor.
        while not rospy.is_shutdown():
            # Eğer servis dönüşü False ise, hedefe varılmadığı için, pozitif veya negatif hız mesajı yayınlanır.
            if not self.kontrol:
                if self.hedef_konum_x > self.guncel_konum.x:
                    hiz_mesaj.linear.x = 0.1
                    hiz_yayinla.publish(hiz_mesaj)
                else:
                    hiz_mesaj.linear.x = -0.1
                    hiz_yayinla.publish(hiz_mesaj)
            # Eğer servis dönüşü True ise, hedefe varıldığı için, robot hızını 0.0 olarak günceller.
            else:
                hiz_mesaj.linear.x = 0.0
                hiz_yayinla.publish(hiz_mesaj)
            rate.sleep()
    # ServiceException istisnası olmadığı müddetçe, istemcinin istekleri iletiliyor.
    def istekte_bulun(self, deger):
        rospy.wait_for_service('varis')
        try:
            servisim = rospy.ServiceProxy('varis', VarisDurum)
            cevap = servisim(deger)
            self.kontrol = cevap.durum
            return self.kontrol
        except rospy.ServiceException:
            pass
    # Yayınlanan odom konusunda robotun konum değerleri ve hedefe uzaklığı güncelleniyor. Değerler ekrana bastırılıyor.
    def odom_callback(self, deger):
        self.guncel_konum.x = deger.pose.pose.position.x
        self.guncel_konum.y = deger.pose.pose.position.y
        # Hedefe uzaklık hesaplanıyor. Harekette kaymalar olabileceği için y-ekseni de hesaba katılmıştır.
        self.hedefe_uzaklik = sqrt(pow(self.guncel_konum.y - self.hedef_konum.y, 2) + pow(self.guncel_konum.x - self.hedef_konum_x, 2))
        # Değerler ekrana bastırılıyor.
        print("Robotun güncel x konumu: {} m\nHedefe uzaklık: {}".format(self.guncel_konum.x, self.hedefe_uzaklik))
        # Servisten istekte bulunuluyor.
        self.istekte_bulun(self.hedefe_uzaklik)
    # Yayınlanan hedef_konum konusunda alınan parametrenin x değeri gidilecek noktayı belirtir.
    def hedef_konum_callback(self, deger):
        self.hedef_konum_x = deger.x

# ROSInterruptException istisnası olmadığı müddet boyunca düğümü başlat ve örnek oluştur.
if __name__ == '__main__':
    try:
        rospy.init_node('hedefe_gitme')
        dugum = HedefeGitme()
    except rospy.ROSInterruptException:
        pass