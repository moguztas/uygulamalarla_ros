#!/usr/bin/env python
# coding=utf-8
"""
Uygulama Adı       : Engel Algılama ve Engelin Fotoğrafını Çekme Uygulaması
Uygulama Amacı     : scan konusuna abone olunarak mesafe verilerinden engel ile robot arasındaki 
                     mesafe kontrol edilecek, mesafe belirli bir değerin altına düşene kadar cmd_vel konusundan
                     hız değeri yayınlanacak ve mesafe belirli bir değerin altına düştüğünde, 
                     camera/rgb/image_raw konusuna abone olunarak, engelin fotoğrafı çekilecektir.
"""
# Gerekli kütüphanelerin eklenmesi
# rospy                     : Python kütüphaneleri ile çalışmak için gereklidir.
# sensor_msgs.msg           : LaserScan ve Image mesajlarını ekler.
# geometry_msgs.msg         : Twist mesajını ekler.
# cv2                       : OpenCV'nin Python kütüphanesidir.
# os                        : İşletim sistemiyle ilgili işler için kullanılır.
# cv_bridge:                : ROS Image mesajları ve OpenCV görüntüleri arasında dönüştürme yapmaya yarayan fonksiyonları içerir.
import rospy
import cv2
import os
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist

# EngelAlgilama sınıfı oluşturuluyor ve tüm işlemler bu sınıfın özellikleri (attribute) ve yöntemleri (method) olarak tanımlanıyor.
class EngelAlgilama:
    # Başlangıç özellik ve metotları ayarlanıyor.
    def __init__(self):
        self.bridge = CvBridge()
        # Çekilen fotoğrafların kaydedileceği dosya yolu gösteriliyor.
        self.dizin = '/home/robotik/catkin_ws/src/engel_foto/goruntuler/'  
        # Engelin resminin bir kere çekilmesini sağlayacak bayrak değişkeni başlangıçta False yapılıyor.
        self.bayrak = False  
        # engel_foto_cek düğümü başlatılıyor.
        rospy.init_node('engel_foto_cek', anonymous=True)
        # scan konusuna abone olunarak lazer sensör verileri alınıyor.
        self.lazer_abone = rospy.Subscriber('scan', LaserScan, self.lazer_callback) 
        # camera/rgb/image_raw konusuna abone olunarak renkli kamera verileri alınıyor.
        self.kamera_abone = rospy.Subscriber('camera/rgb/image_raw', Image, self.kamera_callback) 
        # cmd_vel konusu yayınlanarak, robotun hareketi için gereken hız verileri üretiliyor.
        self.hiz_yayinla = rospy.Publisher('cmd_vel', Twist, queue_size=10) 
        # hiz_mesaji Twist() tipinde verileri tutuyor.
        self.hiz_mesaji = Twist() 
        # Düğüm kapatılana kadar programın sürdürülmesini sağlar. 
        rospy.spin()
    # Renkli kamera görüntüsüne abone olunduğunda çağrılacak callback fonksiyonu
    def kamera_callback(self, msg):
        # Alınan mesaj verilerinin bridge.imgmsg_to_cv2 köprüsü yardımıyla bgr8 görüntü formatına dönüştürülmesi sağlanıyor.
        self.foto = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    # Çekilen fotoğrafın belirtilen dizinde gösterilmesini ve kaydedilmesini sağlayan fonksiyon
    def kaydet_ve_goster(self, img, foto_adi):
        # Terminale yazdırma
        print("Engel algılandı. Fotoğraf çekiliyor...") 
        # Çekilen fotoğrafın gösterilmesi
        cv2.imshow("Engel Fotoğrafı",img) 
        # Çekilen fotoğrafın gösterileceği süreyi milisaniye cinsinden belirtir. 0 program kapatılana kadar gösterilmesini sağlar.
        cv2.waitKey(0) 
        # Fotoğrafın __init__ fonksiyonunda bahsedilen dizine kaydedilmesini sağlar
        cv2.imwrite(os.path.join(self.dizin, foto_adi), img) 
        # Fotoğraf ekranının kapatılmasını sağlar
        cv2.destroyAllWindows() 
    # Lazer verisine abone olunduğunda çağrılacak callback fonksiyonu
    def lazer_callback(self, data):
        # Lazer verisinin robotun sol ve sağ ön taraflarına ait kısımları
        sol_on = list(data.ranges[0:9]) 
        sag_on = list(data.ranges[350:359])
        # sol ve sağ ön lazer verilerinin birleştirilmesi
        sol_sag_on = sol_on + sag_on 
        # Birleştirilen lazer verisindeki minimum mesafenin alınması
        minimum_mesafe = min(sol_sag_on)
        print("Engele olan mesafe: %f metre" %minimum_mesafe)
        # Çekilen fotoğrafın ismi ve uzantısı ayarlanıyor.
        self.foto_adi = "Engel Foto" +'.jpg'  
        # Robot ile engel arasındaki mesafe 1 metreden az ise, önce robot durduruluyor, ardından 
        # bayrak değeri kontrol ediliyor ve son olarak fotoğraf çekiliyor.
        if(minimum_mesafe < 1.0):
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_yayinla.publish(self.hiz_mesaji)
            if self.bayrak == False: 
                self.kaydet_ve_goster(self.foto, self.foto_adi) 
                # Tekrar if bloğuna girilmemesi için bayrak değişkeni True olarak değiştiriliyor.
                self.bayrak = True 
        # Robot ile engel arasındaki mesafe 1 metreden fazla ise bayrak değişkeni False olarak ayarlanıyor ve 
        # robota 0.1 m/s x yönünde hız verisi üretmesi komutu gönderiliyor.
        else:
            self.hiz_mesaji.linear.x = 0.1
            self.hiz_yayinla.publish(self.hiz_mesaji)
            # Bayrak değişkeninin tekrar engel algılamada fotoğraf çekmesi için False olarak değiştirilmesi gerekmektedir.
            self.bayrak = False 

# EngelAlgilama sınıfı robotun engel görene kadar X yönünde hareket etmesini ve engel gördüğünde durup fotoğrafını çekmesini sağlar.
# Engel robotun önünden çekilene kadar durmaya devam edecektir.  
# ROSInterruptException istisnası olmadığı müddet boyunca oluşturulan fonksiyonu test et.
if __name__ == '__main__':
    try:
        EngelAlgilama()
    except rospy.ROSInterruptException:
        pass