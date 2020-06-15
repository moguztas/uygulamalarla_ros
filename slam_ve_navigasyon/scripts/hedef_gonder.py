#!/usr/bin/env python
# coding=utf-8
"""
Uygulama Adı                 : Hedef Noktaya Gitme
Uygulama Amacı               : map ve move_base konuları ile robotun belirtilen hedef noktaya gitmesini sağlama 
                               ve engellere çarpmadan otonom hareketini gerçekleştirme.
"""
# Gerekli kütüphanelerin eklenmesi
# actionlib                 : SimpleActionClient fonksiyonunu kullanmak için gereklidir.
# rospy                     : Python kütüphaneleri ile çalışmak için gereklidir.
# move_base_msgs.msg        : .action dosya mesajlarını kullanamak için gereklidir. 
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_istemci():
   # MoveBaseAction tanımlamasına sahip move_base action istemcisi oluşturuldu.
    istemci = actionlib.SimpleActionClient('move_base',MoveBaseAction)
   # Hedef nokta gelene kadar servis bekleniyor.
    istemci.wait_for_server()
   # Hedef noktası belirlemek için MoveBaseGoal mesajı kullanılıyor.
    hedef = MoveBaseGoal()
    hedef.target_pose.header.frame_id = "map"
   # map üstünde gidilecek x ve y konumları tanımlanıyor.
    hedef.target_pose.pose.position.x = 3.0
    hedef.target_pose.pose.position.y = 4.5
   # mobile robotun map çerçevesine göre dönüş yapmamasını sağlama 
    hedef.target_pose.pose.orientation.w = 1.0
   # Hedef aksiyon server'ına gönderiliyor.
    istemci.send_goal(hedef)
   # Aksiyonu gerçekleştirmek için servis bekleniyor.
    bekle = istemci.wait_for_result()
   # Eğer ulaşılamazsa, sinyal kesilir.
    if not bekle:
        rospy.signal_shutdown("Action Servisi mevcut değil!")
    # Ulaşılırsa, servisin sonucu döndürülür.
    else:
        return istemci.get_result()   
    
# move_base_hedef_gonder düğümü oluşturuluyor ve movebase_istemci fonksiyonunun sonucu True ise, hedef noktaya varıldığı
# Değilse, hedef noktaya varılamadığı bilgisi yazdırılıyor. 
if __name__ == '__main__':
    try:
        rospy.init_node('move_base_hedef_gonder')
        result = movebase_istemci()
        if result:
            rospy.loginfo("Hedef noktaya varıldı!")
        else:
            rospy.loginfo("Hedefe gidiliyor...")
    except rospy.ROSInterruptException:
        pass