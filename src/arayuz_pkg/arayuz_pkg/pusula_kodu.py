from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPainter, QPixmap
from PyQt5.QtCore import Qt, QRect

from arayuz_pkg.resources import resource_path

class DronluPusula(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 1. ESNEKLİK AYARI: "Expanding" kullanıyoruz.
        # Bu sayede büyümeye hevesli olur ama minimum sınırına da saygı duyar.
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        
        # 2. KARINCA FRENİ (ÖNEMLİ KISIM)
        # 10 çok küçüktü, 200 çok büyüktü (önceki hatalarda ekranı kilitlemişti). 
        # Şimdi 120 (veya 100) vererek alt sınırı belirliyoruz.
        self.setMinimumSize(120, 120) 
        
        # RESİMLERİ YÜKLEME
        self.pusula_pixmap = QPixmap(str(resource_path("pusula_resmi.png"))) 
        self.dron_pixmap = QPixmap(str(resource_path("dron_resmi.png")))

        self.aci = 0 

    def sizeHint(self):
        # Program ilk açıldığında veya ekranda bolca yer varken 
        # 250x250 piksel gibi büyük ve güzel bir boyutta başlasın:
        return QtCore.QSize(250, 250)

    def paintEvent(self, event):
        # Eğer resimler yüklenemediyse boşuna çizim yapma
        if self.pusula_pixmap.isNull() or self.dron_pixmap.isNull():
            return

        painter = QPainter(self)
        
        # 1. Çizim araçlarının kalitesini en yükseğe çekiyoruz
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        painter.setRenderHint(QPainter.HighQualityAntialiasing, True) # Ekstra kalite

        width = self.width()
        height = self.height()
        size = min(width, height) 
        
        # 2. ASIL ÇÖZÜM: Resimleri yüksek kaliteyle (SmoothTransformation) o anki boyuta göre ölçekle!
        pusula_kaliteli = self.pusula_pixmap.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        dron_kaliteli = self.dron_pixmap.scaled(size, size, Qt.KeepAspectRatio, Qt.SmoothTransformation)

        # Hedef koordinatları belirle (Tam ortaya hizalamak için)
        hedef_x = (width - size) // 2
        hedef_y = (height - size) // 2

        # --- 1. ALT KATMAN: PUSULAYI ÇİZ ---
        painter.drawPixmap(hedef_x, hedef_y, pusula_kaliteli)

        # --- 2. ÜST KATMAN: DRONU ÇİZ VE DÖNDÜR ---
        painter.save() 
        
        # Dönüş merkezini yeni kaliteli resmin tam ortası olarak belirliyoruz
        merkez_x = hedef_x + (size / 2)
        merkez_y = hedef_y + (size / 2)
        
        painter.translate(merkez_x, merkez_y)
        painter.rotate(self.aci) 
        painter.translate(-merkez_x, -merkez_y)
        
        # Döndürülmüş kaliteli dron resmini çiz
        painter.drawPixmap(hedef_x, hedef_y, dron_kaliteli)
        
        painter.restore()
