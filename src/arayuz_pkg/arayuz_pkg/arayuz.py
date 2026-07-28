import sys
from PySide6.QtCore import QLocale, QSize
from PySide6.QtGui import QDoubleValidator
from PySide6.QtWidgets import (
    QApplication,
    QFormLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QStyle,
    QVBoxLayout,
    QWidget,
)

# Ana arayüz penceremizi oluşturacak sınıf
class DroneArayuzu(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # 1. Pencere Ayarları
        self.setWindowTitle('YKI Drone Kontrol Arayüzü')
        self.setGeometry(300, 300, 420, 380)

        # 2. Modern Tasarım için QSS (Qt Style Sheets)
        # ---------------------------------------------------------------------
        self.setStyleSheet("""
            /* Bütün pencereye uygulanacak stil */
            QWidget {
                background-color: #2c3e50; /* Koyu Mavi-Gri Arka Plan */
            }

            QLabel {
                color: white;
                font-family: 'Arial';
                font-size: 13px;
            }

            QLineEdit {
                color: white;
                background-color: #1f2d3a;
                border: 1px solid #5dade2;
                border-radius: 6px;
                padding: 10px;
                font-family: 'Arial';
                font-size: 13px;
            }

            /* Bütün butonlara uygulanacak stil */
            QPushButton {
                color: white; /* Yazı rengi */
                background-color: #3498db; /* Canlı Mavi */
                border: none; /* Kenarlık yok */
                padding: 15px; /* Buton içi boşluk */
                font-family: 'Arial';
                font-size: 14px;
                font-weight: bold;
                border-radius: 8px; /* Yuvarlak köşeler */
            }

            /* Fare butonun üzerine geldiğinde uygulanacak stil */
            QPushButton:hover {
                background-color: #5dade2; /* Biraz daha açık mavi */
            }

            /* Butona basıldığında uygulanacak stil */
            QPushButton:pressed {
                background-color: #217dbb; /* Biraz daha koyu mavi */
            }

            QPushButton#emergencyButton {
                background-color: #c0392b;
            }

            QPushButton#emergencyButton:hover {
                background-color: #e74c3c;
            }

            QPushButton#emergencyButton:pressed {
                background-color: #922b21;
            }
        """)

        # 3. Butonları Oluşturma ve İkon Ekleme
        # ---------------------------------------------------------------------
        icon_yukari_ok = self.style().standardIcon(QStyle.SP_ArrowUp)
        coordinate_locale = QLocale.c()
        coordinate_locale.setNumberOptions(QLocale.NumberOption.RejectGroupSeparator)

        latitude_validator = QDoubleValidator(self)
        latitude_validator.setLocale(coordinate_locale)
        latitude_validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        latitude_validator.setDecimals(99)

        longitude_validator = QDoubleValidator(self)
        longitude_validator.setLocale(coordinate_locale)
        longitude_validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        longitude_validator.setDecimals(99)

        altitude_validator = QDoubleValidator(self)
        altitude_validator.setLocale(coordinate_locale)
        altitude_validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        altitude_validator.setBottom(0.1)
        altitude_validator.setDecimals(2)

        distance_validator = QDoubleValidator(self)
        distance_validator.setLocale(coordinate_locale)
        distance_validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        distance_validator.setBottom(0.1)
        distance_validator.setDecimals(2)

        self.kalkis_buton = QPushButton(' Kalkış Yap', self)
        self.kalkis_buton.setIcon(icon_yukari_ok)
        self.kalkis_buton.setIconSize(QSize(24, 24))

        self.acil_inis_buton = QPushButton(' Acil İniş', self)
        self.acil_inis_buton.setObjectName("emergencyButton")

        self.latitude_input = QLineEdit(self)
        self.latitude_input.setPlaceholderText("37.412294366500")
        self.latitude_input.setValidator(latitude_validator)

        self.longitude_input = QLineEdit(self)
        self.longitude_input.setPlaceholderText("-121.998570782860")
        self.longitude_input.setValidator(longitude_validator)

        self.takeoff_altitude_input = QLineEdit(self)
        self.takeoff_altitude_input.setPlaceholderText("10")
        self.takeoff_altitude_input.setValidator(altitude_validator)

        self.follow_distance_input = QLineEdit(self)
        self.follow_distance_input.setPlaceholderText("5")
        self.follow_distance_input.setValidator(distance_validator)

        # 5. Yerleşim (Layout) Ayarları
        # ---------------------------------------------------------------------
        v_box = QVBoxLayout()
        v_box.setSpacing(15)
        v_box.setContentsMargins(20, 20, 20, 20)

        form_layout = QFormLayout()
        form_layout.setSpacing(12)
        form_layout.addRow(QLabel("İlk Hedef Latitude"), self.latitude_input)
        form_layout.addRow(QLabel("İlk Hedef Longitude"), self.longitude_input)
        form_layout.addRow(QLabel("Takeoff Altitude (m)"), self.takeoff_altitude_input)
        form_layout.addRow(
            QLabel("Drone'lar Arası Mesafe (m)"),
            self.follow_distance_input,
        )

        v_box.addLayout(form_layout)
        v_box.addWidget(self.kalkis_buton)
        v_box.addWidget(self.acil_inis_buton)

        self.setLayout(v_box)
        self.show()



# Uygulamayı başlatmak için ana blok
if __name__ == '__main__':
    app = QApplication(sys.argv)
    arayuz = DroneArayuzu()
    sys.exit(app.exec_())
