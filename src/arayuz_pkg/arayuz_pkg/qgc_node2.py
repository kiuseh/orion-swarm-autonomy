import sys
import json
import html
import os
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from PyQt5 import QtWidgets, uic, QtCore, QtWebEngineWidgets, QtGui
from PyQt5.QtCore import Qt

from arayuz_pkg.resources import resource_path

os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"
os.environ["QTWEBENGINE_CHROMIUM_FLAGS"] = "--disable-gpu --no-sandbox"

ARAYUZ_TOPIC = "arayuz_topic"
EMERGENCY_LAND_TOPIC = "emergency_land_topic"
SWARM_INFO_TOPIC = "swarm_info_topic"
ROSOUT_TOPIC = "/rosout"
UAV_LOGGER_NAME = "drone_controller_node"
ROLE_LOG_COLORS = {
    "lider": "#FFD700",
    "sag": "#00FF66",
    "sol": "#FF4444",
}

# ---------------------------------------------------------
# ARTI / EKSİ ÖZEL BUTONLU DEĞER SEÇİCİ (CUSTOM SPINBOX)
# ---------------------------------------------------------
class PlusMinusSpinBox(QtWidgets.QWidget):
    def __init__(self, val, min_val=-10, max_val=10, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        
        self.btn_minus = QtWidgets.QPushButton("-")
        self.btn_minus.setFixedSize(30, 30)
        self.btn_minus.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 68, 68, 15);
                color: #ff4444;
                border: 1px solid rgba(255, 68, 68, 80);
                border-radius: 4px;
                font-size: 16px;
                font-weight: bold;
                padding: 0px;
            }
            QPushButton:hover { background-color: rgba(255, 68, 68, 40); }
        """)
        
        self.lbl_value = QtWidgets.QLabel(str(val))
        self.lbl_value.setFixedSize(45, 30)
        self.lbl_value.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_value.setStyleSheet("""
            QLabel {
                background-color: rgba(0, 20, 30, 200);
                color: #00ffff;
                border: 1px solid rgba(0, 229, 255, 100);
                border-radius: 4px;
                font-size: 14px;
                font-weight: bold;
            }
        """)
        
        self.btn_plus = QtWidgets.QPushButton("+")
        self.btn_plus.setFixedSize(30, 30)
        self.btn_plus.setStyleSheet("""
            QPushButton {
                background-color: rgba(0, 255, 102, 15);
                color: #00ff66;
                border: 1px solid rgba(0, 255, 102, 80);
                border-radius: 4px;
                font-size: 16px;
                font-weight: bold;
                padding: 0px;
            }
            QPushButton:hover { background-color: rgba(0, 255, 102, 40); }
        """)
        
        layout.addWidget(self.btn_minus)
        layout.addWidget(self.lbl_value)
        layout.addWidget(self.btn_plus)
        
        self.min_val = min_val
        self.max_val = max_val
        self.current_value = val
        
        self.btn_minus.clicked.connect(self.decrement)
        self.btn_plus.clicked.connect(self.increment)
        
    def decrement(self):
        if self.current_value > self.min_val:
            self.current_value -= 1
            self.lbl_value.setText(str(self.current_value))
            
    def increment(self):
        if self.current_value < self.max_val:
            self.current_value += 1
            self.lbl_value.setText(str(self.current_value))
            
    def value(self):
        return self.current_value


# ---------------------------------------------------------
# DİNAMİK SÜRÜ AYARLARI PENCERESİ (EKLEME / SİLME DESTEKLİ)
# ---------------------------------------------------------
class RolAtamaDialog(QtWidgets.QDialog):
    def __init__(self, mevcut_roller, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sürü Rol ve Slot Ataması")
        self.setMinimumSize(380, 350)
        
        self.setStyleSheet("""
            QDialog { background-color: #0c111a; border: 2px solid rgba(0, 229, 255, 70); border-radius: 10px; }
            QLabel { color: white; font-weight: bold; font-size: 12px; }
            QLineEdit { background-color: rgba(0, 20, 30, 200); border: 1px solid rgba(0, 229, 255, 80); border-radius: 4px; color: #00ffff; padding: 4px; font-weight: bold; }
            QPushButton { background-color: rgba(0, 123, 255, 180); color: white; border-radius: 6px; padding: 6px; font-weight: bold; }
            QPushButton:hover { background-color: rgba(0, 86, 179, 255); }
        """)

        self.main_layout = QtWidgets.QVBoxLayout(self)
        
        info = QtWidgets.QLabel("Dron Listesini Yönetin ve Slot Atayın:\n(0: Lider | +: Sağ Takipçi | -: Sol Takipçi)")
        info.setStyleSheet("color: #00e5ff; font-size: 13px; margin-bottom: 10px;")
        info.setAlignment(QtCore.Qt.AlignCenter)
        self.main_layout.addWidget(info)
        
        ekle_layout = QtWidgets.QHBoxLayout()
        self.txt_yeni_id = QtWidgets.QLineEdit()
        self.txt_yeni_id.setPlaceholderText("Yeni Drone ID (Örn: 14544)")
        btn_ekle = QtWidgets.QPushButton("Sürüye Ekle")
        btn_ekle.setStyleSheet("background-color: rgba(0, 200, 81, 150);")
        btn_ekle.clicked.connect(self.dron_ekle)
        ekle_layout.addWidget(self.txt_yeni_id)
        ekle_layout.addWidget(btn_ekle)
        self.main_layout.addLayout(ekle_layout)

        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setStyleSheet("background: transparent; border: none;")
        self.scroll_widget = QtWidgets.QWidget()
        self.scroll_widget.setObjectName("scroll_widget")
        self.scroll_layout = QtWidgets.QVBoxLayout(self.scroll_widget)
        self.scroll.setWidget(self.scroll_widget)
        self.main_layout.addWidget(self.scroll)
        
        self.spin_boxes = {}
        self.satirlar = {}
        self.current_roles = mevcut_roller.copy()
        
        for d_id, slot in self.current_roles.items():
            self.listeye_satir_ekle(d_id, slot)
            
        self.btn_kaydet = QtWidgets.QPushButton("DEĞİŞİKLİKLERİ UYGULA")
        self.btn_kaydet.setStyleSheet("padding: 10px; font-size: 13px; background-color: #007bff; margin-top: 10px;")
        self.btn_kaydet.clicked.connect(self.accept)
        self.main_layout.addWidget(self.btn_kaydet)

    def listeye_satir_ekle(self, d_id, slot):
        h_layout = QtWidgets.QHBoxLayout()
        lbl = QtWidgets.QLabel(f"DRON ID: {d_id}")
        sb = PlusMinusSpinBox(slot)
        
        btn_sil = QtWidgets.QPushButton("Kaldır")
        btn_sil.setFixedSize(55, 30)
        btn_sil.setStyleSheet("background-color: rgba(255, 53, 71, 150); font-size: 11px;")
        btn_sil.clicked.connect(lambda: self.dron_kaldir(d_id))
        
        h_layout.addWidget(lbl)
        h_layout.addWidget(sb)
        h_layout.addWidget(btn_sil)
        
        container = QtWidgets.QWidget()
        container.setLayout(h_layout)
        self.scroll_layout.addWidget(container)
        
        self.spin_boxes[d_id] = sb
        self.satirlar[d_id] = container

    def dron_ekle(self):
        try:
            d_id = int(self.txt_yeni_id.text().strip())
            if d_id in self.spin_boxes: return
            self.listeye_satir_ekle(d_id, 0)
            self.txt_yeni_id.clear()
        except ValueError: pass

    def dron_kaldir(self, d_id):
        if d_id in self.satirlar:
            self.scroll_layout.removeWidget(self.satirlar[d_id])
            self.satirlar[d_id].deleteLater()
            del self.satirlar[d_id]
            del self.spin_boxes[d_id]

    def rolleri_al(self):
        return {d_id: sb.value() for d_id, sb in self.spin_boxes.items()}


# ---------------------------------------------------------
# ANA ARAYÜZ (YER KONTROL İSTASYONU)
# ---------------------------------------------------------
class YerKontrolIstasyonu(Node, QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__('qgc_node')
        QtWidgets.QMainWindow.__init__(self)
        
        self.setWindowTitle("Dinamik Sürü İHA Yer Kontrol İstasyonu")
        self.setMinimumSize(1024, 768)

        self.scroll_area_sol = QtWidgets.QScrollArea(self)
        self.scroll_area_sol.setWidgetResizable(True)
        self.scroll_area_sol.setObjectName("sol_scroll")
        self.scroll_area_sol.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.scroll_area_sol.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.scroll_area_sol.viewport().setAutoFillBackground(False)
        self.scroll_area_sol.viewport().setAttribute(QtCore.Qt.WA_TranslucentBackground)

        self.sol_frame = QtWidgets.QFrame()
        self.sol_frame.setObjectName("sol_frame")
        self.sol_frame.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.sol_layout = QtWidgets.QVBoxLayout(self.sol_frame)
        self.sol_layout.setContentsMargins(8, 8, 8, 8)
        self.sol_layout.setSpacing(22)
        self.sol_layout.setAlignment(QtCore.Qt.AlignTop)
        self.scroll_area_sol.setWidget(self.sol_frame)

        self.sag_frame = QtWidgets.QFrame(self)
        self.sag_frame.setObjectName("sag_frame")
        self.sag_layout = QtWidgets.QVBoxLayout(self.sag_frame)
        self.sag_layout.setContentsMargins(12, 12, 12, 12)
        self.sag_layout.setSpacing(8)

        grid_layout = QtWidgets.QGridLayout()
        grid_layout.setSpacing(6)

        lbl_irtifa = QtWidgets.QLabel("Kalkış Yüksekliği (m):")
        self.kalkis_yuksekligi = QtWidgets.QLineEdit("7.5")
        lbl_enlem = QtWidgets.QLabel("Hedef Enlem:")
        self.enlem = QtWidgets.QLineEdit("37.412175143823063")
        lbl_boylam = QtWidgets.QLabel("Hedef Boylam:")
        self.boylam = QtWidgets.QLineEdit("-121.998676647076721")
        lbl_mesafe = QtWidgets.QLabel("Takip Mesafesi (m):")
        self.takip_mesafesi = QtWidgets.QLineEdit("5")

        grid_layout.addWidget(lbl_irtifa, 0, 0)
        grid_layout.addWidget(self.kalkis_yuksekligi, 0, 1)
        grid_layout.addWidget(lbl_mesafe, 0, 2)
        grid_layout.addWidget(self.takip_mesafesi, 0, 3)
        grid_layout.addWidget(lbl_enlem, 1, 0)
        grid_layout.addWidget(self.enlem, 1, 1)
        grid_layout.addWidget(lbl_boylam, 1, 2)
        grid_layout.addWidget(self.boylam, 1, 3)
        self.sag_layout.addLayout(grid_layout)

        buton_layout = QtWidgets.QHBoxLayout()
        self.pushButton = QtWidgets.QPushButton("KALKIŞ BAŞLAT")
        self.pushButton.clicked.connect(self.kalkis_baslat)
        
        self.pushButton_3 = QtWidgets.QPushButton("BEKLEME YAP")
        self.pushButton_3.setEnabled(False)
        self.pushButton_3.setToolTip("Gerçek görev akışında HOLD komutu desteklenmiyor.")
        self.pushButton_3.clicked.connect(self.bekleme_yap)

        self.pushButton_2 = QtWidgets.QPushButton("ACİL İNİŞ")
        self.pushButton_2.clicked.connect(self.acil_inis_yap)

        buton_layout.addWidget(self.pushButton)
        buton_layout.addWidget(self.pushButton_3)
        buton_layout.addWidget(self.pushButton_2)
        self.sag_layout.addLayout(buton_layout)

        self.btn_rol_atama = QtWidgets.QPushButton("SÜRÜ ROL AYARLARI (N DRON)")
        self.btn_rol_atama.clicked.connect(self.rol_penceresini_ac)
        self.sag_layout.addWidget(self.btn_rol_atama)

        self.dron_rolleri = {14541: -1, 14542: 0, 14543: 1} 
        self.role_map = {}
        self.gorev_basladi = False
        
        self.paneller = {}
        self.son_telemetri = {}
        self.onceki_durumlar = {}  

        self.browser = QtWebEngineWidgets.QWebEngineView(self)
        self.haritayi_hazirla()
        self.browser.lower() 
        
        self.log_ekrani = QtWidgets.QTextEdit(self)
        self.log_ekrani.setReadOnly(True)
        self.log_ekrani.setObjectName("log_ekrani")
        self.log_ekle("Gerçek sürü telemetrisi bekleniyor.")

        self.rol_haritasini_guncelle()
        self.panelleri_kur()
        self.basliklari_guncelle()
        
        self.scroll_area_sol.raise_()
        self.sag_frame.raise_()
        self.log_ekrani.raise_()

        self.subscription = self.create_subscription(
            String,
            SWARM_INFO_TOPIC,
            self.telemetri_callback,
            10,
        )
        self.rosout_subscription = self.create_subscription(
            Log,
            ROSOUT_TOPIC,
            self.rosout_callback,
            50,
        )
        self.command_publisher = self.create_publisher(String, ARAYUZ_TOPIC, 10)
        self.emergency_land_publisher = self.create_publisher(
            String,
            EMERGENCY_LAND_TOPIC,
            10,
        )

        self.ros_timer = QtCore.QTimer()
        self.ros_timer.timeout.connect(self.ros_spin)
        self.ros_timer.start(20)
        
        self.harita_timer = QtCore.QTimer()
        self.harita_timer.timeout.connect(self.haritayi_js_ile_guncelle)
        self.harita_timer.start(100)
        self.showMaximized()

    def rol_penceresini_ac(self):
        if self.gorev_basladi:
            self.log_ekle("Görev başladıktan sonra rol değişikliği gönderilmiyor.", role="sol")
            return

        dialog = RolAtamaDialog(self.dron_rolleri, self)
        dialog.adjustSize()
        dialog.move(self.frameGeometry().center() - dialog.rect().center())
        if dialog.exec_():
            yeni_roller = dialog.rolleri_al()
            gecerli, hata = self.rolleri_dogrula(yeni_roller)
            if not gecerli:
                self.log_ekle(f"Rol ayarı geçersiz: {hata}", role="sol")
                return

            self.dron_rolleri = yeni_roller
            self.rol_haritasini_guncelle()
            self.panelleri_kur() 
            self.basliklari_guncelle()
            self.rol_konfigurasyonu_yayinla()
            self.log_ekle(
                f"Sürü yapısı güncellendi ve {len(self.dron_rolleri)} İHA'ya rol ayarı gönderildi."
            )

    def rol_konfigurasyonu_yayinla(self):
        msg = String()
        msg.data = json.dumps({
            "command": "configure_roles",
            "roles": self.dron_rolleri,
        })
        self.command_publisher.publish(msg)

    def rol_haritasini_guncelle(self):
        self.role_map.clear()
        for d_id, slot in self.dron_rolleri.items():
            str_id = str(d_id)
            self.role_map[str_id] = d_id
            self.role_map[self.slot_to_role_key(slot)] = d_id

    def slot_to_role_key(self, slot):
        slot = int(slot)
        if slot == 0:
            return "lider"
        if slot < 0:
            return f"sol_{abs(slot)}"
        return f"sag_{slot}"

    def role_key_to_slot(self, role_key):
        try:
            if role_key == "lider":
                return 0
            if role_key.startswith("sol_"):
                return -int(role_key.split("_", 1)[1])
            if role_key.startswith("sag_"):
                return int(role_key.split("_", 1)[1])
        except (IndexError, ValueError):
            return None
        return None

    def rolleri_dogrula(self, roller):
        if not roller:
            return False, "en az bir İHA tanımlanmalı"

        try:
            normalized = {int(d_id): int(slot) for d_id, slot in roller.items()}
        except (TypeError, ValueError):
            return False, "port ve slot değerleri tam sayı olmalı"

        leader_count = sum(1 for slot in normalized.values() if slot == 0)
        if leader_count != 1:
            return False, "tam olarak bir lider slotu (0) olmalı"

        slots = list(normalized.values())
        if len(slots) != len(set(slots)):
            return False, "slot değerleri benzersiz olmalı"

        for udp_port in normalized:
            if udp_port <= 0:
                return False, "UDP portları pozitif olmalı"

        return True, ""

    def basliklari_guncelle(self):
        for d_id, slot in self.dron_rolleri.items():
            if slot == 0:     grup_adi = f"LİDER DRON (ID: {d_id})"
            elif slot < 0:   grup_adi = f"SOL TAKİPÇİ [{abs(slot)}] (ID: {d_id})"
            elif slot > 0:   grup_adi = f"SAĞ TAKİPÇİ [{slot}] (ID: {d_id})"
            
            if d_id in self.paneller:
                gb = self.paneller[d_id].findChild(QtWidgets.QGroupBox)
                if gb: gb.setTitle(grup_adi)

    def panelleri_kur(self):
        while self.sol_layout.count():
            child = self.sol_layout.takeAt(0)
            if child.widget(): child.widget().deleteLater()
        self.paneller.clear()

        for d_id in self.dron_rolleri.keys():
            panel_widget = QtWidgets.QWidget()
            panel_widget.setAttribute(QtCore.Qt.WA_TranslucentBackground)
            try:
                uic.loadUi(str(resource_path('drone_panel.ui')), panel_widget)
                lcd_hiz = panel_widget.findChild(QtWidgets.QLCDNumber, "lcd_hiz")
                lcd_irtifa = panel_widget.findChild(QtWidgets.QLCDNumber, "lcd_irtifa")
                if lcd_hiz: lcd_hiz.setSegmentStyle(QtWidgets.QLCDNumber.Flat)
                if lcd_irtifa: lcd_irtifa.setSegmentStyle(QtWidgets.QLCDNumber.Flat)

                gb = panel_widget.findChild(QtWidgets.QGroupBox, "groupBox_4")
                if gb:
                    shadow = QtWidgets.QGraphicsDropShadowEffect(gb)
                    shadow.setBlurRadius(18)
                    shadow.setOffset(0, 4)
                    shadow.setColor(QtGui.QColor(0, 0, 0, 120))
                    gb.setGraphicsEffect(shadow)

                if panel_widget.layout(): panel_widget.layout().setContentsMargins(5, 5, 5, 5)
                
                self.sol_layout.addWidget(panel_widget)
                self.paneller[d_id] = panel_widget 
            except Exception as e:
                print(f"Hata: 'drone_panel.ui' yüklenemedi: {e}")

    def resizeEvent(self, event):
        w = self.width(); h = self.height()
        if self.browser: self.browser.setGeometry(0, 0, w, h)
        
        sol_w = 450  
        if hasattr(self, 'scroll_area_sol'): self.scroll_area_sol.setGeometry(15, 15, sol_w, h - 30)
        
        sag_w = 460; ortak_x = w - sag_w - 15; sag_h = 210; sag_y = h - sag_h - 15 
        if hasattr(self, 'sag_frame'): self.sag_frame.setGeometry(ortak_x, sag_y, sag_w, sag_h)
        if hasattr(self, 'log_ekrani'): self.log_ekrani.setGeometry(ortak_x, 15, sag_w, sag_y - 30)
        super().resizeEvent(event)

    def log_ekle(self, mesaj, role=None, renk_override=None, etiket_override=None):
        zaman = QtCore.QTime.currentTime().toString("HH:mm:ss")
        renk = renk_override or ROLE_LOG_COLORS.get(role, "#FFFFFF")
        etiket = etiket_override if etiket_override is not None else (f"[{role.upper()}]:" if role else "[SİSTEM]:")
        safe_etiket = html.escape(str(etiket), quote=False)
        safe_mesaj = html.escape(str(mesaj), quote=False).replace("\n", "<br>")
        html_mesaj = f"<div style='margin-bottom: 4px; font-family: monospace; font-size: 13px;'><span style='color: #888;'>[{zaman}]</span> <b style='color: {renk};'>{safe_etiket}</b> <span style='color: {renk};'>{safe_mesaj}</span></div>"
        self.log_ekrani.append(html_mesaj)
        self.log_ekrani.verticalScrollBar().setValue(self.log_ekrani.verticalScrollBar().maximum())

    def ros_log_level_name(self, level):
        return {
            10: "DEBUG",
            20: "INFO",
            30: "WARN",
            40: "ERROR",
            50: "FATAL",
        }.get(int(level), "LOG")

    def ros_log_level_color(self, level):
        return {
            10: "#9AA0A6",
            20: "#FFFFFF",
            30: "#FFFF00",
            40: "#FF4444",
            50: "#FF0055",
        }.get(int(level), "#FFFFFF")

    def logger_port_from_name(self, logger_name):
        prefix = f"{UAV_LOGGER_NAME}."
        if not logger_name.startswith(prefix):
            return None

        port_text = logger_name[len(prefix):].split(".", 1)[0]
        try:
            return int(port_text)
        except ValueError:
            return None

    def log_role_for_port(self, port):
        if port is None:
            return None

        slot = self.dron_rolleri.get(int(port))
        if slot is None:
            return None
        slot = int(slot)
        if slot == 0:
            return "lider"
        if slot > 0:
            return "sag"
        return "sol"

    def log_label_for_port(self, port, role, level_name):
        if role == "lider":
            role_label = "LİDER"
        elif role == "sag":
            role_label = "SAĞ"
        elif role == "sol":
            role_label = "SOL"
        else:
            role_label = "UAV"

        if port is None:
            return f"[{role_label} {level_name}]:"
        return f"[{role_label} {port} {level_name}]:"

    def rosout_callback(self, msg):
        logger_name = str(getattr(msg, "name", ""))
        if UAV_LOGGER_NAME not in logger_name:
            return

        mesaj = str(getattr(msg, "msg", "")).strip()
        if not mesaj:
            return

        level = int(getattr(msg, "level", 0) or 0)
        port = self.logger_port_from_name(logger_name)
        role = self.log_role_for_port(port)
        level_name = self.ros_log_level_name(level)
        renk = ROLE_LOG_COLORS.get(role, self.ros_log_level_color(level))
        self.log_ekle(
            mesaj,
            renk_override=renk,
            etiket_override=self.log_label_for_port(port, role, level_name),
        )

    def durum_text_uret(self, data):
        state = data.get("state")
        if state:
            return str(state)
        if data.get("all_mission_done"):
            return "GÖREV TAMAM"
        if data.get("shared_target_reached"):
            return "HEDEFTE"
        if data.get("takeoff_done"):
            return "UÇUŞTA"
        return "BİLİNMİYOR"

    def durum_rengi_uret(self, data):
        if data.get("all_mission_done"):
            return "mavi"
        if data.get("state") in ("MOVE", "ROTATION", "INITIAL_POSITIONING"):
            return "yesil"
        if data.get("state") in ("TAKE_OFF", "RETURN_TO_HOME"):
            return "sari"
        if data.get("swarm_departure_active"):
            return "sari"
        return "gri"

    def swarm_info_to_ui_packet(self, data):
        role_key = data.get("sender_role")
        slot = data.get("slot_index")
        if slot is None and role_key:
            slot = self.role_key_to_slot(str(role_key))

        d_id = data.get("sender_udp_port")
        if d_id is None and role_key:
            d_id = self.role_map.get(str(role_key))
        if d_id is None and slot is not None:
            for port, port_slot in self.dron_rolleri.items():
                if int(port_slot) == int(slot):
                    d_id = port
                    break
        if d_id is None:
            return None, None

        try:
            d_id = int(d_id)
        except (TypeError, ValueError):
            return None, None

        if slot is None:
            slot = self.dron_rolleri.get(d_id)
        if slot is None:
            return None, None

        latitude = data.get("latitude")
        longitude = data.get("longitude")
        display_altitude = data.get("relative_altitude")
        if display_altitude is None:
            display_altitude = data.get("absolute_altitude")

        return d_id, {
            "latitude": latitude,
            "longitude": longitude,
            "absolute_altitude": display_altitude,
            "speed": data.get("speed_m_s"),
            "heading": data.get("heading"),
            "durum_text": self.durum_text_uret(data),
            "renk": self.durum_rengi_uret(data),
            "battery": data.get("battery_percent"),
            "slot": int(slot),
        }

    def telemetri_callback(self, msg):
        try:
            data = json.loads(msg.data)
            d_id, drone_data = self.swarm_info_to_ui_packet(data)
            if d_id is None:
                return

            self.son_telemetri[str(d_id)] = drone_data
            if d_id not in self.paneller:
                return

            p = self.paneller[d_id]
            mevcut_durum = drone_data.get("durum_text", "BİLİNMİYOR")
            key = str(d_id)

            if self.onceki_durumlar.get(key) != mevcut_durum:
                slot = self.dron_rolleri.get(d_id, drone_data.get("slot", 0))
                aktif_rol = "lider" if slot == 0 else ("sag" if slot > 0 else "sol")
                self.log_ekle(f"Port {d_id} Durum: {mevcut_durum}", role=aktif_rol)
                self.onceki_durumlar[key] = mevcut_durum

            hiz = drone_data.get("speed")
            if hiz is not None:
                lcd_hiz = p.findChild(QtWidgets.QLCDNumber, "lcd_hiz")
                if lcd_hiz:
                    lcd_hiz.display(round(float(hiz), 2))

            irtifa = drone_data.get("absolute_altitude")
            if irtifa is not None:
                lcd_irtifa = p.findChild(QtWidgets.QLCDNumber, "lcd_irtifa")
                if lcd_irtifa:
                    lcd_irtifa.display(round(float(irtifa), 2))

            pb = p.findChild(QtWidgets.QProgressBar)
            batarya = drone_data.get("battery")
            if pb and batarya is not None:
                batarya = int(max(0, min(100, float(batarya))))
                pb.setValue(batarya)
                b_renk = "#FF4444" if batarya <= 20 else "#007BFF"
                pb.setStyleSheet(f"QProgressBar::chunk {{ background-color: {b_renk}; width: 8px; margin: 1px; border-radius: 1px; }}")

            heading = drone_data.get('heading')
            if heading is not None:
                heading = float(heading)
                lbl_aci = p.findChild(QtWidgets.QLabel, "label_63")
                if lbl_aci: lbl_aci.setText(f"{int(heading)}°")

                pusula_nesnesi = p.findChild(QtWidgets.QWidget, "widget_8")
                if pusula_nesnesi and hasattr(pusula_nesnesi, 'aci'):
                    pusula_nesnesi.aci = heading
                    pusula_nesnesi.update()

            bg_color = {"yesil": "#00FF00", "kirmizi": "#FF0000", "sari": "#FFFF00", "mavi": "#3399FF"}.get(drone_data.get("renk", "gri"), "#555555")
            lbl_durum = p.findChild(QtWidgets.QLabel, "textLabel") or p.findChild(QtWidgets.QLabel, "tlabel2")
            if lbl_durum: lbl_durum.setText(mevcut_durum)

            kutu = p.findChild(QtWidgets.QWidget, "durum_kutusu")
            if kutu: kutu.setStyleSheet(f"background-color: {bg_color}; border-radius: 4px;")
        except Exception as e:
            self.log_ekle(f"Telemetri okunamadı: {e}", role="sol")

    def haritayi_hazirla(self):
        html = """
        <!DOCTYPE html>
        <html><head><meta charset="utf-8" />
        <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
        <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
        <style>
            #map { height: 100vh; width: 100vw; margin: 0; background: #0b0d0f; }
            body { margin: 0; padding: 0; overflow: hidden; background: transparent; }
            .drone-label { background: rgba(0, 0, 0, 0.8); color: #fff; border: 1px solid #777; border-radius: 4px; padding: 2px 6px; font-size: 11px; font-weight: bold; }
            .custom-icon { background: transparent; border: none; }
        </style>
        </head><body><div id="map"></div>
        <script>
            var map = L.map('map', { center: [37.412175143823063, -121.998676647076721], zoom: 19, zoomControl: false });
            L.tileLayer('https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}', { maxZoom: 24 }).addTo(map);
            var markers = {};
            var pathLines = {};
            var pathPoints = {};
            var maxPathPoints = 5000;
            var minPathPointDistanceMeters = 0.2;
            var userDragged = false;
            map.on('dragstart', function() { userDragged = true; });
            map.on('dblclick', function() { userDragged = false; });
            
            function createDroneIcon(heading, color) {
                var svg = `<svg viewBox="0 0 100 100" style="width: 40px; height: 40px; transform: rotate(${heading}deg); transform-origin: center; filter: drop-shadow(0px 4px 5px rgba(0,0,0,0.8));">
                              <path d="M50 10 L85 80 L50 65 L15 80 Z" fill="${color}" stroke="#ffffff" stroke-width="3"/></svg>`;
	                return L.divIcon({ className: 'custom-icon', html: svg, iconSize: [40, 40], iconAnchor: [20, 20] });
	            }

                function getPoint(d) {
                    var lat = Number(d.latitude);
                    var lon = Number(d.longitude);
                    if (!isFinite(lat) || !isFinite(lon)) { return null; }
                    return [lat, lon];
                }

                function distanceMeters(a, b) {
                    var earthRadiusMeters = 6378137;
                    var lat1 = a[0] * Math.PI / 180.0;
                    var lat2 = b[0] * Math.PI / 180.0;
                    var deltaLat = (b[0] - a[0]) * Math.PI / 180.0;
                    var deltaLon = (b[1] - a[1]) * Math.PI / 180.0;
                    var sinLat = Math.sin(deltaLat / 2.0);
                    var sinLon = Math.sin(deltaLon / 2.0);
                    var h = sinLat * sinLat + Math.cos(lat1) * Math.cos(lat2) * sinLon * sinLon;
                    return 2.0 * earthRadiusMeters * Math.atan2(Math.sqrt(h), Math.sqrt(1.0 - h));
                }

                function updatePathLine(role, point, color) {
                    if (!pathPoints[role]) { pathPoints[role] = []; }

                    var points = pathPoints[role];
                    var lastPoint = points[points.length - 1];
                    if (!lastPoint || distanceMeters(lastPoint, point) >= minPathPointDistanceMeters) {
                        points.push(point);
                        if (points.length > maxPathPoints) {
                            points.shift();
                        }
                    }

                    if (pathLines[role]) {
                        pathLines[role].setLatLngs(points);
                        pathLines[role].setStyle({color: color});
                    } else {
                        pathLines[role] = L.polyline(points, {
                            color: color,
                            weight: 3,
                            opacity: 0.85,
                            lineCap: 'round',
                            lineJoin: 'round'
                        }).addTo(map);
                        pathLines[role].bringToBack();
                    }
                }

	            function removeMissingDrones(drones) {
	                for (var role in markers) {
	                    if (!Object.prototype.hasOwnProperty.call(drones, role)) {
	                        map.removeLayer(markers[role]);
	                        delete markers[role];
	                    }
	                }
                    for (var pathRole in pathLines) {
                        if (!Object.prototype.hasOwnProperty.call(drones, pathRole)) {
                            map.removeLayer(pathLines[pathRole]);
                            delete pathLines[pathRole];
                            delete pathPoints[pathRole];
                        }
                    }
	            }
	            
	            function updateAllDrones(drones) {
	                removeMissingDrones(drones);
	                for (var role in drones) {
	                    var d = drones[role];
                        var point = getPoint(d);
                        if (!point) { continue; }
	                    var color = '#b30000'; 
                    if (d.slot === 0) color = '#FFD700';
                    else if (d.slot > 0) color = '#00FF66';
                    else color = '#FF4444';
                    
                    if (markers[role]) {
                        markers[role].setLatLng(point);
                        markers[role].setIcon(createDroneIcon(d.heading, color));
                    } else {
	                        markers[role] = L.marker(point, {icon: createDroneIcon(d.heading, color)}).addTo(map);
	                        markers[role].bindTooltip("ID: " + role, {permanent: true, direction: 'top', className: 'drone-label', offset: [0, -15]});
	                    }
	                    updatePathLine(role, point, color);
	                    if (d.slot === 0 && !userDragged) { map.setView(point, map.getZoom(), {animate: false}); }
	                }
	            }
        </script>
        </body></html>
        """
        self.browser.setHtml(html)

    def haritayi_js_ile_guncelle(self):
        if not self.browser or not self.son_telemetri: return
        harita_verisi = {
            role: data
            for role, data in self.son_telemetri.items()
            if data.get("latitude") is not None and data.get("longitude") is not None
        }
        if not harita_verisi:
            return
        js_data = json.dumps(harita_verisi)
        js_code = f"updateAllDrones({js_data});" 
        self.browser.page().runJavaScript(js_code)

    def ros_spin(self): rclpy.spin_once(self, timeout_sec=0)

    def kalkis_baslat(self):
        try:
            irtifa = float(self.kalkis_yuksekligi.text().strip())
            enlem_val = float(self.enlem.text().strip())
            boylam_val = float(self.boylam.text().strip())
            mesafe = float(self.takip_mesafesi.text().strip())
            gecerli, hata = self.rolleri_dogrula(self.dron_rolleri)
            if not gecerli:
                self.log_ekle(f"Görev başlatılamadı: {hata}", role="sol")
                return
            if irtifa <= 0 or mesafe <= 0:
                self.log_ekle("Görev başlatılamadı: irtifa ve takip mesafesi pozitif olmalı.", role="sol")
                return

            plan = {
                "roles": self.dron_rolleri, "takeoff_altitude": irtifa,
                "follow_distance_m": mesafe, "latitude": enlem_val, "longitude": boylam_val
            }
            msg = String(); msg.data = json.dumps(plan); self.command_publisher.publish(msg)
            self.gorev_basladi = True
            self.btn_rol_atama.setEnabled(False)
            self.log_ekle(f"GÖREV YAYINLANDI -> {len(self.dron_rolleri)} Dron Harekete Geçiyor.")
        except ValueError:
            self.log_ekle("HATA: Değerleri kontrol edin!", role="sol")

    def acil_inis_yap(self): 
        msg = String(); msg.data = json.dumps({"command": "emergency_land"}); self.emergency_land_publisher.publish(msg)
        self.log_ekle("ACİL İNİŞ KOMUTU GÖNDERİLDİ.", role="sol")
        
    def bekleme_yap(self):
        self.log_ekle("HOLD komutu gerçek görev akışında pasif.", role="lider")

def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    
    app.setStyleSheet("""
        QMainWindow { background-color: #05080c; } 
        QScrollArea#sol_scroll { background: transparent; border: none; }
        QScrollArea#sol_scroll > QWidget { background: transparent; }
        QFrame#sol_frame { background: transparent; border: none; }
        QFrame#sag_frame { background-color: rgba(12, 17, 26, 180); border-radius: 12px; border: 1px solid rgba(0, 229, 255, 40); }
        QGroupBox { color: #00e5ff; font-weight: bold; border: 1px solid rgba(255, 255, 255, 30); border-radius: 6px; margin-top: 15px; }
        QGroupBox#groupBox_4 { background-color: rgba(12, 17, 26, 190); border: 1px solid rgba(0, 229, 255, 45); border-radius: 10px; margin-top: 15px; }
        QGroupBox#groupBox_4::title { color: #00e5ff; subcontrol-origin: margin; left: 12px; padding: 0 4px; }
        QLabel { color: #ffffff; background: transparent; font-weight: bold; font-size: 11px; }
        QLineEdit { background-color: rgba(0, 20, 30, 200); border: 1px solid rgba(0, 229, 255, 80); border-radius: 4px; color: #00ffff; padding: 3px; font-weight: bold; }
        QLCDNumber { color: #00ffff; background-color: rgba(0, 20, 30, 150); border: 1px solid rgba(0, 229, 255, 50); border-radius: 4px; }
        QProgressBar { background-color: rgba(0, 20, 30, 150); border: 1px solid rgba(0, 229, 255, 30); border-radius: 2px; color: white; text-align: center; font-weight: bold; font-size: 11px; }
        QTextEdit#log_ekrani { background-color: rgba(12, 17, 26, 180); border: 1px solid rgba(0, 229, 255, 40); border-radius: 12px; color: #fff; padding: 10px; }
        QPushButton { background-color: rgba(0, 123, 255, 180); color: white; border-radius: 6px; padding: 8px; font-weight: bold; border: 1px solid #0056b3; }
        QPushButton:hover { background-color: rgba(0, 86, 179, 255); }
        QScrollArea#sol_scroll QScrollBar:vertical { background: rgba(0,0,0,50); width: 8px; }
        QScrollArea#sol_scroll QScrollBar::handle:vertical { background: rgba(0, 229, 255, 100); border-radius: 4px; }
    """)
    win = YerKontrolIstasyonu()
    win.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
