import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PyQt5 import QtWidgets, uic, QtCore, QtWebEngineWidgets

from arayuz_pkg.resources import resource_path

class YerKontrolIstasyonu(Node, QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__('qgc_node')
        QtWidgets.QMainWindow.__init__(self)
        uic.loadUi(str(resource_path('yenitasarım.ui')), self) 
        self.showMaximized()

        self.role_map = {"lider": 1, "sag": 2, "sağ": 2, "sol": 3}
        self.paneller = {}
        self.publisherlar = {}
        self.veri_akisi_aktif = False 

        self.browser = QtWebEngineWidgets.QWebEngineView()
        self.haritayi_hazirla()
        
        if hasattr(self, 'groupBox_5'):
            layout = QtWidgets.QVBoxLayout(self.groupBox_5)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.addWidget(self.browser)

        self.panelleri_kur()
        self.subscription = self.create_subscription(String, 'yki_drone_topic', self.telemetri_callback, 10)

        self.pushButton.clicked.connect(self.kalkis_baslat)
        self.pushButton_2.clicked.connect(self.acil_inis_yap)
        self.pushButton_3.clicked.connect(lambda: self.komut_yayinla("BEKLEME"))

        self.ros_timer = QtCore.QTimer()
        self.ros_timer.timeout.connect(self.ros_spin)
        self.ros_timer.start(30)

    def haritayi_hazirla(self):
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
            <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
            <style>
                #map { height: 100vh; width: 100vw; margin: 0; background: #000; }
                body { margin: 0; padding: 0; overflow: hidden; }
                .drone-label {
                    background: rgba(0, 0, 0, 0.7);
                    color: #fff;
                    border: 1px solid #00e5ff;
                    border-radius: 3px;
                    padding: 1px 4px;
                    font-size: 10px;
                    font-weight: bold;
                    white-space: nowrap;
                }
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                var map = L.map('map', {
                    center: [37.4122, -121.9985],
                    zoom: 19,
                    maxZoom: 22,
                    zoomControl: false
                });

                L.tileLayer('https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}', {
                    maxZoom: 22
                }).addTo(map);

                var markers = {};

                function createArrow(heading, color) {
                    return L.divIcon({
                        className: 'arrow-icon',
                        html: `<div style="transform: rotate(${heading}deg); width: 0; height: 0; border-left: 8px solid transparent; border-right: 8px solid transparent; border-bottom: 22px solid ${color}; filter: drop-shadow(0 0 3px black);"></div>`,
                        iconSize: [16, 22],
                        iconAnchor: [8, 11]
                    });
                }

                function updateMap(role, lat, lon, heading) {
                    var color = role === 'lider' ? '#ff3d00' : '#00e5ff';
                    if (markers[role]) {
                        markers[role].setLatLng([lat, lon]);
                        markers[role].setIcon(createArrow(heading, color));
                    } else {
                        markers[role] = L.marker([lat, lon], {icon: createArrow(heading, color)}).addTo(map);
                        markers[role].bindTooltip(role.toUpperCase(), {
                            permanent: true, direction: 'top', className: 'drone-label', offset: [0, -10]
                        });
                    }
                }
            </script>
        </body>
        </html>
        """
        self.browser.setHtml(html)

    def telemetri_callback(self, msg):
        if not self.veri_akisi_aktif: return
        try:
            data = json.loads(msg.data)
            role_raw = str(data.get("drone_role", "")).lower().strip()
            d_id = self.role_map.get(role_raw)

            if d_id and d_id in self.paneller:
                p = self.paneller[d_id]
                heading = data.get('heading', 0)
                
                p.findChild(QtWidgets.QLCDNumber, "lcd_hiz").display(round(data.get('speed', 0), 2))
                p.findChild(QtWidgets.QLCDNumber, "lcd_irtifa").display(round(data.get('absolute_altitude', 0), 2))
                lbl = p.findChild(QtWidgets.QLabel, "label_63")
                if lbl: lbl.setText(f"{int(heading)}°")

                js = f"updateMap('{role_raw}', {data['latitude']}, {data['longitude']}, {heading});"
                self.browser.page().runJavaScript(js)

                try:
                    from arayuz_pkg.pusula_kodu import DronluPusula
                    for pusula in p.findChildren(DronluPusula):
                        pusula.aci = heading
                        pusula.update()
                except: pass
        except Exception as e: print(f"Hata: {e}")

    def panelleri_kur(self):
        layout = self.verticalLayout_2
        while layout.count(): layout.takeAt(0).widget().deleteLater()
        panel_basliklari = {1: "LİDER DRON", 2: "SAĞ DRON", 3: "SOL DRON"}
        for i in range(1, 4):
            panel = QtWidgets.QWidget()
            uic.loadUi(str(resource_path('drone_panel.ui')), panel)
            layout.addWidget(panel)
            gb = panel.findChild(QtWidgets.QGroupBox)
            if gb: gb.setTitle(panel_basliklari[i])
            lbl16 = panel.findChild(QtWidgets.QLabel, "label_16")
            if lbl16: lbl16.hide()
            self.publisherlar[i] = self.create_publisher(String, f'/drone_{i}/komut', 10)
            self.paneller[i] = panel

    def kalkis_baslat(self): self.veri_akisi_aktif = True; self.komut_yayinla("KALKIS")
    def acil_inis_yap(self): self.veri_akisi_aktif = False; self.komut_yayinla("ACIL_INIS")
    def komut_yayinla(self, komut):
        for i in range(1, 4):
            msg = String(); msg.data = komut; self.publisherlar[i].publish(msg)

    def ros_spin(self): rclpy.spin_once(self, timeout_sec=0)

def main():
    rclpy.init(); app = QtWidgets.QApplication(sys.argv)
    win = YerKontrolIstasyonu(); win.show(); sys.exit(app.exec_())

if __name__ == '__main__': main()
