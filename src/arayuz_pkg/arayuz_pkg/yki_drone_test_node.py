import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class YkiDroneTestNode(Node):
    def __init__(self):
        super().__init__("yki_drone_test_node")
        
        self.publisher_ = self.create_publisher(String, "yki_drone_topic", 10)
        self.command_subscriber_ = self.create_subscription(String, "yki_command_topic", self.command_callback, 10)
        self.timer_ = self.create_timer(0.1, self.publish_test_data) 
        
        self.base_lat = 37.4122
        self.base_lon = -121.9985
        self.target_altitude = 25.0 
        self.takip_offset = 0.00015
        
        self.aktif_roller = {"14541": -1, "14542": 1, "14543": 0}
        self.battery_levels = {"14541": 100.0, "14542": 100.0, "14543": 100.0}
        
        self.flight_state = "WAITING" 
        self.current_lat_offset = 0.0
        self.current_lon_offset = 0.0
        self.current_altitude = 0.0 
        self.current_speed = 0.0    
        self.current_heading = 0.0
        self.step_ = 0

        self.get_logger().info("Sistem Yerde. YENİ 'PLAN' MODÜLÜ AKTİF. Kalkış Bekleniyor...")

    # --- YENİ FORMATA UYGUN ŞEKİLDE GELEN VERİYİ AYIKLAYAN KISIM ---
    def command_callback(self, msg):
        try:
            plan = json.loads(msg.data)
            
            # Eğer gelen pakette "takeoff_altitude" varsa bu bir Görev/Kalkış Planıdır
            if "takeoff_altitude" in plan:
                self.target_altitude = plan.get("takeoff_altitude", 10.0)
                self.base_lat = plan.get("latitude", 37.4122)
                self.base_lon = plan.get("longitude", -121.9985)
                
                takip_m = plan.get("follow_distance_m", 5.0)
                self.takip_offset = takip_m * 0.000009
                
                if "roles" in plan:
                    # JSON'dan gelen ID'ler string olacağı için test node formatına çeviriyoruz
                    self.aktif_roller = {str(k): int(v) for k, v in plan["roles"].items()}
                
                self.current_lat_offset = 0.0
                self.current_lon_offset = 0.0
                
                self.get_logger().info(f"YENİ PLAN GELDİ! Merkez: {self.base_lat}, {self.base_lon}")
                self.flight_state = "FLYING"
                
            else:
                # Eğer pakette "takeoff_altitude" yoksa standart buton komutlarından biridir
                komut = plan.get("komut", "")
                if komut == "UPDATE_ROLES":
                    if "roles" in plan:
                        self.aktif_roller = {str(k): int(v) for k, v in plan["roles"].items()}
                    self.get_logger().info(f"Roller Güncellendi: {self.aktif_roller}")
                    
                elif komut == "LAND":
                    self.flight_state = "LANDING"
                    
                elif komut == "HOLD":
                    self.flight_state = "HOVERING"

        except json.JSONDecodeError:
            pass 

    def publish_test_data(self):
        durum_gonder = "YERDE"
        renk_gonder = "gri"
        
        if self.flight_state == "FLYING":
            self.step_ += 1
            if self.current_speed < 15.0: self.current_speed += 0.5
            
            if self.current_altitude < self.target_altitude: self.current_altitude += 0.5
            elif self.current_altitude > self.target_altitude + 0.5: self.current_altitude -= 0.5

            self.current_heading = (self.step_ * 2) % 360
            self.current_lat_offset += 0.000005
            self.current_lon_offset += 0.0000025
            
            for d_id in self.aktif_roller.keys():
                if d_id not in self.battery_levels: self.battery_levels[d_id] = 100.0
                if self.battery_levels[d_id] > 15.0: self.battery_levels[d_id] -= 0.02
            
            durum_gonder = "UÇUŞTA"
            renk_gonder = "yesil"

        elif self.flight_state == "LANDING":
            if self.current_altitude > 0: self.current_altitude -= 0.5  
            else: self.current_altitude = 0.0
            if self.current_speed > 0: self.current_speed -= 0.5 
            else: self.current_speed = 0.0
            if self.current_altitude == 0 and self.current_speed == 0: self.flight_state = "WAITING"
            durum_gonder = "İNİŞTE"
            renk_gonder = "mavi"
                
        elif self.flight_state == "HOVERING":
            if self.current_speed > 0: self.current_speed -= 1.0
            else: self.current_speed = 0.0
            durum_gonder = "BEKLEMEDE"
            renk_gonder = "sari"
                
        elif self.flight_state == "WAITING":
            self.current_speed = 0.0
            durum_gonder = "YERDE"
            renk_gonder = "gri"

        combined_packet = {}
        for d_id, slot in self.aktif_roller.items():
            if slot == 0:
                lat_off, lon_off = 0.0, 0.0
            elif slot > 0:
                lat_off, lon_off = -self.takip_offset * slot, self.takip_offset * slot
            else:
                lat_off, lon_off = -self.takip_offset * abs(slot), -self.takip_offset * abs(slot)

            combined_packet[d_id] = {
                "latitude": self.base_lat + self.current_lat_offset + lat_off,
                "longitude": self.base_lon + self.current_lon_offset + lon_off,
                "absolute_altitude": max(0.0, self.current_altitude),
                "speed": max(0.0, self.current_speed),
                "heading": self.current_heading,
                "durum_text": durum_gonder,  
                "renk": renk_gonder,
                "battery": int(self.battery_levels.get(d_id, 100)),
                "slot": slot  
            }
            
        msg = String()
        msg.data = json.dumps(combined_packet)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = YkiDroneTestNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()