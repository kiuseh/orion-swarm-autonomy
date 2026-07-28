import copy
import json
import queue
import subprocess
import threading
import time
from pathlib import Path

import cv2
import gz.msgs10.image_pb2 as gz_image_pb2
import gz.transport13 as gz_transport
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

DEFAULT_GZ_IMAGE_TOPIC = (
    "/world/default/model/x500_down_cam/link/down_cam_link/sensor/down_camera/image"
)
DISPLAY_WINDOW_NAME = "IHA Kamera (Raw)"
DISPLAY_WINDOW_DEFAULT_WIDTH = 1280
DISPLAY_WINDOW_DEFAULT_HEIGHT = 720
DEFAULT_PROCESSING_HZ = 15.0
VISION_CONTROL_TOPIC = "vision_control_topic"
COLORED_FIELD_TOPIC = "colored_field_topic"
X_Y_ERROR_TOPIC = "x_y_error_topic"
QR_RESULT_TOPIC = "qr_result_topic"
UDP_SCOPED_TOPIC_SUFFIXES = (
    VISION_CONTROL_TOPIC,
    COLORED_FIELD_TOPIC,
    X_Y_ERROR_TOPIC,
    QR_RESULT_TOPIC,
)
IMAGE_TOPIC_SUFFIXES = (
    "/link/down_cam_link/sensor/down_camera/image",
    "/sensor/down_camera/image",
)
WECHAT_QR_MODEL_DIR = Path(__file__).resolve().parent / "models" / "wechat_qr"


def strip_udp_scoped_topic_prefix(topic_name):
    for suffix in UDP_SCOPED_TOPIC_SUFFIXES:
        if topic_name == suffix or topic_name.endswith(f"_{suffix}"):
            return suffix
    return topic_name

MESAFE_OLCEK = {
    "10m": 1.0,
    "15m": 1.4,
    "20m": 2.0,
    "30m": 3.0,
}

MISSION_LIBRARY = {
    "gorev_1": {
        "qr_id": 1,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "V",
            },
            "manevra_pitch_roll": {
                "aktif": False,
                "pitch_deg": "0",
                "roll_deg": "20",
            },
            "irtifa_degisimi": {
                "aktif": False,
                "deger": 10,
            },
            "bekleme_suresi": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi": None,
        },
        "sonraki_qr": {
            "latitude": 37.412175143823063,
            "longitude": -121.998676647076721,
        },
    },
    "gorev_2": {
        "qr_id": 2,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "V",
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "0",
                "roll_deg": "20",
            },
            "irtifa_degisimi": {
                "aktif": True,
                "deger": 10,
            },
            "bekleme_suresi": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi": None,
        },
        "sonraki_qr": {
            "latitude": 37.412272182936924,
            "longitude": -121.99842689755684,
        },
    },
    "gorev_3": {
        "qr_id": 3,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "Okbaşı",
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "-20",
                "roll_deg": "0",
            },
            "irtifa_degisimi": {
                "aktif": False,
                "deger": 10,
            },
            "bekleme_suresi": 3,
        },
        "suruden_ayrilma": {
            "aktif": True,
            "ayrilacak_drone_id": 1,
            "hedef_renk": "red",
            "bekleme_suresi": 3,
        },
        "sonraki_qr": 0,
    },
    "gorev_4": {
        "qr_id": 4,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "Çizgi",
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "0",
                "roll_deg": "-20",
            },
            "irtifa_degisimi": {
                "aktif": True,
                "deger": 7.5,
            },
            "bekleme_suresi": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi": None,
        },
        "sonraki_qr": 0,
    },
    "gorev_5": {
        "qr_id": 5,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "Okbaşı",
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "-20",
                "roll_deg": "0",
            },
            "irtifa_degisimi": {
                "aktif": False,
                "deger": 10,
            },
            "bekleme_suresi": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi": None,
        },
        "sonraki_qr": {
            "latitude": 37.412536813213983,
            "longitude": -121.998873757815232,
        },
    },
    "gorev_6": {
        "qr_id": 6,
        "gorev": {
            "formasyon": {
                "aktif": True,
                "tip": "Çizgi",
            },
            "manevra_pitch_roll": {
                "aktif": True,
                "pitch_deg": "0",
                "roll_deg": "0",
            },
            "irtifa_degisimi": {
                "aktif": False,
                "deger": 10,
            },
            "bekleme_suresi": 3,
        },
        "suruden_ayrilma": {
            "aktif": False,
            "ayrilacak_drone_id": None,
            "hedef_renk": None,
            "bekleme_suresi": None,
        },
        "sonraki_qr": 0,
    },
}

QR_SCAN_INTERVAL = 1
QR_REDETECT_INTERVAL = 3
QR_TRACK_TIMEOUT_SEC = 0.35
QR_PENDING_CONFIRMATIONS = 1
QR_PENDING_TIMEOUT_SEC = 0.8
QR_ROI_SCALE = 1.8
QR_ROI_MIN_SIZE = 140
QR_DETECTION_SMOOTHING = 0.65
QR_MAX_TRACK_CENTER_JUMP = 90.0
QR_MIN_TRACK_AREA_RATIO = 0.35
QR_MAX_TRACK_AREA_RATIO = 2.8
QR_TRACK_FINDER_MEAN_MIN = 0.48
QR_FAST_FULL_SCALES = (1.0,)
QR_FAST_SCAN_SCALES_ROI = (1.0,)
QR_DECODE_RETRY_INTERVAL = 2
QR_HEAVY_FALLBACK_INTERVAL = 15
QR_RESULT_OVERLAY_HOLD_SEC = 2.5
CIRCLE_PROCESS_SCALE = 0.5
CONSOLE_UPDATE_INTERVAL_SEC = 0.25
CIRCLE_MIN_CIRCULARITY = 0.85
CIRCLE_MAX_CIRCULARITY = 1.15
CIRCLE_MIN_ASPECT_RATIO = 0.8
CIRCLE_MAX_ASPECT_RATIO = 1.2
COLOR_TARGETS = (
    {
        "etiket": "KIRMIZI",
        "topic_type": "red",
        "bgr": (0, 0, 255),
        "alt_hsv": (
            np.array([0, 150, 70], dtype=np.uint8),
            np.array([170, 150, 70], dtype=np.uint8),
        ),
        "ust_hsv": (
            np.array([10, 255, 255], dtype=np.uint8),
            np.array([180, 255, 255], dtype=np.uint8),
        ),
    },
    {
        "etiket": "MAVI",
        "topic_type": "blue",
        "bgr": (255, 0, 0),
        "alt_hsv": (
            np.array([100, 170, 100], dtype=np.uint8),
        ),
        "ust_hsv": (
            np.array([140, 255, 255], dtype=np.uint8),
        ),
    },
)

QR_FINDER_TEMPLATE = np.array(
    [
        [1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1],
    ],
    dtype=np.uint8,
)


def _normalize_mission_payload(payload):
    normalized = str(payload).strip().lower()
    if not normalized:
        return ""
    return normalized


def _resolve_mission_plan(payload):
    normalized_payload = _normalize_mission_payload(payload)
    if not normalized_payload:
        return None, None

    mission_plan = MISSION_LIBRARY.get(normalized_payload)
    if mission_plan is None:
        return normalized_payload, None

    return normalized_payload, copy.deepcopy(mission_plan)


def _qr_preprocess_variants(gray_frame):
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray_frame)
    adaptive = cv2.adaptiveThreshold(
        clahe,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        31,
        2,
    )
    return (
        ("ham", gray_frame),
        ("kontrast", clahe),
        ("esikleme", adaptive),
    )


def _qr_preprocess_color_variants(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    sharpened = cv2.addWeighted(
        frame,
        1.35,
        cv2.GaussianBlur(frame, (0, 0), 1.2),
        -0.35,
        0,
    )
    return (
        ("ham", frame),
        ("kinestir", sharpened),
        ("kontrast", cv2.cvtColor(clahe, cv2.COLOR_GRAY2BGR)),
    )


def _qr_roi_genislet(noktalar, frame_shape, scale=1.35, min_margin=24):
    pts = _sirali_qr_noktalari(noktalar)
    if pts is None:
        return None

    h, w = frame_shape[:2]
    x_min = float(np.min(pts[:, 0]))
    y_min = float(np.min(pts[:, 1]))
    x_max = float(np.max(pts[:, 0]))
    y_max = float(np.max(pts[:, 1]))
    cx = (x_min + x_max) * 0.5
    cy = (y_min + y_max) * 0.5
    roi_w = max((x_max - x_min) * scale, (x_max - x_min) + min_margin * 2)
    roi_h = max((y_max - y_min) * scale, (y_max - y_min) + min_margin * 2)

    x0 = max(0, int(round(cx - roi_w * 0.5)))
    y0 = max(0, int(round(cy - roi_h * 0.5)))
    x1 = min(w, int(round(cx + roi_w * 0.5)))
    y1 = min(h, int(round(cy + roi_h * 0.5)))
    if x1 - x0 < 40 or y1 - y0 < 40:
        return None
    return x0, y0, x1, y1


class WeChatQRBackend:
    def __init__(self):
        self.detector = None
        self.available = False
        self.last_error = ""

        if not hasattr(cv2, "wechat_qrcode_WeChatQRCode"):
            self.last_error = "OpenCV wechat_qrcode modulu yok"
            return

        paths = {
            "detect_prototxt": WECHAT_QR_MODEL_DIR / "detect.prototxt",
            "detect_model": WECHAT_QR_MODEL_DIR / "detect.caffemodel",
            "sr_prototxt": WECHAT_QR_MODEL_DIR / "sr.prototxt",
            "sr_model": WECHAT_QR_MODEL_DIR / "sr.caffemodel",
        }
        missing = [str(path) for path in paths.values() if not path.exists()]
        if missing:
            self.last_error = "WeChat QR model dosyalari eksik"
            return

        try:
            self.detector = cv2.wechat_qrcode_WeChatQRCode(
                str(paths["detect_prototxt"]),
                str(paths["detect_model"]),
                str(paths["sr_prototxt"]),
                str(paths["sr_model"]),
            )
            self.available = True
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"

    def detect(self, frame):
        if not self.available or self.detector is None:
            return []

        try:
            texts, points = self.detector.detectAndDecode(frame)
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            return []

        if points is None:
            return []

        bulunan = []
        for idx, pts in enumerate(points):
            mesaj = ""
            if texts and idx < len(texts):
                mesaj = (texts[idx] or "").strip()
            bulunan.append(
                {
                    "noktalar": np.asarray(pts, dtype=np.float32),
                    "cozuldu": bool(mesaj),
                    "payload": mesaj,
                }
            )
        return bulunan


def _qr_roi_rect(gray_frame, qr_listesi):
    if not qr_listesi:
        return None

    h, w = gray_frame.shape[:2]
    tum_noktalar = []
    for qr in qr_listesi:
        pts = _sirali_qr_noktalari(qr.get("noktalar"))
        if pts is not None:
            tum_noktalar.append(pts)

    if not tum_noktalar:
        return None

    pts = np.concatenate(tum_noktalar, axis=0)
    x_min = float(np.min(pts[:, 0]))
    y_min = float(np.min(pts[:, 1]))
    x_max = float(np.max(pts[:, 0]))
    y_max = float(np.max(pts[:, 1]))

    cx = (x_min + x_max) * 0.5
    cy = (y_min + y_max) * 0.5
    kutu_w = max((x_max - x_min) * QR_ROI_SCALE, QR_ROI_MIN_SIZE)
    kutu_h = max((y_max - y_min) * QR_ROI_SCALE, QR_ROI_MIN_SIZE)

    x0 = max(0, int(round(cx - kutu_w * 0.5)))
    y0 = max(0, int(round(cy - kutu_h * 0.5)))
    x1 = min(w, int(round(cx + kutu_w * 0.5)))
    y1 = min(h, int(round(cy + kutu_h * 0.5)))

    if x1 - x0 < 32 or y1 - y0 < 32:
        return None
    return x0, y0, x1, y1


def _qr_candidates_from_variant(
    variant_frame,
    detector,
    allow_detect_only=False,
    decode_first=True,
):
    bulunan = []

    if decode_first:
        retval, decoded_info, points, _ = detector.detectAndDecodeMulti(variant_frame)
        if retval and points is not None:
            for idx, pts in enumerate(points):
                mesaj = ""
                if decoded_info and idx < len(decoded_info):
                    mesaj = (decoded_info[idx] or "").strip()
                bulunan.append(
                    {
                        "noktalar": np.asarray(pts, dtype=np.float32),
                        "cozuldu": bool(mesaj),
                        "payload": mesaj,
                    }
                )

        if bulunan:
            return bulunan

    if allow_detect_only:
        retval, points = detector.detectMulti(variant_frame)
        if retval and points is not None:
            for pts in points:
                bulunan.append(
                    {
                        "noktalar": np.asarray(pts, dtype=np.float32),
                        "cozuldu": False,
                        "payload": "",
                    }
                )

    return bulunan


def _qr_bulunan_ekle(bulunan, etiket, aday, gray_frame, display_only=False):
    noktalar = _sirali_qr_noktalari(aday["noktalar"])
    if noktalar is None:
        return
    if display_only:
        if not _qr_geometri_gecerli_mi(gray_frame, noktalar):
            return
    elif not _qr_aday_gecerli_mi(gray_frame, noktalar, aday.get("cozuldu", False)):
        return
    merkez = noktalar.mean(axis=0)
    alan = abs(cv2.contourArea(noktalar))

    for mevcut in bulunan:
        mevcut_pts = np.asarray(mevcut["noktalar"], dtype=np.float32).reshape(-1, 2)
        mevcut_merkez = mevcut_pts.mean(axis=0)
        mevcut_alan = abs(cv2.contourArea(mevcut_pts))
        if (
            np.linalg.norm(merkez - mevcut_merkez) < 12.0
            and abs(alan - mevcut_alan) / max(alan, mevcut_alan, 1.0) < 0.25
        ):
            if aday.get("cozuldu"):
                mevcut["cozuldu"] = True
            if aday.get("payload"):
                mevcut["payload"] = aday["payload"]
            return

    bulunan.append(
        {
            "mesafe": etiket,
            "noktalar": noktalar,
            "cozuldu": bool(aday.get("cozuldu", False)),
            "payload": str(aday.get("payload", "")),
        }
    )


def _qr_hizli_tara(
    gri_tarama,
    classic_detector,
    full_gray,
    offset_x,
    offset_y,
    allow_detect_only=False,
    scales=(1.0,),
    variant_limit=2,
    decode_first=True,
    display_only=False,
):
    bulunan = []
    h, w = gri_tarama.shape[:2]

    for olcek in scales:
        hedef = (
            gri_tarama
            if olcek == 1.0
            else cv2.resize(
                gri_tarama,
                (int(w * olcek), int(h * olcek)),
                interpolation=cv2.INTER_LINEAR,
            )
        )

        for _, variant in _qr_preprocess_variants(hedef)[:variant_limit]:
            adaylar = _qr_candidates_from_variant(
                variant,
                classic_detector,
                allow_detect_only=allow_detect_only,
                decode_first=decode_first,
            )
            if not adaylar:
                continue

            for aday in adaylar:
                _qr_bulunan_ekle(
                    bulunan,
                    "hizli",
                    {
                        "noktalar": (aday["noktalar"] / olcek)
                        + np.array([offset_x, offset_y], dtype=np.float32),
                        "cozuldu": aday.get("cozuldu", False),
                        "payload": str(aday.get("payload", "")).strip(),
                    },
                    full_gray,
                    display_only=display_only,
                )

            if bulunan:
                return bulunan

    return bulunan


def _sirali_qr_noktalari(noktalar):
    pts = np.asarray(noktalar, dtype=np.float32).reshape(-1, 2)
    if len(pts) < 4:
        return None

    if len(pts) > 4:
        rect = cv2.minAreaRect(pts)
        pts = cv2.boxPoints(rect)

    rect = np.zeros((4, 2), dtype=np.float32)
    toplam = pts.sum(axis=1)
    fark = np.diff(pts, axis=1).reshape(-1)

    rect[0] = pts[np.argmin(toplam)]
    rect[2] = pts[np.argmax(toplam)]
    rect[1] = pts[np.argmin(fark)]
    rect[3] = pts[np.argmax(fark)]

    if len({tuple(np.round(p, 3)) for p in rect}) != 4:
        merkez = pts.mean(axis=0)
        acilar = np.arctan2(pts[:, 1] - merkez[1], pts[:, 0] - merkez[0])
        rect = pts[np.argsort(acilar)]
        baslangic = np.argmin(rect.sum(axis=1))
        rect = np.roll(rect, -baslangic, axis=0)
        if rect[1][0] < rect[3][0]:
            rect[[1, 3]] = rect[[3, 1]]

    return rect


def _qr_cizim_noktalari(noktalar):
    return _sirali_qr_noktalari(noktalar)


def _sec_en_uygun_qr_adayi(adaylar, hedef_merkez, hedef_alan):
    if not adaylar:
        return None

    en_iyi = None
    en_iyi_skor = None
    for aday in adaylar:
        pts = _sirali_qr_noktalari(aday)
        if pts is None:
            continue
        alan = abs(cv2.contourArea(pts))
        if alan <= 1.0:
            continue
        merkez = pts.mean(axis=0)
        merkez_mesafe = float(np.linalg.norm(merkez - hedef_merkez))
        alan_orani = abs(alan - hedef_alan) / max(alan, hedef_alan, 1.0)
        skor = merkez_mesafe + (alan_orani * 120.0)
        if en_iyi_skor is None or skor < en_iyi_skor:
            en_iyi = pts
            en_iyi_skor = skor
    return en_iyi


def _classic_qr_kose_rafinasyonu(roi_gray, classic_detector):
    for _, variant in _qr_preprocess_variants(roi_gray):
        try:
            retval, points = classic_detector.detect(variant)
        except Exception:
            retval, points = False, None
        if retval and points is not None:
            pts = _sirali_qr_noktalari(points.reshape(-1, 2))
            if pts is not None:
                return pts

        try:
            retval, points = classic_detector.detectMulti(variant)
        except Exception:
            retval, points = False, None
        if retval and points is not None and len(points) > 0:
            h, w = roi_gray.shape[:2]
            hedef_merkez = np.array([w * 0.5, h * 0.5], dtype=np.float32)
            hedef_alan = float(w * h) * 0.35
            secilen = _sec_en_uygun_qr_adayi(points, hedef_merkez, hedef_alan)
            if secilen is not None:
                return secilen

    return None


def _kontur_qr_kose_rafinasyonu(roi_gray):
    bulanmis = cv2.GaussianBlur(roi_gray, (5, 5), 0)
    _, ikili = cv2.threshold(
        bulanmis, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    kernel = np.ones((5, 5), np.uint8)
    ikili = cv2.morphologyEx(ikili, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(ikili, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    h, w = roi_gray.shape[:2]
    hedef_merkez = np.array([w * 0.5, h * 0.5], dtype=np.float32)
    hedef_alan = float(w * h) * 0.30
    adaylar = []

    for cnt in contours:
        alan = cv2.contourArea(cnt)
        if alan < 0.03 * h * w or alan > 0.95 * h * w:
            continue
        rect = cv2.minAreaRect(cnt)
        rw, rh = rect[1]
        if rw <= 1.0 or rh <= 1.0:
            continue
        oran = max(rw, rh) / min(rw, rh)
        if oran > 1.35:
            continue
        adaylar.append(cv2.boxPoints(rect))

    return _sec_en_uygun_qr_adayi(adaylar, hedef_merkez, hedef_alan)


def _wechat_kose_rafinasyonu(frame, noktalar, classic_detector):
    ham_noktalar = _sirali_qr_noktalari(noktalar)
    if ham_noktalar is None:
        return None

    roi = _qr_roi_genislet(ham_noktalar, frame.shape)
    if roi is None:
        return ham_noktalar

    x0, y0, x1, y1 = roi
    roi_frame = frame[y0:y1, x0:x1]
    if roi_frame.size == 0:
        return ham_noktalar

    roi_gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)
    rafine = _classic_qr_kose_rafinasyonu(roi_gray, classic_detector)
    if rafine is None:
        rafine = _kontur_qr_kose_rafinasyonu(roi_gray)
    if rafine is None:
        return ham_noktalar

    rafine = rafine + np.array([x0, y0], dtype=np.float32)
    return _sirali_qr_noktalari(rafine)


def _qr_warp_kare(gray_frame, noktalar, boyut=96):
    pts = _sirali_qr_noktalari(noktalar)
    if pts is None:
        return None

    hedef = np.array(
        [
            [0, 0],
            [boyut - 1, 0],
            [boyut - 1, boyut - 1],
            [0, boyut - 1],
        ],
        dtype=np.float32,
    )
    mat = cv2.getPerspectiveTransform(pts.astype(np.float32), hedef)
    return cv2.warpPerspective(gray_frame, mat, (boyut, boyut))


def _qr_finder_benzerligi(patch):
    if patch is None or patch.size == 0:
        return 0.0
    kucuk = cv2.resize(patch, (7, 7), interpolation=cv2.INTER_AREA)
    ikili = (kucuk < 128).astype(np.uint8)
    return float(np.mean(ikili == QR_FINDER_TEMPLATE))


def _qr_finder_skorlari(gray_frame, noktalar):
    warp = _qr_warp_kare(gray_frame, noktalar)
    if warp is None:
        return []

    _, ikili = cv2.threshold(
        warp, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    pencere = 32
    return [
        _qr_finder_benzerligi(ikili[:pencere, :pencere]),
        _qr_finder_benzerligi(ikili[:pencere, -pencere:]),
        _qr_finder_benzerligi(ikili[-pencere:, :pencere]),
    ]


def _qr_geometri_gecerli_mi(gray_frame, noktalar):
    pts = _sirali_qr_noktalari(noktalar)
    if pts is None:
        return False

    alan = abs(cv2.contourArea(pts))
    if alan < 80.0:
        return False

    h, w = gray_frame.shape[:2]
    goruntu_alani = float(h * w)
    if alan / max(goruntu_alani, 1.0) > 0.45:
        return False

    rect = cv2.minAreaRect(pts)
    genislik, yukseklik = rect[1]
    if genislik <= 1.0 or yukseklik <= 1.0:
        return False
    oran = max(genislik, yukseklik) / min(genislik, yukseklik)
    if oran > 1.35:
        return False

    return True


def _qr_aday_gecerli_mi(gray_frame, noktalar, cozuldu):
    pts = _sirali_qr_noktalari(noktalar)
    if pts is None:
        return False

    if not _qr_geometri_gecerli_mi(gray_frame, pts):
        return False

    if cozuldu:
        return True

    benzerlikler = _qr_finder_skorlari(gray_frame, pts)
    if not benzerlikler:
        return False

    guclu_koseler = sum(score >= 0.6 for score in benzerlikler)
    return guclu_koseler >= 2 and float(np.mean(benzerlikler)) >= 0.58


def _qr_izleme_gecerli_mi(eski_noktalar, yeni_noktalar, gray_frame, cozuldu):
    eski = _sirali_qr_noktalari(eski_noktalar)
    yeni = _sirali_qr_noktalari(yeni_noktalar)
    if eski is None or yeni is None:
        return False

    eski_merkez = eski.mean(axis=0)
    yeni_merkez = yeni.mean(axis=0)
    merkez_sicrama = float(np.linalg.norm(yeni_merkez - eski_merkez))
    if merkez_sicrama > QR_MAX_TRACK_CENTER_JUMP:
        return False

    eski_alan = abs(cv2.contourArea(eski))
    yeni_alan = abs(cv2.contourArea(yeni))
    if eski_alan <= 1.0 or yeni_alan <= 1.0:
        return False

    alan_orani = yeni_alan / eski_alan
    if alan_orani < QR_MIN_TRACK_AREA_RATIO or alan_orani > QR_MAX_TRACK_AREA_RATIO:
        return False

    if not _qr_aday_gecerli_mi(gray_frame, yeni, cozuldu):
        if cozuldu:
            benzerlikler = _qr_finder_skorlari(gray_frame, yeni)
            if not benzerlikler:
                return False
            if max(benzerlikler) < 0.55 or float(np.mean(benzerlikler)) < QR_TRACK_FINDER_MEAN_MIN:
                return False
        else:
            return False

    return True


def qr_tara(
    frame,
    classic_detector,
    wechat_backend,
    search_rect=None,
    source="full_fast",
    allow_detect_only=False,
):
    bulunan = []
    offset_x = 0
    offset_y = 0
    tarama = frame
    full_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if search_rect is not None:
        x0, y0, x1, y1 = search_rect
        tarama = frame[y0:y1, x0:x1]
        if tarama.size == 0:
            return []
        offset_x = x0
        offset_y = y0

    gri_tarama = cv2.cvtColor(tarama, cv2.COLOR_BGR2GRAY)
    h, w = gri_tarama.shape[:2]

    if search_rect is None and source != "full_heavy":
        bulunan = _qr_hizli_tara(
            gri_tarama,
            classic_detector,
            full_gray,
            offset_x,
            offset_y,
            allow_detect_only=True,
            scales=QR_FAST_FULL_SCALES,
            variant_limit=1,
            decode_first=False,
            display_only=True,
        )
        if bulunan or source == "full_fast":
            return bulunan
    else:
        bulunan = _qr_hizli_tara(
            gri_tarama,
            classic_detector,
            full_gray,
            offset_x,
            offset_y,
            allow_detect_only=allow_detect_only,
            scales=QR_FAST_SCAN_SCALES_ROI,
            variant_limit=2,
            decode_first=True,
        )
        if bulunan:
            return bulunan

        if wechat_backend is not None and wechat_backend.available:
            for _, variant in _qr_preprocess_color_variants(tarama):
                adaylar = wechat_backend.detect(variant)
                if not adaylar:
                    continue

                for aday in adaylar:
                    rafine_noktalar = _wechat_kose_rafinasyonu(
                        tarama,
                        aday["noktalar"],
                        classic_detector,
                    )
                    _qr_bulunan_ekle(
                        bulunan,
                        "wechat",
                        {
                            "noktalar": np.asarray(
                                rafine_noktalar if rafine_noktalar is not None else aday["noktalar"],
                                dtype=np.float32,
                            )
                            + np.array([offset_x, offset_y], dtype=np.float32),
                            "cozuldu": aday.get("cozuldu", False),
                            "payload": str(aday.get("payload", "")).strip(),
                        },
                        full_gray,
                    )

                if bulunan:
                    return bulunan

    for etiket, olcek in MESAFE_OLCEK.items():
        hedef = (
            gri_tarama
            if olcek == 1.0
            else cv2.resize(
                gri_tarama,
                (int(w * olcek), int(h * olcek)),
                interpolation=cv2.INTER_LINEAR,
            )
        )

        for _, variant in _qr_preprocess_variants(hedef):
            adaylar = _qr_candidates_from_variant(
                variant,
                classic_detector,
                allow_detect_only=allow_detect_only,
                decode_first=True,
            )
            if not adaylar:
                continue

            for aday in adaylar:
                _qr_bulunan_ekle(
                    bulunan,
                    etiket,
                    {
                        "noktalar": (aday["noktalar"] / olcek)
                        + np.array([offset_x, offset_y], dtype=np.float32),
                        "cozuldu": aday.get("cozuldu", False),
                        "payload": str(aday.get("payload", "")).strip(),
                    },
                    full_gray,
                )

        if bulunan:
            break

    return bulunan


def discover_gz_image_topic():
    try:
        topics = gz_transport.Node().topic_list()
    except Exception:
        topics = []

    for suffix in IMAGE_TOPIC_SUFFIXES:
        for topic in topics:
            if topic.endswith(suffix):
                return topic

    for topic in topics:
        if "/sensor/" in topic and topic.endswith("/image"):
            return topic

    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return None

    if result.returncode != 0:
        return None

    topics = [line.strip() for line in result.stdout.splitlines() if line.strip()]

    for suffix in IMAGE_TOPIC_SUFFIXES:
        for topic in topics:
            if topic.endswith(suffix):
                return topic

    for topic in topics:
        if "/sensor/" in topic and topic.endswith("/image"):
            return topic

    return None


def discover_ros_image_topic(node: Node):
    for topic_name, topic_types in node.get_topic_names_and_types():
        if topic_name.endswith("/image") and "sensor_msgs/msg/Image" in topic_types:
            return topic_name

    return None


class RosImageBuffer:
    def __init__(self):
        from cv_bridge import CvBridge

        self.bridge = CvBridge()
        self.latest_frame = None
        self._lock = threading.Lock()
        self.received_frames = 0
        self.last_frame_time = 0.0
        self.last_error = ""

    def on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            return

        self.last_error = ""
        self.last_frame_time = time.time()
        self.received_frames += 1

        with self._lock:
            self.latest_frame = frame

    def get_frame(self):
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        return None


class GzImageBuffer:
    def __init__(self, topic: str):
        self.topic = topic
        self.node = gz_transport.Node()
        self.latest_frame = None
        self._lock = threading.Lock()
        self.received_frames = 0
        self.last_frame_time = 0.0
        self.last_error = ""
        self.subscribed = False

    def start(self):
        try:
            subscribed = self.node.subscribe(
                gz_image_pb2.Image, self.topic, self.on_image
            )
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            self.subscribed = False
            return

        if subscribed is False:
            self.last_error = f"Gazebo subscribe basarisiz: {self.topic}"
            self.subscribed = False
            return

        self.last_error = ""
        self.subscribed = True

    def restart(self, topic: str):
        self.stop()
        self.topic = topic
        self.start()

    def _decode_image(self, msg: gz_image_pb2.Image):
        width = int(msg.width)
        height = int(msg.height)
        step = int(msg.step)

        if width <= 0 or height <= 0:
            raise ValueError(f"Gecersiz boyut: {width}x{height}")
        if step <= 0:
            raise ValueError(f"Gecersiz step: {step}")

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        required = height * step
        if raw.size < required:
            raise ValueError(
                f"Eksik image data: {raw.size} < {required} byte"
            )

        rows = raw[:required].reshape(height, step)

        if msg.pixel_format_type == gz_image_pb2.RGB_INT8:
            pixels = rows[:, : width * 3].reshape(height, width, 3)
            return pixels[:, :, ::-1].copy()

        if msg.pixel_format_type == gz_image_pb2.BGR_INT8:
            return rows[:, : width * 3].reshape(height, width, 3).copy()

        if msg.pixel_format_type == gz_image_pb2.RGBA_INT8:
            pixels = rows[:, : width * 4].reshape(height, width, 4)
            return cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)

        if msg.pixel_format_type == gz_image_pb2.BGRA_INT8:
            pixels = rows[:, : width * 4].reshape(height, width, 4)
            return cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)

        if msg.pixel_format_type == gz_image_pb2.L_INT8:
            gray = rows[:, :width].reshape(height, width)
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        if msg.pixel_format_type == gz_image_pb2.L_INT16:
            gray16 = rows[:, : width * 2].reshape(height, width, 2).view(np.uint16)
            gray16 = gray16.reshape(height, width)
            max_value = int(gray16.max()) if gray16.size else 0
            gray8 = (
                cv2.convertScaleAbs(gray16, alpha=255.0 / max_value)
                if max_value > 0
                else gray16.astype(np.uint8)
            )
            return cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)

        raise ValueError(f"Desteklenmeyen pixel format: {msg.pixel_format_type}")

    def on_image(self, msg: gz_image_pb2.Image):
        try:
            frame = self._decode_image(msg)
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}: {exc}"
            return

        self.last_error = ""
        self.last_frame_time = time.time()
        self.received_frames += 1

        with self._lock:
            self.latest_frame = frame

    def get_frame(self):
        with self._lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None

    def stop(self):
        if self.subscribed:
            try:
                self.node.unsubscribe(self.topic)
            except Exception:
                pass
            self.subscribed = False


class QRScanThread(threading.Thread):
    def __init__(self, name="QRScanThread", use_wechat=True):
        super().__init__(daemon=True, name=name)
        self._q: queue.Queue = queue.Queue(maxsize=1)
        self._result_lock = threading.Lock()
        self._result = []
        self._result_version = 0
        self.running = True
        self.classic_detector = cv2.QRCodeDetector()
        self.wechat_backend = WeChatQRBackend() if use_wechat else None
        self.last_error = ""

    def submit(
        self,
        frame,
        search_rect=None,
        source="full",
        allow_detect_only=False,
    ):
        try:
            self._q.get_nowait()
        except queue.Empty:
            pass
        try:
            self._q.put_nowait(
                {
                    "frame": frame,
                    "search_rect": search_rect,
                    "source": source,
                    "allow_detect_only": allow_detect_only,
                }
            )
        except queue.Full:
            pass

    def get_result(self):
        with self._result_lock:
            result = []
            for qr in self._result:
                pts = qr["noktalar"]
                result.append(
                    {
                        "mesafe": qr["mesafe"],
                        "noktalar": None if pts is None else pts.copy(),
                        "cozuldu": qr.get("cozuldu", False),
                        "payload": str(qr.get("payload", "")),
                    }
                )
            return self._result_version, result

    def run(self):
        while self.running:
            try:
                task = self._q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                found = qr_tara(
                    task["frame"],
                    self.classic_detector,
                    self.wechat_backend,
                    search_rect=task.get("search_rect"),
                    source=task.get("source", "full_fast"),
                    allow_detect_only=task.get("allow_detect_only", False),
                )
            except Exception as exc:
                self.last_error = f"{type(exc).__name__}: {exc}"
                continue
            self.last_error = ""
            with self._result_lock:
                self._result = found
                self._result_version += 1

    def stop(self):
        self.running = False


class ImageProcessingNode(Node):

    def __init__(self):
        super().__init__("image_processing_node")

        self.declare_parameter("min_contour_area", 1200)
        self.declare_parameter("image_source", "auto")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("gz_image_topic", "")
        self.declare_parameter("auto_discover_topic", True)
        self.declare_parameter("enable_display", True)
        self.declare_parameter("display_window_name", DISPLAY_WINDOW_NAME)
        self.declare_parameter(
            "display_window_width",
            DISPLAY_WINDOW_DEFAULT_WIDTH,
        )
        self.declare_parameter(
            "display_window_height",
            DISPLAY_WINDOW_DEFAULT_HEIGHT,
        )
        self.declare_parameter("processing_hz", DEFAULT_PROCESSING_HZ)
        self.declare_parameter("qr_enabled_on_start", False)
        self.declare_parameter("udp_port", 14540)

        self.min_contour_area = self.get_parameter("min_contour_area").value
        requested_image_source = (
            self.get_parameter("image_source").value.strip().lower()
        )
        image_source = requested_image_source
        image_topic = self.get_parameter("image_topic").value.strip()
        gz_image_topic = self.get_parameter("gz_image_topic").value.strip()
        auto_discover_topic = self.get_parameter("auto_discover_topic").value
        self.enable_display = self.get_parameter("enable_display").value
        self.display_window_name = self.get_parameter("display_window_name").value
        self.display_window_width = int(
            self.get_parameter("display_window_width").value
        )
        self.display_window_height = int(
            self.get_parameter("display_window_height").value
        )
        self.processing_hz = float(self.get_parameter("processing_hz").value)
        if self.processing_hz <= 0.0:
            self.get_logger().warning(
                "Gecersiz processing_hz="
                f"{self.processing_hz}, {DEFAULT_PROCESSING_HZ} kullanilacak."
            )
            self.processing_hz = DEFAULT_PROCESSING_HZ
        self.qr_enabled_on_start = bool(
            self.get_parameter("qr_enabled_on_start").value
        )
        self.udp_port = int(self.get_parameter("udp_port").value)

        discovered_ros_topic = ""
        discovered_gz_topic = ""

        if auto_discover_topic:
            discovered_ros_topic = discover_ros_image_topic(self) or ""
            discovered_gz_topic = discover_gz_image_topic() or ""

        if not image_topic:
            image_topic = discovered_ros_topic

        if not gz_image_topic:
            gz_image_topic = discovered_gz_topic or DEFAULT_GZ_IMAGE_TOPIC

        if image_source not in {"auto", "ros", "gz"}:
            self.get_logger().warning(
                f"Gecersiz image_source={image_source}, auto kullanilacak."
            )
            image_source = "auto"

        if image_source == "auto":
            image_source = "ros" if image_topic else "gz"

        self._auto_source_requested = requested_image_source == "auto"
        self.image_source = image_source
        self.image_topic = image_topic
        self.gz_image_topic = gz_image_topic
        self.colored_field_detection_allowed = False
        self.xy_error_enabled = False
        self.qr_detection_allowed = False
        self.target_color = None
        self.vision_control_topic = f"udp_{self.udp_port}_{VISION_CONTROL_TOPIC}"
        self.x_y_error_topic = f"udp_{self.udp_port}_{X_Y_ERROR_TOPIC}"
        self.colored_field_topic = f"udp_{self.udp_port}_{COLORED_FIELD_TOPIC}"
        self.qr_result_topic = f"udp_{self.udp_port}_{QR_RESULT_TOPIC}"
        self._stream_ready_logged = False
        self._stream_wait_warning_logged = False
        self._last_stream_error = ""
        self._startup_time = time.time()

        self.image_sub = None

        self._configure_capture(self.image_source)

        self._cleanup_done = False
        self._frame_count = 0
        self._fps_counter = 0
        self._fps_t0 = time.time()
        self._fps_display = 0
        self.colored_field_publisher = self.create_publisher(
            String,
            self.colored_field_topic,
            10,
        )
        self.x_y_error_publisher = self.create_publisher(
            String,
            self.x_y_error_topic,
            10,
        )
        self.qr_result_publisher = self.create_publisher(
            String,
            self.qr_result_topic,
            10,
        )
        self.vision_control_subscriber = self.create_subscription(
            String,
            self.vision_control_topic,
            self.vision_control_callback,
            10,
        )
        self._onceki_renk_uzakliklari = {
            hedef["topic_type"]: None for hedef in COLOR_TARGETS
        }
        self.fast_qr_thread = None
        self.decode_qr_thread = None
        self.heavy_qr_thread = None
        self.qr_active = False
        self._qr_decode_reported = False
        self._qr_expected_qr_id = None
        self._qr_active_mission_index = None
        self._prev_gray = None
        self._tracked_qr = []
        self._visible_qr = []
        self._pending_qr = []
        self._last_fast_qr_result_version = -1
        self._last_decode_qr_result_version = -1
        self._last_heavy_qr_result_version = -1
        self._last_qr_refresh_time = 0.0
        self._last_qr_visible_time = 0.0
        self._last_console_update_time = 0.0
        self._last_full_qr_submit_frame = -QR_REDETECT_INTERVAL
        self._last_heavy_qr_submit_frame = -QR_HEAVY_FALLBACK_INTERVAL
        self._last_roi_qr_submit_frame = -1
        self._display_window_ready = False
        self._recent_qr_overlay = []
        self._recent_qr_overlay_until = 0.0

        print("\n" + "=" * 70)
        print(" IHA KAMERA SISTEMI ".center(70, "="))
        print(" Mesafe: 10m | 15m | 20m | 30m    q: cikis    s: kaydet ".center(70))
        print("=" * 70 + "\n")

        self._init_display_window()

        self.frame_timer = self.create_timer(
            1.0 / self.processing_hz,
            self.process_frame,
        )
        self.status_timer = self.create_timer(1.0, self._stream_status)
        self.get_logger().info(
            f"Goruntu isleme hizi {self.processing_hz:.2f} Hz olarak ayarlandi."
        )
        self.get_logger().info(
            f"Goruntu isleme udp_port={self.udp_port} ile basladi. "
            f"vision={self.vision_control_topic}, "
            f"colored_field={self.colored_field_topic}, "
            f"x_y_error={self.x_y_error_topic}, "
            f"qr_result={self.qr_result_topic}"
        )
        if self.qr_enabled_on_start:
            self.qr_detection_allowed = True
            self._set_qr_active(True, reason="startup")

    def _init_display_window(self):
        if not self.enable_display or self._display_window_ready:
            return

        cv2.namedWindow(
            self.display_window_name,
            cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL,
        )
        if self.display_window_width > 0 and self.display_window_height > 0:
            cv2.resizeWindow(
                self.display_window_name,
                self.display_window_width,
                self.display_window_height,
            )
        self._display_window_ready = True

    def _configure_capture(self, image_source: str):
        self.image_source = image_source
        self._stream_ready_logged = False
        self._stream_wait_warning_logged = False
        self._last_stream_error = ""
        self._startup_time = time.time()

        if image_source == "ros":
            self.capture = RosImageBuffer()
            self.get_logger().info("ROS2 raw image subscriber baslatiliyor...")
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.capture.on_image,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"Abone olunan ROS topic: {self.image_topic}")
            self.get_logger().info(
                "Not: Bu mod icin ros_gz_bridge parameter_bridge calisiyor olmali."
            )
            return

        self.capture = GzImageBuffer(self.gz_image_topic)
        self.capture.start()
        self.get_logger().info("Gazebo raw image subscriber baslatiliyor...")
        self.get_logger().info(
            f"Abone olunan Gazebo topic: {self.gz_image_topic}"
        )

    def _teardown_capture(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if hasattr(self, "capture"):
            self.capture.stop()

    def _switch_capture_source(self, image_source: str, topic: str):
        if image_source == "ros":
            self.image_topic = topic
        else:
            self.gz_image_topic = topic

        self._teardown_capture()
        self.get_logger().warning(
            f"Goruntu kaynagi {self.image_source} -> {image_source} olarak degistiriliyor."
        )
        self._configure_capture(image_source)

    def _reset_qr_runtime_state(self):
        self._qr_decode_reported = False
        self._qr_expected_qr_id = None
        self._qr_active_mission_index = None
        self._prev_gray = None
        self._tracked_qr = []
        self._visible_qr = []
        self._pending_qr = []
        self._last_fast_qr_result_version = -1
        self._last_decode_qr_result_version = -1
        self._last_heavy_qr_result_version = -1
        self._last_qr_refresh_time = 0.0
        self._last_qr_visible_time = 0.0
        self._last_full_qr_submit_frame = -QR_REDETECT_INTERVAL
        self._last_heavy_qr_submit_frame = -QR_HEAVY_FALLBACK_INTERVAL
        self._last_roi_qr_submit_frame = -1

    def _start_qr_threads(self):
        if all(
            thread is not None and thread.is_alive()
            for thread in (
                self.fast_qr_thread,
                self.decode_qr_thread,
                self.heavy_qr_thread,
            )
        ):
            return

        self.fast_qr_thread = QRScanThread(
            name="QRFastScanThread",
            use_wechat=False,
        )
        self.decode_qr_thread = QRScanThread(
            name="QRDecodeScanThread",
            use_wechat=True,
        )
        self.heavy_qr_thread = QRScanThread(
            name="QRHeavyScanThread",
            use_wechat=False,
        )
        self.fast_qr_thread.start()
        self.decode_qr_thread.start()
        self.heavy_qr_thread.start()

        if (
            self.decode_qr_thread.wechat_backend is not None
            and self.decode_qr_thread.wechat_backend.available
        ):
            self.get_logger().info("QR backend: WeChatQRCode + OpenCV fallback")
        else:
            self.get_logger().warning(
                "QR backend fallback: OpenCV QRCodeDetector "
                f"({self.decode_qr_thread.wechat_backend.last_error})"
            )

    def _stop_qr_threads(self):
        for attr in ("fast_qr_thread", "decode_qr_thread", "heavy_qr_thread"):
            thread = getattr(self, attr, None)
            if thread is not None:
                thread.stop()
                thread.join(timeout=0.5)
                setattr(self, attr, None)

    def _reset_color_runtime_state(self):
        self._onceki_renk_uzakliklari = {
            hedef["topic_type"]: None for hedef in COLOR_TARGETS
        }

    def _cache_recent_qr_overlay(self, qr_listesi, now):
        normalized_qr = self._normalize_qr_list(qr_listesi)
        if not normalized_qr:
            return

        self._recent_qr_overlay = normalized_qr
        self._recent_qr_overlay_until = now + QR_RESULT_OVERLAY_HOLD_SEC

    def _consume_recent_qr_overlay(self, now):
        if now > self._recent_qr_overlay_until:
            self._recent_qr_overlay = []
            return []

        return self._recent_qr_overlay

    def _build_local_message(self, message_type, payload):
        msg = String()
        msg.data = json.dumps(payload)
        return msg

    def _parse_local_message(self, msg, expected_type):
        data = json.loads(msg.data)
        message_type = data.get("message_type")
        if message_type is None:
            return data
        if (
            strip_udp_scoped_topic_prefix(str(message_type))
            != strip_udp_scoped_topic_prefix(expected_type)
        ):
            return None
        payload = data.get("payload", {})
        if not isinstance(payload, dict):
            raise ValueError("payload dict formatinda olmali")
        return payload

    def vision_control_callback(self, msg):
        try:
            data = self._parse_local_message(msg, self.vision_control_topic)
        except (json.JSONDecodeError, ValueError) as exc:
            self.get_logger().error(f"vision_control json parse hatasi: {exc}")
            return

        if data is None:
            return

        new_colored_field_detection_allowed = bool(
            data.get(
                "colored_field_detection_allowed",
                data.get(
                    "colored_field_enabled",
                    False,
                ),
            )
        )
        if "xy_error_enabled" in data:
            new_xy_error_enabled = bool(data.get("xy_error_enabled", False))
        else:
            new_xy_error_enabled = new_colored_field_detection_allowed

        new_qr_detection_allowed = bool(
            data.get("qr_detection_allowed", False)
        )
        new_target_color = data.get("target_color")
        if new_target_color is not None:
            new_target_color = str(new_target_color).strip().lower() or None

        if (
            new_colored_field_detection_allowed != self.colored_field_detection_allowed
            or new_xy_error_enabled != self.xy_error_enabled
            or new_qr_detection_allowed != self.qr_detection_allowed
            or new_target_color != self.target_color
        ):
            self._reset_color_runtime_state()
            self.get_logger().info(
                "Vision ayarlari guncellendi. "
                f"colored_field_detection_allowed={new_colored_field_detection_allowed}, "
                f"xy_error_enabled={new_xy_error_enabled}, "
                f"qr_detection_allowed={new_qr_detection_allowed}, "
                f"target_color={new_target_color}, udp_port={self.udp_port}"
            )

        self.colored_field_detection_allowed = new_colored_field_detection_allowed
        self.xy_error_enabled = new_xy_error_enabled
        self.qr_detection_allowed = new_qr_detection_allowed
        self.target_color = new_target_color

        if (
            self.qr_detection_allowed
            and not self.qr_active
            and not self._qr_decode_reported
        ):
            self._set_qr_active(
                True,
                reason="vision_control_allowed",
                mission_index=self._qr_active_mission_index,
                expected_qr_id=self._qr_expected_qr_id,
            )
        elif not self.qr_detection_allowed:
            if self.qr_active:
                self._set_qr_active(
                    False,
                    reason="vision_control_blocked",
                    mission_index=self._qr_active_mission_index,
                    expected_qr_id=self._qr_expected_qr_id,
                )
            else:
                self._qr_decode_reported = False

    def _set_qr_active(
        self,
        enabled: bool,
        reason="manual",
        mission_index=None,
        expected_qr_id=None,
    ):
        if enabled:
            if not self.qr_detection_allowed:
                self.get_logger().info(
                    "QR tarama istegi yoksayildi. "
                    f"qr_detection_allowed={self.qr_detection_allowed}, "
                    f"udp_port={self.udp_port}, reason={reason}"
                )
                return
            if self.qr_active:
                return

            self._reset_qr_runtime_state()
            self._qr_active_mission_index = mission_index
            self._qr_expected_qr_id = expected_qr_id
            self._start_qr_threads()
            self.qr_active = True
            self.get_logger().info(
                "QR tarama acildi. "
                f"reason={reason}, mission_index={mission_index}, "
                f"expected_qr_id={expected_qr_id}"
            )
            return

        self.qr_active = False
        self._reset_qr_runtime_state()
        if reason == "decoded":
            self._qr_decode_reported = True
        self._stop_qr_threads()
        self.get_logger().info(
            "QR tarama kapatildi. "
            f"reason={reason}, mission_index={mission_index}, "
            f"expected_qr_id={expected_qr_id}"
        )

    def _publish_qr_result(self, payload: str, timestamp_s: float, mission_plan: dict):
        msg = self._build_local_message(
            QR_RESULT_TOPIC,
            {
                "decoded": True,
                "payload": payload,
                "timestamp": timestamp_s,
                "mission_plan": mission_plan,
                "qr_id": mission_plan.get("qr_id"),
            },
        )
        self.qr_result_publisher.publish(msg)

    def _handle_decoded_qr(self, detected_qr, now):
        if not self.qr_active or self._qr_decode_reported:
            return

        decoded_qr = [
            qr for qr in self._normalize_qr_list(detected_qr)
            if qr.get("cozuldu", False)
        ]
        if decoded_qr:
            self._cache_recent_qr_overlay(decoded_qr, now)

        for qr in detected_qr:
            if not qr.get("cozuldu", False):
                continue

            payload = str(qr.get("payload", "")).strip()
            if not payload:
                self.get_logger().warning(
                    "QR cozuldu olarak isaretlendi ancak payload bos geldi; "
                    "sonuc yoksayiliyor."
                )
                continue
            normalized_payload, mission_plan = _resolve_mission_plan(payload)
            if mission_plan is None:
                self.get_logger().warning(
                    "QR cozuldu ancak eslesen gorev plani bulunamadi. "
                    f"payload='{normalized_payload}'"
                )
                continue

            self._qr_decode_reported = True
            self._publish_qr_result(normalized_payload, now, mission_plan)
            self.get_logger().info(
                f"QR decode basarili. payload='{normalized_payload}', "
                f"QR {mission_plan['qr_id']}"
            )
            mission_index = self._qr_active_mission_index
            expected_qr_id = self._qr_expected_qr_id
            self.qr_detection_allowed = False
            self._set_qr_active(
                False,
                reason="decoded",
                mission_index=mission_index,
                expected_qr_id=expected_qr_id,
            )
            return

    def _shutdown(self):
        if rclpy.ok():
            rclpy.shutdown()

    def destroy_node(self):
        self._cleanup()
        return super().destroy_node()

    def _cleanup(self):
        if self._cleanup_done:
            return
        self._cleanup_done = True
        self._teardown_capture()
        self.qr_active = False
        self._stop_qr_threads()
        cv2.destroyAllWindows()
        print("\n[BILGI] Sistem durduruldu.")

    def _stream_status(self):
        if self.capture.received_frames > 0:
            if not self._stream_ready_logged:
                self.get_logger().info(
                    f"{self.image_source.upper()} image akisi gelmeye basladi."
                )
                self._stream_ready_logged = True
            return

        if self.capture.last_error:
            if self.capture.last_error != self._last_stream_error:
                self.get_logger().warning(
                    f"Image donusum hatasi: {self.capture.last_error}"
                )
                self._last_stream_error = self.capture.last_error

        for thread in (
            getattr(self, "fast_qr_thread", None),
            getattr(self, "decode_qr_thread", None),
            getattr(self, "heavy_qr_thread", None),
        ):
            if thread is not None and thread.last_error:
                self.get_logger().warning(
                    f"{thread.name} hatasi: {thread.last_error}"
                )
                thread.last_error = ""

        if (
            not self._stream_wait_warning_logged
            and time.time() - self._startup_time >= 5.0
        ):
            discovered_ros_topic = discover_ros_image_topic(self) or ""
            discovered_gz_topic = discover_gz_image_topic() or ""

            if self._auto_source_requested:
                if self.image_source == "ros" and discovered_gz_topic:
                    self._switch_capture_source("gz", discovered_gz_topic)
                    return
                if self.image_source == "gz" and discovered_ros_topic:
                    self._switch_capture_source("ros", discovered_ros_topic)
                    return

            if self.image_source == "ros":
                self.get_logger().warning(
                    "Henuz hic frame gelmedi. ros_gz_bridge ve ROS image topic adini kontrol edin."
                )
                self.get_logger().warning(
                    f"Beklenen ROS topic: {self.image_topic}"
                )
            else:
                if (
                    discovered_gz_topic
                    and discovered_gz_topic != self.gz_image_topic
                ):
                    self.get_logger().warning(
                        "Frame gelmeyen topic yeniden eslestiriliyor."
                    )
                    self.get_logger().warning(
                        f"Eski Gazebo topic: {self.gz_image_topic}"
                    )
                    self.gz_image_topic = discovered_gz_topic
                    self.capture.restart(self.gz_image_topic)
                    self.get_logger().warning(
                        f"Yeni Gazebo topic: {self.gz_image_topic}"
                    )
                self.get_logger().warning(
                    "Henuz hic frame gelmedi. Gazebo image topic adini kontrol edin."
                )
                self.get_logger().warning(
                    f"Beklenen Gazebo topic: {self.gz_image_topic}"
                )
            self._stream_wait_warning_logged = True

    def _daire_konturu_gecerli_mi(self, cnt, min_contour_area=None):
        if min_contour_area is None:
            min_contour_area = self.min_contour_area
        alan = cv2.contourArea(cnt)
        if alan <= min_contour_area:
            return False

        cevre = cv2.arcLength(cnt, True)
        if cevre <= 0:
            return False

        dairesellik = 4 * np.pi * alan / (cevre * cevre)
        x, y, w, h = cv2.boundingRect(cnt)
        if h <= 0:
            return False

        oran = float(w) / h
        return (
            CIRCLE_MIN_CIRCULARITY < dairesellik < CIRCLE_MAX_CIRCULARITY
            and CIRCLE_MIN_ASPECT_RATIO < oran < CIRCLE_MAX_ASPECT_RATIO
        )

    def _gecerli_daire_konturlari(self, maske, min_contour_area=None):
        contours, _ = cv2.findContours(
            maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        return [
            cnt
            for cnt in contours
            if self._daire_konturu_gecerli_mi(
                cnt,
                min_contour_area=min_contour_area,
            )
        ]

    def _renk_hedeflerini_isle(
        self,
        frame,
        konturlar,
        cam_cx,
        cam_cy,
        renk_adi,
        renk_tipi,
        bgr,
        scale_x=1.0,
        scale_y=1.0,
    ):
        timestamp_s = time.time()
        image_height, image_width = frame.shape[:2]
        if not konturlar:
            self._onceki_renk_uzakliklari[renk_tipi] = None
            if self.xy_error_enabled:
                msg = self._build_local_message(
                    X_Y_ERROR_TOPIC,
                    {
                        "type": renk_tipi,
                        "detected": False,
                        "x": 0.0,
                        "y": 0.0,
                        "timestamp": timestamp_s,
                        "image_width": image_width,
                        "image_height": image_height,
                    },
                )
                self.x_y_error_publisher.publish(msg)
            return 0

        olcekli_konturlar = []
        for cnt in konturlar:
            if scale_x != 1.0 or scale_y != 1.0:
                olcekli = cnt.astype(np.float32).copy()
                olcekli[:, 0, 0] *= scale_x
                olcekli[:, 0, 1] *= scale_y
                cnt = np.round(olcekli).astype(np.int32)
            olcekli_konturlar.append(cnt)

        for cnt in olcekli_konturlar:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
            cv2.putText(
                frame, renk_adi, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.8, bgr, 3,
            )

        cnt = max(olcekli_konturlar, key=cv2.contourArea)
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        M = cv2.moments(cnt)
        if M["m00"] <= 0:
            self._onceki_renk_uzakliklari[renk_tipi] = None
            if self.xy_error_enabled:
                msg = self._build_local_message(
                    X_Y_ERROR_TOPIC,
                    {
                        "type": renk_tipi,
                        "detected": False,
                        "x": 0.0,
                        "y": 0.0,
                        "timestamp": timestamp_s,
                        "image_width": image_width,
                        "image_height": image_height,
                    },
                )
                self.x_y_error_publisher.publish(msg)
            return len(olcekli_konturlar)
        hx = int(M["m10"] / M["m00"])
        hy = int(M["m01"] / M["m00"])
        dx, dy = hx - cam_cx, hy - cam_cy
        contour_area_px = float(cv2.contourArea(cnt))
        if self.xy_error_enabled:
            msg = self._build_local_message(
                X_Y_ERROR_TOPIC,
                {
                    "type": renk_tipi,
                    "detected": True,
                    "x": float(dx),
                    "y": float(dy),
                    "timestamp": timestamp_s,
                    "contour_area_px": contour_area_px,
                    "radius_px": float(radius),
                    "center_x": float(hx),
                    "center_y": float(hy),
                    "image_width": image_width,
                    "image_height": image_height,
                },
            )
            self.x_y_error_publisher.publish(msg)
        cv2.circle(frame, (int(cx), int(cy)), int(radius), bgr, 2)
        cv2.circle(frame, (hx, hy), 5, bgr, -1)
        cv2.line(frame, (cam_cx, cam_cy), (hx, hy), bgr, 2)
        cv2.putText(
            frame,
            f"{renk_adi} Offset: dx={dx}, dy={dy}",
            (hx - 60, hy + int(radius) + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 1.6, bgr, 2,
        )

        mevcut_uzaklik = {"dx": abs(dx), "dy": abs(dy)}
        onceki_uzaklik = self._onceki_renk_uzakliklari.get(renk_tipi)
        self._onceki_renk_uzakliklari[renk_tipi] = mevcut_uzaklik

        if (
            self.colored_field_detection_allowed
            and
            onceki_uzaklik is not None
            and mevcut_uzaklik["dx"] <= onceki_uzaklik["dx"]
            and mevcut_uzaklik["dy"] <= onceki_uzaklik["dy"]
            and (
                mevcut_uzaklik["dx"] < onceki_uzaklik["dx"]
                or mevcut_uzaklik["dy"] < onceki_uzaklik["dy"]
            )
        ):
            msg = self._build_local_message(
                COLORED_FIELD_TOPIC,
                {
                    "type": renk_tipi,
                    "detected": True,
                    "timestamp": timestamp_s,
                    "contour_area_px": contour_area_px,
                    "radius_px": float(radius),
                    "center_x": float(hx),
                    "center_y": float(hy),
                    "x": float(dx),
                    "y": float(dy),
                    "image_width": image_width,
                    "image_height": image_height,
                },
            )
            self.colored_field_publisher.publish(msg)

        return len(olcekli_konturlar)

    def _durum_etiketi_ciz(
        self,
        frame,
        metin,
        x,
        y,
        text_color,
        bg_color=(20, 20, 20),
        border_color=None,
        alpha=0.7,
    ):
        font = cv2.FONT_HERSHEY_DUPLEX
        font_scale = 1
        thickness = 1
        pad_x = 12
        pad_y = 8
        gap_y = 10

        (tw, th), baseline = cv2.getTextSize(metin, font, font_scale, thickness)
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(frame.shape[1], x0 + tw + pad_x * 2)
        y1 = min(frame.shape[0], y0 + th + pad_y * 2 + baseline)
        if x0 >= x1 or y0 >= y1:
            return y

        panel = frame[y0:y1, x0:x1]
        shade = np.full_like(panel, bg_color, dtype=np.uint8)
        cv2.addWeighted(shade, alpha, panel, 1.0 - alpha, 0.0, dst=panel)

        if border_color is not None:
            cv2.rectangle(frame, (x0, y0), (x1, y1), border_color, 1, cv2.LINE_AA)

        text_org = (x0 + pad_x, y0 + pad_y + th)
        cv2.putText(
            frame,
            metin,
            (text_org[0] + 1, text_org[1] + 1),
            font,
            font_scale,
            (0, 0, 0),
            thickness + 1,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            metin,
            text_org,
            font,
            font_scale,
            text_color,
            thickness,
            cv2.LINE_AA,
        )
        return y1 + gap_y

    def _qr_ciz(self, frame, qr_listesi):
        for qr in qr_listesi:
            pts = _qr_cizim_noktalari(qr["noktalar"])
            mesafe = qr["mesafe"]
            cozuldu = qr.get("cozuldu", False)
            payload = str(qr.get("payload", "")).strip()
            if pts is None:
                continue

            pts_int = np.round(pts).astype(int)

            # Yesil cerceve
            cv2.polylines(frame, [pts_int], True, (0, 255, 0), 3, cv2.LINE_AA)

            x_min = int(pts_int[:, 0].min())
            y_min = int(pts_int[:, 1].min())

            etiket = "QR COZUMLENDI" if cozuldu else "QR ALGILANDI"
            if cozuldu and payload:
                etiket = f"{etiket}: {payload}"
            (tw, th), _ = cv2.getTextSize(
                etiket, cv2.FONT_HERSHEY_SIMPLEX, 1.125, 2
            )
            cv2.rectangle(
                frame,
                (x_min, y_min - th - 14),
                (x_min + tw + 10, y_min),
                (0, 180, 0), -1,
            )
            cv2.putText(
                frame, etiket,
                (x_min + 5, y_min - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 1.125,
                (255, 255, 255), 2, cv2.LINE_AA,
            )

    def _normalize_qr_list(self, qr_listesi):
        normalized = []
        for qr in qr_listesi:
            pts = _sirali_qr_noktalari(qr.get("noktalar"))
            if pts is None:
                continue
            payload = str(qr.get("payload", "")).strip()
            normalized.append(
                {
                    "mesafe": qr.get("mesafe", "?"),
                    "noktalar": pts[:4].copy(),
                    "cozuldu": bool(qr.get("cozuldu", False) and payload),
                    "payload": payload,
                }
            )
        return normalized

    def _qr_benzer_mi(self, qr_a, qr_b):
        pts_a = np.asarray(qr_a["noktalar"], dtype=np.float32).reshape(-1, 2)
        pts_b = np.asarray(qr_b["noktalar"], dtype=np.float32).reshape(-1, 2)
        merkez_a = pts_a.mean(axis=0)
        merkez_b = pts_b.mean(axis=0)
        alan_a = abs(cv2.contourArea(pts_a))
        alan_b = abs(cv2.contourArea(pts_b))
        return (
            np.linalg.norm(merkez_a - merkez_b) < 24.0
            and abs(alan_a - alan_b) / max(alan_a, alan_b, 1.0) < 0.4
        )

    def _guncelle_bekleyen_qr(self, qr_listesi, now):
        aktif_bekleyen = []
        for aday in self._pending_qr:
            if now - aday["last_seen"] <= QR_PENDING_TIMEOUT_SEC:
                aktif_bekleyen.append(aday)
        self._pending_qr = aktif_bekleyen

        onayli = []
        for qr in qr_listesi:
            eslesen = None
            for aday in self._pending_qr:
                if self._qr_benzer_mi(aday["qr"], qr):
                    eslesen = aday
                    break

            if eslesen is None:
                yeni_aday = {
                    "qr": qr,
                    "hits": 1,
                    "last_seen": now,
                }
                if QR_PENDING_CONFIRMATIONS <= 1:
                    onayli.append(qr)
                else:
                    self._pending_qr.append(yeni_aday)
                continue

            eslesen["qr"] = qr
            eslesen["hits"] += 1
            eslesen["last_seen"] = now
            if eslesen["hits"] >= QR_PENDING_CONFIRMATIONS:
                onayli.append(qr)

        if onayli:
            self._pending_qr = [
                aday
                for aday in self._pending_qr
                if all(not self._qr_benzer_mi(aday["qr"], qr) for qr in onayli)
            ]

        return onayli

    def _roi_eslesmelerini_onayla(self, qr_listesi):
        if not self._tracked_qr:
            return []

        onayli = []
        for qr in qr_listesi:
            if any(self._qr_benzer_mi(qr, aktif_qr) for aktif_qr in self._tracked_qr):
                onayli.append(qr)
        return onayli

    def _yumusat_tespitleri(self, qr_listesi):
        if not self._tracked_qr:
            return qr_listesi

        yumusatilmis = []
        for qr in qr_listesi:
            eslesen = None
            for aktif_qr in self._tracked_qr:
                if self._qr_benzer_mi(qr, aktif_qr):
                    eslesen = aktif_qr
                    break

            if eslesen is None:
                yumusatilmis.append(qr)
                continue

            eski = np.asarray(eslesen["noktalar"], dtype=np.float32)
            yeni = np.asarray(qr["noktalar"], dtype=np.float32)
            karisik = (eski * QR_DETECTION_SMOOTHING) + (
                yeni * (1.0 - QR_DETECTION_SMOOTHING)
            )
            yumusatilmis.append(
                {
                    "mesafe": qr["mesafe"],
                    "noktalar": _sirali_qr_noktalari(karisik),
                    "cozuldu": qr.get("cozuldu", False) or eslesen.get("cozuldu", False),
                    "payload": str(qr.get("payload", "") or eslesen.get("payload", "")),
                }
            )

        return [qr for qr in yumusatilmis if qr["noktalar"] is not None]

    def _track_qr_list(self, prev_gray, gray_frame, qr_listesi):
        if prev_gray is None or not qr_listesi:
            return qr_listesi

        all_points = np.concatenate(
            [qr["noktalar"] for qr in qr_listesi], axis=0
        ).reshape(-1, 1, 2)
        next_points, status, _ = cv2.calcOpticalFlowPyrLK(
            prev_gray,
            gray_frame,
            all_points,
            None,
            winSize=(21, 21),
            maxLevel=2,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                20,
                0.03,
            ),
        )

        if next_points is None or status is None:
            return []

        h, w = gray_frame.shape[:2]
        tracked = []
        point_index = 0

        for qr in qr_listesi:
            point_count = len(qr["noktalar"])
            onceki_qr_noktalari = np.asarray(qr["noktalar"], dtype=np.float32).reshape(-1, 2)
            qr_points = next_points[point_index : point_index + point_count].reshape(
                -1, 2
            )
            qr_status = status[point_index : point_index + point_count].reshape(-1)
            point_index += point_count

            if np.count_nonzero(qr_status) != point_count:
                continue

            if (
                np.any(qr_points[:, 0] < 0)
                or np.any(qr_points[:, 0] >= w)
                or np.any(qr_points[:, 1] < 0)
                or np.any(qr_points[:, 1] >= h)
            ):
                continue

            if cv2.contourArea(qr_points.astype(np.float32)) < 80.0:
                continue

            qr_points = _sirali_qr_noktalari(qr_points)
            if qr_points is None:
                continue
            if not _qr_izleme_gecerli_mi(
                onceki_qr_noktalari,
                qr_points,
                gray_frame,
                qr.get("cozuldu", False),
            ):
                continue

            tracked.append(
                {
                    "mesafe": qr["mesafe"],
                    "noktalar": qr_points,
                    "cozuldu": qr.get("cozuldu", False),
                    "payload": str(qr.get("payload", "")),
                }
            )

        return tracked

    def _guncelle_qr_sonuclari(self, detected_qr, now):
        if not detected_qr:
            return

        normalized_qr = self._normalize_qr_list(detected_qr)
        if not normalized_qr:
            return

        self._visible_qr = self._yumusat_tespitleri(normalized_qr)
        self._last_qr_visible_time = now

        decoded_qr = [qr for qr in normalized_qr if qr.get("cozuldu", False)]
        if decoded_qr:
            self._pending_qr = []
            self._tracked_qr = self._yumusat_tespitleri(decoded_qr)
            self._last_qr_refresh_time = now
            return

        onayli_qr = self._roi_eslesmelerini_onayla(normalized_qr)
        if not onayli_qr:
            onayli_qr = self._guncelle_bekleyen_qr(normalized_qr, now)
        if onayli_qr:
            self._tracked_qr = self._yumusat_tespitleri(onayli_qr)
            self._last_qr_refresh_time = now

    def _ilerlet_qr_takibi(self, gray_frame, now):
        if self._prev_gray is None:
            return

        if self._visible_qr:
            self._visible_qr = self._track_qr_list(
                self._prev_gray,
                gray_frame,
                self._visible_qr,
            )
            if self._visible_qr:
                self._last_qr_visible_time = now
        elif self._tracked_qr:
            self._tracked_qr = self._track_qr_list(
                self._prev_gray,
                gray_frame,
                self._tracked_qr,
            )
            if self._tracked_qr:
                self._last_qr_refresh_time = now


    def process_frame(self):
        raw_frame = self.capture.get_frame()
        if raw_frame is None:
            return
        frame = raw_frame.copy()

        self._frame_count += 1

        self._fps_counter += 1
        now = time.time()
        if now - self._fps_t0 >= 1.0:
            self._fps_display = self._fps_counter
            self._fps_counter = 0
            self._fps_t0 = now

        h, w = frame.shape[:2]
        cam_cx, cam_cy = w // 2, h // 2

        cv2.line(frame, (cam_cx - 20, cam_cy), (cam_cx + 20, cam_cy), (0, 255, 0), 2)
        cv2.line(frame, (cam_cx, cam_cy - 20), (cam_cx, cam_cy + 20), (0, 255, 0), 2)

        circle_scale = (
            CIRCLE_PROCESS_SCALE
            if max(h, w) >= 640
            else 1.0
        )
        if circle_scale != 1.0:
            circle_size = (
                max(1, int(round(w * circle_scale))),
                max(1, int(round(h * circle_scale))),
            )
            circle_frame = cv2.resize(
                raw_frame,
                circle_size,
                interpolation=cv2.INTER_AREA,
            )
            scale_x = w / float(circle_size[0])
            scale_y = h / float(circle_size[1])
            circle_min_area = max(
                30.0,
                float(self.min_contour_area) * circle_scale * circle_scale,
            )
        else:
            circle_frame = raw_frame
            scale_x = 1.0
            scale_y = 1.0
            circle_min_area = float(self.min_contour_area)

        renk_sayaclari = {
            hedef["etiket"]: 0 for hedef in COLOR_TARGETS
        }
        if self.colored_field_detection_allowed or self.xy_error_enabled:
            hsv = cv2.cvtColor(circle_frame, cv2.COLOR_BGR2HSV)
            for hedef in COLOR_TARGETS:
                if (
                    self.target_color is not None
                    and hedef["topic_type"] != self.target_color
                ):
                    continue
                mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
                for alt, ust in zip(hedef["alt_hsv"], hedef["ust_hsv"]):
                    mask = cv2.bitwise_or(mask, cv2.inRange(hsv, alt, ust))
                mask = cv2.medianBlur(mask, 5)
                konturlar = self._gecerli_daire_konturlari(
                    mask,
                    min_contour_area=circle_min_area,
                )
                renk_sayaclari[hedef["etiket"]] = self._renk_hedeflerini_isle(
                    frame,
                    konturlar,
                    cam_cx,
                    cam_cy,
                    hedef["etiket"],
                    hedef["topic_type"],
                    hedef["bgr"],
                    scale_x=scale_x,
                    scale_y=scale_y,
                )

        bulunan_qr = []
        okunan_qr = []
        qr_str = "QR kapali"

        if self.qr_active:
            gray = cv2.cvtColor(raw_frame, cv2.COLOR_BGR2GRAY)
            self._ilerlet_qr_takibi(gray, now)

            qr_roi_kaynagi = self._visible_qr if self._visible_qr else self._tracked_qr
            roi_rect = _qr_roi_rect(gray, qr_roi_kaynagi)
            should_scan_roi = (
                roi_rect is not None
                and self._frame_count % QR_DECODE_RETRY_INTERVAL == 0
                and self._last_roi_qr_submit_frame != self._frame_count
            )
            should_scan_full_fast = (
                self._frame_count % QR_SCAN_INTERVAL == 0
                and not self._visible_qr
                and self._last_full_qr_submit_frame != self._frame_count
            )
            should_scan_full_heavy = (
                self._frame_count % QR_SCAN_INTERVAL == 0
                and not self._visible_qr
                and (
                    self._frame_count - self._last_heavy_qr_submit_frame
                    >= QR_HEAVY_FALLBACK_INTERVAL
                )
            )

            # QR kilitlenene kadar tam karede ucuz detect ayri thread'de,
            # decode/heavy ayri thread'lerde.
            if should_scan_roi and self.decode_qr_thread is not None:
                self.decode_qr_thread.submit(
                    raw_frame,
                    search_rect=roi_rect,
                    source="roi",
                    allow_detect_only=True,
                )
                self._last_roi_qr_submit_frame = self._frame_count
            if should_scan_full_heavy and self.heavy_qr_thread is not None:
                self.heavy_qr_thread.submit(
                    raw_frame,
                    source="full_heavy",
                    allow_detect_only=True,
                )
                self._last_heavy_qr_submit_frame = self._frame_count
            if should_scan_full_fast and self.fast_qr_thread is not None:
                self.fast_qr_thread.submit(
                    raw_frame,
                    source="full_fast",
                    allow_detect_only=True,
                )
                self._last_full_qr_submit_frame = self._frame_count

            if self.fast_qr_thread is not None:
                fast_result_version, fast_detected_qr = self.fast_qr_thread.get_result()
                if fast_result_version != self._last_fast_qr_result_version:
                    self._last_fast_qr_result_version = fast_result_version
                    self._handle_decoded_qr(fast_detected_qr, now)
                    if self.qr_active:
                        self._guncelle_qr_sonuclari(fast_detected_qr, now)

            if self.decode_qr_thread is not None:
                decode_result_version, decode_detected_qr = self.decode_qr_thread.get_result()
                if decode_result_version != self._last_decode_qr_result_version:
                    self._last_decode_qr_result_version = decode_result_version
                    self._handle_decoded_qr(decode_detected_qr, now)
                    if self.qr_active:
                        self._guncelle_qr_sonuclari(decode_detected_qr, now)

            if self.heavy_qr_thread is not None:
                heavy_result_version, heavy_detected_qr = self.heavy_qr_thread.get_result()
                if heavy_result_version != self._last_heavy_qr_result_version:
                    self._last_heavy_qr_result_version = heavy_result_version
                    self._handle_decoded_qr(heavy_detected_qr, now)
                    if self.qr_active:
                        self._guncelle_qr_sonuclari(heavy_detected_qr, now)

            if self.qr_active and now - self._last_qr_refresh_time > QR_TRACK_TIMEOUT_SEC:
                self._tracked_qr = []
                self._pending_qr = []

            if (
                self.qr_active
                and not self._tracked_qr
                and now - self._last_qr_visible_time > QR_TRACK_TIMEOUT_SEC
            ):
                self._visible_qr = []

            if self.qr_active:
                bulunan_qr = self._visible_qr if self._visible_qr else self._tracked_qr
                okunan_qr = [qr for qr in bulunan_qr if qr.get("cozuldu", False)]
                self._qr_ciz(frame, bulunan_qr)
                self._prev_gray = gray
                if bulunan_qr:
                    qr_str = (
                        f"QR Algilandi:{len(bulunan_qr)} | "
                        f"QR Cozumlendi:{len(okunan_qr)}"
                    )
                else:
                    qr_str = "QR yok"

        if not bulunan_qr:
            bulunan_qr = self._consume_recent_qr_overlay(now)
            if bulunan_qr:
                okunan_qr = [qr for qr in bulunan_qr if qr.get("cozuldu", False)]
                self._qr_ciz(frame, bulunan_qr)
                qr_str = (
                    f"QR Algilandi:{len(bulunan_qr)} | "
                    f"QR Cozumlendi:{len(okunan_qr)}"
                )

        # Üst-sol bilgi overlay
        bilgi_y = self._durum_etiketi_ciz(
            frame,
            f"FPS {self._fps_display:02d}",
            10,
            10,
            text_color=(255, 245, 210),
            bg_color=(28, 52, 68),
            border_color=(0, 220, 255),
        )
        if bulunan_qr:
            bilgi_y = self._durum_etiketi_ciz(
                frame,
                f"QR ALGILANDI {len(bulunan_qr)}",
                10,
                bilgi_y,
                text_color=(255, 240, 210),
                bg_color=(60, 52, 20),
                border_color=(0, 200, 255),
            )
        if okunan_qr:
            self._durum_etiketi_ciz(
                frame,
                f"QR COZUMLENDI {len(okunan_qr)}",
                10,
                bilgi_y,
                text_color=(230, 255, 230),
                bg_color=(20, 70, 30),
                border_color=(0, 255, 0),
            )

        if now - self._last_console_update_time >= CONSOLE_UPDATE_INTERVAL_SEC:
            print(
                f"\r[CANLI] FPS:{self._fps_display:3d} | "
                f"Kirmizi:{renk_sayaclari['KIRMIZI']} | "
                f"Mavi:{renk_sayaclari['MAVI']} | {qr_str}".ljust(80),
                end="", flush=True,
            )
            self._last_console_update_time = now

        if self.enable_display:
            self._init_display_window()
            cv2.imshow(self.display_window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("Cikis.")
                if rclpy.ok():
                    rclpy.shutdown()
            elif key == ord("s"):
                fname = f"frame_{int(time.time())}.png"
                cv2.imwrite(fname, frame)
                print(f"\n[BILGI] Kaydedildi: {fname}")


class CameraImageBuffer:
    def __init__(self, camera_index: int, width: int, height: int):
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.cap = cv2.VideoCapture(camera_index)
        self.received_frames = 0
        self.last_frame_time = 0.0
        self.last_error = ""

        if width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_frame(self):
        if not self.cap.isOpened():
            self.last_error = f"Kamera acik degil: index={self.camera_index}"
            return None

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.last_error = "Kameradan frame okunamadi"
            return None

        self.last_error = ""
        self.last_frame_time = time.time()
        self.received_frames += 1
        return frame

    def stop(self):
        if self.cap is not None:
            self.cap.release()


class AdaptedImageProcessingNode(Node):
    """Yeni goruntu isleme algoritmasi + mevcut ucus/Gazebo sozlesmesi."""

    def __init__(self):
        super().__init__("image_processing_node")

        self.declare_parameter("udp_port", 14540)
        self.declare_parameter("image_source", "auto")
        self.declare_parameter("image_topic", "")
        self.declare_parameter("gz_image_topic", "")
        self.declare_parameter("auto_discover_topic", True)
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_width", 1080)
        self.declare_parameter("camera_height", 1920)
        self.declare_parameter("enable_display", True)
        self.declare_parameter("show_masks", False)
        self.declare_parameter("display_window_name", "WeChat QR Okuyucu")
        self.declare_parameter("display_window_width", DISPLAY_WINDOW_DEFAULT_WIDTH)
        self.declare_parameter("display_window_height", DISPLAY_WINDOW_DEFAULT_HEIGHT)
        self.declare_parameter("display_width", 0)
        self.declare_parameter("display_height", 0)
        self.declare_parameter("processing_hz", 30.0)
        self.declare_parameter("qr_enabled_on_start", False)
        self.declare_parameter("min_contour_area", 1200)
        self.declare_parameter("min_colored_circle_area", 1200)
        self.declare_parameter("min_colored_circle_radius", 18.0)
        self.declare_parameter("min_colored_fill_ratio", 0.45)

        self.udp_port = int(self.get_parameter("udp_port").value)
        self.image_source = str(self.get_parameter("image_source").value).strip().lower()
        self.image_topic = str(self.get_parameter("image_topic").value).strip()
        self.gz_image_topic = str(self.get_parameter("gz_image_topic").value).strip()
        self.auto_discover_topic = bool(self.get_parameter("auto_discover_topic").value)
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.enable_display = bool(self.get_parameter("enable_display").value)
        self.show_masks = bool(self.get_parameter("show_masks").value)
        self.display_window_name = str(self.get_parameter("display_window_name").value)
        self.display_width = int(self.get_parameter("display_width").value)
        self.display_height = int(self.get_parameter("display_height").value)
        if self.display_width <= 0:
            self.display_width = int(self.get_parameter("display_window_width").value)
        if self.display_height <= 0:
            self.display_height = int(self.get_parameter("display_window_height").value)
        self.processing_hz = float(self.get_parameter("processing_hz").value)
        self.qr_enabled_on_start = bool(
            self.get_parameter("qr_enabled_on_start").value
        )
        legacy_min_contour_area = int(self.get_parameter("min_contour_area").value)
        configured_min_colored_area = int(
            self.get_parameter("min_colored_circle_area").value
        )
        self.min_colored_circle_area = (
            configured_min_colored_area
            if configured_min_colored_area != 1200
            else legacy_min_contour_area
        )
        self.min_colored_circle_radius = float(
            self.get_parameter("min_colored_circle_radius").value
        )
        self.min_colored_fill_ratio = float(
            self.get_parameter("min_colored_fill_ratio").value
        )

        if self.processing_hz <= 0.0:
            self.get_logger().warning(
                f"Gecersiz processing_hz={self.processing_hz}, 30.0 kullanilacak."
            )
            self.processing_hz = 30.0

        self.vision_control_topic = f"udp_{self.udp_port}_{VISION_CONTROL_TOPIC}"
        self.colored_field_topic = f"udp_{self.udp_port}_{COLORED_FIELD_TOPIC}"
        self.x_y_error_topic = f"udp_{self.udp_port}_{X_Y_ERROR_TOPIC}"
        self.qr_result_topic = f"udp_{self.udp_port}_{QR_RESULT_TOPIC}"

        self.qr_detection_allowed = bool(self.qr_enabled_on_start)
        self.colored_field_detection_allowed = False
        self.xy_error_enabled = False
        self.target_color = None
        self.last_sent_qr = set()
        self._onceki_renk_uzakliklari = {"red": None, "blue": None}
        self._display_window_ready = False
        self._last_stream_warning_time = 0.0
        self._stream_ready_logged = False
        self.image_sub = None
        self.capture = None

        self.qr_detector = None
        self.classic_qr_detector = cv2.QRCodeDetector()
        self._init_qr_detector()

        self._configure_capture(self._resolve_image_source())

        self.vision_control_subscriber = self.create_subscription(
            String,
            self.vision_control_topic,
            self.vision_control_callback,
            10,
        )
        self.colored_field_publisher = self.create_publisher(
            String,
            self.colored_field_topic,
            10,
        )
        self.x_y_error_publisher = self.create_publisher(
            String,
            self.x_y_error_topic,
            10,
        )
        self.qr_result_publisher = self.create_publisher(
            String,
            self.qr_result_topic,
            10,
        )

        self.frame_timer = self.create_timer(
            1.0 / self.processing_hz,
            self.process_frame,
        )
        self.get_logger().info(
            "Yeni goruntu isleme uyarlamasi basladi. "
            f"source={self.image_source}, udp_port={self.udp_port}, "
            f"vision={self.vision_control_topic}, "
            f"qr={self.qr_result_topic}, color={self.colored_field_topic}, "
            f"xy={self.x_y_error_topic}"
        )

    def _init_qr_detector(self):
        if not hasattr(cv2, "wechat_qrcode_WeChatQRCode"):
            self.get_logger().warning(
                "OpenCV WeChatQRCode yok; klasik QRCodeDetector fallback kullanilacak."
            )
            return

        paths = [
            WECHAT_QR_MODEL_DIR / "detect.prototxt",
            WECHAT_QR_MODEL_DIR / "detect.caffemodel",
            WECHAT_QR_MODEL_DIR / "sr.prototxt",
            WECHAT_QR_MODEL_DIR / "sr.caffemodel",
        ]
        missing = [str(path) for path in paths if not path.exists()]
        if missing:
            self.get_logger().warning(
                "WeChat QR model dosyalari eksik; fallback kullanilacak: "
                f"{missing}"
            )
            return

        try:
            self.qr_detector = cv2.wechat_qrcode_WeChatQRCode(
                *(str(path) for path in paths)
            )
            self.get_logger().info("QR backend: WeChatQRCode")
        except Exception as exc:
            self.qr_detector = None
            self.get_logger().warning(
                f"WeChatQRCode baslatilamadi; fallback kullanilacak: {exc}"
            )

    def _resolve_image_source(self):
        requested_source = self.image_source
        if requested_source not in {"auto", "camera", "ros", "gz"}:
            self.get_logger().warning(
                f"Gecersiz image_source={requested_source}, auto kullanilacak."
            )
            requested_source = "auto"

        discovered_ros_topic = ""
        discovered_gz_topic = ""
        if self.auto_discover_topic:
            discovered_ros_topic = discover_ros_image_topic(self) or ""
            discovered_gz_topic = discover_gz_image_topic() or ""

        configured_gz_topic = self.gz_image_topic

        if not self.image_topic:
            self.image_topic = discovered_ros_topic

        if requested_source == "auto":
            if self.image_topic:
                return "ros"
            if discovered_gz_topic or configured_gz_topic:
                self.gz_image_topic = (
                    configured_gz_topic
                    or discovered_gz_topic
                    or DEFAULT_GZ_IMAGE_TOPIC
                )
                return "gz"
            return "camera"

        if requested_source == "gz" and not self.gz_image_topic:
            self.gz_image_topic = discovered_gz_topic or DEFAULT_GZ_IMAGE_TOPIC
        return requested_source

    def _configure_capture(self, source):
        self.image_source = source
        self._stream_ready_logged = False
        self._last_stream_warning_time = 0.0

        if source == "ros":
            self.capture = RosImageBuffer()
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.capture.on_image,
                qos_profile_sensor_data,
            )
            self.get_logger().info(f"ROS image topic dinleniyor: {self.image_topic}")
            return

        if source == "gz":
            self.capture = GzImageBuffer(self.gz_image_topic)
            self.capture.start()
            self.get_logger().info(f"Gazebo image topic dinleniyor: {self.gz_image_topic}")
            return

        self.capture = CameraImageBuffer(
            self.camera_index,
            self.camera_width,
            self.camera_height,
        )
        self.get_logger().info(
            f"Fiziksel kamera aciliyor: index={self.camera_index}, "
            f"istenen={self.camera_width}x{self.camera_height}"
        )

    def _teardown_capture(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
        if self.capture is not None:
            self.capture.stop()
            self.capture = None

    def _parse_control_message(self, msg):
        data = json.loads(msg.data)
        if not isinstance(data, dict):
            raise ValueError("vision_control payload dict olmali")

        message_type = data.get("message_type")
        if message_type is None:
            return data

        if (
            strip_udp_scoped_topic_prefix(str(message_type))
            != strip_udp_scoped_topic_prefix(self.vision_control_topic)
        ):
            return None

        payload = data.get("payload", {})
        if not isinstance(payload, dict):
            raise ValueError("vision_control envelope payload dict olmali")
        return payload

    def vision_control_callback(self, msg):
        try:
            data = self._parse_control_message(msg)
        except Exception as exc:
            self.get_logger().error(f"Vision control JSON parse hatasi: {exc}")
            return

        if data is None:
            return

        new_qr_allowed = bool(data.get("qr_detection_allowed", False))
        new_colored_allowed = bool(
            data.get(
                "colored_field_detection_allowed",
                data.get("colored_field_enabled", False),
            )
        )
        new_xy_enabled = bool(data.get("xy_error_enabled", new_colored_allowed))
        new_target_color = data.get("target_color", None)
        if new_target_color is not None:
            new_target_color = str(new_target_color).strip().lower() or None
        if new_target_color not in ("red", "blue", None):
            self.get_logger().warning(
                f"Gecersiz target_color geldi: {new_target_color}"
            )
            new_target_color = None

        if new_qr_allowed and not self.qr_detection_allowed:
            self.last_sent_qr.clear()

        changed = (
            new_qr_allowed != self.qr_detection_allowed
            or new_colored_allowed != self.colored_field_detection_allowed
            or new_xy_enabled != self.xy_error_enabled
            or new_target_color != self.target_color
        )
        color_tracking_changed = (
            new_colored_allowed != self.colored_field_detection_allowed
            or new_target_color != self.target_color
        )

        self.qr_detection_allowed = new_qr_allowed
        self.colored_field_detection_allowed = new_colored_allowed
        self.xy_error_enabled = new_xy_enabled
        self.target_color = new_target_color

        if color_tracking_changed:
            self._reset_color_approach_state()

        if changed:
            self.get_logger().info(
                "Vision control alindi | "
                f"qr={self.qr_detection_allowed}, "
                f"colored={self.colored_field_detection_allowed}, "
                f"xy={self.xy_error_enabled}, "
                f"target_color={self.target_color}"
            )

    def publish_json(self, publisher, data):
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        publisher.publish(msg)

    def _reset_color_approach_state(self, renk_tipi=None):
        if renk_tipi in ("red", "blue"):
            self._onceki_renk_uzakliklari[renk_tipi] = None
            return

        self._onceki_renk_uzakliklari = {"red": None, "blue": None}

    def _is_colored_field_approaching(self, frame, detected_circle):
        renk_tipi = detected_circle.get("type")
        if renk_tipi not in ("red", "blue"):
            return False

        frame_h, frame_w = frame.shape[:2]
        dx = abs(float(detected_circle["center_x"]) - (frame_w / 2.0))
        dy = abs(float(detected_circle["center_y"]) - (frame_h / 2.0))
        mevcut_uzaklik = {"dx": dx, "dy": dy}
        onceki_uzaklik = self._onceki_renk_uzakliklari.get(renk_tipi)
        self._onceki_renk_uzakliklari[renk_tipi] = mevcut_uzaklik

        if onceki_uzaklik is None:
            return False

        return (
            mevcut_uzaklik["dx"] <= onceki_uzaklik["dx"]
            and mevcut_uzaklik["dy"] <= onceki_uzaklik["dy"]
            and (
                mevcut_uzaklik["dx"] < onceki_uzaklik["dx"]
                or mevcut_uzaklik["dy"] < onceki_uzaklik["dy"]
            )
        )

    def publish_qr_result(self, text):
        payload = str(text).strip()
        if not payload:
            return

        mission_plan = None
        normalized_payload = payload
        try:
            parsed_payload = json.loads(payload)
        except Exception:
            parsed_payload = None

        if isinstance(parsed_payload, dict):
            mission_plan = parsed_payload
        else:
            normalized_payload, mission_plan = _resolve_mission_plan(payload)

        if mission_plan is None:
            self.get_logger().warning(
                f"QR cozuldu ancak gorev plani bulunamadi: payload='{payload}'"
            )
            return

        data = {
            "decoded": True,
            "payload": normalized_payload,
            "timestamp": time.time(),
            "mission_plan": mission_plan,
            "qr_id": mission_plan.get("qr_id"),
        }
        self.publish_json(self.qr_result_publisher, data)
        self.get_logger().info(
            f"QR sonucu yayinlandi: payload='{normalized_payload}', "
            f"qr_id={mission_plan.get('qr_id')}"
        )

    def publish_colored_field_result(self, detected_circle):
        data = {
            "type": detected_circle["type"],
            "detected": True,
            "timestamp": time.time(),
            "center_x": float(detected_circle["center_x"]),
            "center_y": float(detected_circle["center_y"]),
            "radius_px": float(detected_circle["radius"]),
            "contour_area_px": float(detected_circle["area"]),
        }
        self.publish_json(self.colored_field_publisher, data)

    def publish_x_y_error_result(self, frame, detected_circle):
        frame_h, frame_w = frame.shape[:2]
        image_center_x = frame_w / 2.0
        image_center_y = frame_h / 2.0
        target_x = float(detected_circle["center_x"])
        target_y = float(detected_circle["center_y"])

        data = {
            "x": float(target_x - image_center_x),
            "y": float(target_y - image_center_y),
            "detected": True,
            "type": detected_circle["type"],
            "timestamp": time.time(),
            "center_x": target_x,
            "center_y": target_y,
            "radius_px": float(detected_circle["radius"]),
            "contour_area_px": float(detected_circle["area"]),
            "image_width": frame_w,
            "image_height": frame_h,
        }
        self.publish_json(self.x_y_error_publisher, data)

    def publish_no_detection(self):
        data = {
            "x": 0.0,
            "y": 0.0,
            "detected": False,
            "type": self.target_color if self.target_color is not None else "",
            "timestamp": time.time(),
        }
        self.publish_json(self.x_y_error_publisher, data)

    def hue_mesafesi(self, h1, h2):
        fark = abs(float(h1) - float(h2))
        return min(fark, 180.0 - fark)

    def hedef_renk_hsv_maskesi(self, hsv, hedef_renk):
        if hedef_renk == "red":
            alt_kirmizi_1 = np.array([0, 90, 70])
            ust_kirmizi_1 = np.array([12, 255, 255])
            alt_kirmizi_2 = np.array([168, 90, 70])
            ust_kirmizi_2 = np.array([180, 255, 255])
            maske_kirmizi_1 = cv2.inRange(hsv, alt_kirmizi_1, ust_kirmizi_1)
            maske_kirmizi_2 = cv2.inRange(hsv, alt_kirmizi_2, ust_kirmizi_2)
            return cv2.bitwise_or(maske_kirmizi_1, maske_kirmizi_2)

        if hedef_renk == "blue":
            alt_mavi = np.array([95, 80, 50])
            ust_mavi = np.array([135, 255, 255])
            return cv2.inRange(hsv, alt_mavi, ust_mavi)

        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    def kontur_renk_dogrula(self, frame, contour, renk_tipi):
        kontur_maskesi = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(kontur_maskesi, [contour], -1, 255, -1)

        kontur_pixel_sayisi = cv2.countNonZero(kontur_maskesi)
        if kontur_pixel_sayisi == 0:
            return False

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hedef_maskesi = self.hedef_renk_hsv_maskesi(hsv, renk_tipi)
        dogru_renk_maskesi = cv2.bitwise_and(hedef_maskesi, kontur_maskesi)

        dogru_renk_pixel_sayisi = cv2.countNonZero(dogru_renk_maskesi)
        renk_doluluk_orani = dogru_renk_pixel_sayisi / float(kontur_pixel_sayisi)
        if renk_doluluk_orani < self.min_colored_fill_ratio:
            return False

        renkli_pixeller = frame[dogru_renk_maskesi > 0]
        if len(renkli_pixeller) == 0:
            return False

        ort_b, ort_g, ort_r = np.mean(renkli_pixeller, axis=0)
        if renk_tipi == "red":
            return ort_r > ort_g + 45 and ort_r > ort_b + 55
        if renk_tipi == "blue":
            return ort_b > ort_g + 25 and ort_b > ort_r + 45
        return False

    def kmeans_renk_maskesi_uret(self, frame, hedef_renk, k=4):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        frame_h, frame_w = hsv.shape[:2]
        kmeans_width = 320
        oran = kmeans_width / float(frame_w)
        kmeans_height = max(1, int(frame_h * oran))
        kucuk_hsv = cv2.resize(hsv, (kmeans_width, kmeans_height))
        pixels = np.float32(kucuk_hsv.reshape((-1, 3)))
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            20,
            1.0,
        )
        _, labels, centers = cv2.kmeans(
            pixels,
            k,
            None,
            criteria,
            3,
            cv2.KMEANS_PP_CENTERS,
        )

        centers = np.uint8(centers)
        secilen_kumeler = []
        for index, center in enumerate(centers):
            h, s, v = center
            if s < 80 or v < 50:
                continue
            if hedef_renk == "red":
                kirmiziye_uzaklik = min(
                    self.hue_mesafesi(h, 0),
                    self.hue_mesafesi(h, 180),
                )
                if kirmiziye_uzaklik <= 14:
                    secilen_kumeler.append(index)
            elif hedef_renk == "blue":
                maviye_uzaklik = self.hue_mesafesi(h, 110)
                if maviye_uzaklik <= 25:
                    secilen_kumeler.append(index)

        labels_2d = labels.reshape((kmeans_height, kmeans_width))
        kucuk_maske = np.zeros((kmeans_height, kmeans_width), dtype=np.uint8)
        for kume_index in secilen_kumeler:
            kucuk_maske[labels_2d == kume_index] = 255

        maske = cv2.resize(
            kucuk_maske,
            (frame_w, frame_h),
            interpolation=cv2.INTER_NEAREST,
        )
        hedef_hsv_maskesi = self.hedef_renk_hsv_maskesi(hsv, hedef_renk)
        return cv2.bitwise_and(maske, hedef_hsv_maskesi)

    def qr_alanini_maskeden_cikar(self, maske_kirmizi, maske_mavi, frame, points_list):
        if points_list is None:
            return

        for points in points_list:
            if points is None:
                continue
            pts = np.array(points, dtype=np.int32).reshape(-1, 2)
            if len(pts) < 4:
                continue
            x, y, w, h = cv2.boundingRect(pts)
            pad = 15
            x1 = max(x - pad, 0)
            y1 = max(y - pad, 0)
            x2 = min(x + w + pad, frame.shape[1])
            y2 = min(y + h + pad, frame.shape[0])
            maske_kirmizi[y1:y2, x1:x2] = 0
            maske_mavi[y1:y2, x1:x2] = 0

    def daireleri_bul(self, frame, maske, renk_tipi):
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        temiz_maske = cv2.morphologyEx(maske, cv2.MORPH_OPEN, kernel, iterations=2)
        temiz_maske = cv2.morphologyEx(temiz_maske, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(
            temiz_maske,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        bulunan_daireler = []
        for contour in contours:
            alan = cv2.contourArea(contour)
            if alan < self.min_colored_circle_area:
                continue

            cevre = cv2.arcLength(contour, True)
            if cevre == 0:
                continue

            dairesellik = 4 * np.pi * alan / (cevre * cevre)
            epsilon = 0.02 * cevre
            approx = cv2.approxPolyDP(contour, epsilon, True)
            kose_sayisi = len(approx)
            x, y, w, h = cv2.boundingRect(contour)
            if w == 0 or h == 0:
                continue

            oran = w / float(h)
            (merkez_x, merkez_y), yaricap = cv2.minEnclosingCircle(contour)
            if yaricap <= 0 or yaricap < self.min_colored_circle_radius:
                continue

            min_cevre_alan = np.pi * yaricap * yaricap
            cember_doluluk = alan / min_cevre_alan
            kutu_doluluk = alan / float(w * h)
            noktalar = contour.reshape(-1, 2)
            uzakliklar = np.sqrt(
                (noktalar[:, 0] - merkez_x) ** 2
                + (noktalar[:, 1] - merkez_y) ** 2
            )
            if np.mean(uzakliklar) == 0:
                continue
            yaricap_sapma = np.std(uzakliklar) / np.mean(uzakliklar)

            if (
                dairesellik < 0.78
                or not 0.78 <= oran <= 1.22
                or kose_sayisi < 5
                or cember_doluluk < 0.65
                or kutu_doluluk > 0.90
                or yaricap_sapma > 0.22
            ):
                continue

            if not self.kontur_renk_dogrula(frame, contour, renk_tipi):
                continue

            bulunan_daireler.append(
                {
                    "type": renk_tipi,
                    "center_x": int(merkez_x),
                    "center_y": int(merkez_y),
                    "radius": int(yaricap),
                    "area": float(alan),
                }
            )

        return bulunan_daireler, temiz_maske

    def renkli_daire_tespiti(self, frame, points_list=None):
        maske_kirmizi = np.zeros(frame.shape[:2], dtype=np.uint8)
        maske_mavi = np.zeros(frame.shape[:2], dtype=np.uint8)

        if self.target_color in ("red", None):
            maske_kirmizi = self.kmeans_renk_maskesi_uret(frame, "red", k=4)
        if self.target_color in ("blue", None):
            maske_mavi = self.kmeans_renk_maskesi_uret(frame, "blue", k=4)

        self.qr_alanini_maskeden_cikar(
            maske_kirmizi,
            maske_mavi,
            frame,
            points_list,
        )

        tum_daireler = []
        temiz_kirmizi = maske_kirmizi
        temiz_mavi = maske_mavi
        if self.target_color in ("red", None):
            kirmizi_daireler, temiz_kirmizi = self.daireleri_bul(
                frame,
                maske_kirmizi,
                "red",
            )
            tum_daireler.extend(kirmizi_daireler)
        if self.target_color in ("blue", None):
            mavi_daireler, temiz_mavi = self.daireleri_bul(
                frame,
                maske_mavi,
                "blue",
            )
            tum_daireler.extend(mavi_daireler)

        if self.enable_display and self.show_masks:
            cv2.imshow("Kirmizi Maske", temiz_kirmizi)
            cv2.imshow("Mavi Maske", temiz_mavi)

        return tum_daireler

    def renkli_daireleri_frame_uzerine_ciz(self, frame, detected_circles):
        frame_w = frame.shape[1]
        frame_h = frame.shape[0]
        kamera_merkezi = (frame_w // 2, frame_h // 2)
        for circle in detected_circles:
            renk_tipi = circle["type"]
            if renk_tipi == "red":
                cizim_rengi = (0, 0, 255)
                etiket = "Kirmizi Daire"
            elif renk_tipi == "blue":
                cizim_rengi = (255, 0, 0)
                etiket = "Mavi Daire"
            else:
                continue

            merkez_x = frame_w - 1 - int(circle["center_x"])
            merkez_y = int(circle["center_y"])
            yaricap = int(circle["radius"])
            merkez = (merkez_x, merkez_y)
            cv2.line(frame, kamera_merkezi, merkez, cizim_rengi, 2, cv2.LINE_AA)
            cv2.circle(frame, merkez, yaricap, cizim_rengi, 2)
            cv2.circle(frame, merkez, 4, cizim_rengi, -1)
            cv2.putText(
                frame,
                etiket,
                (max(merkez_x - yaricap, 0), max(merkez_y - yaricap - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                cizim_rengi,
                2,
            )

    def qr_cozumunu_frame_uzerine_ciz(self, frame, qr_results, qr_points_list):
        frame_w = frame.shape[1]
        qr_frame_drawn = False
        if qr_points_list is not None:
            for index, points in enumerate(qr_points_list):
                if points is None:
                    continue
                pts = np.array(points, dtype=np.int32).reshape(-1, 2)
                if len(pts) < 4:
                    continue
                pts[:, 0] = frame_w - 1 - pts[:, 0]
                x, y, _, _ = cv2.boundingRect(pts)
                cv2.polylines(frame, [pts], True, (0, 255, 0), 3, cv2.LINE_AA)
                text = ""
                if qr_results is not None and index < len(qr_results):
                    text = qr_results[index] if isinstance(qr_results[index], str) else ""
                etiket = "QR Cozumlendi" if text.strip() else "QR Algilandi"
                cv2.putText(
                    frame,
                    etiket,
                    (x, max(y - 10, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                qr_frame_drawn = True

        has_decoded_text = any(
            isinstance(text, str) and text.strip()
            for text in (qr_results or [])
        )
        if not qr_frame_drawn and has_decoded_text:
            cv2.putText(
                frame,
                "QR Cozumlendi",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )

    def detect_qr(self, frame):
        if self.qr_detector is not None:
            try:
                texts, points = self.qr_detector.detectAndDecode(frame)
                return list(texts or []), points
            except Exception as exc:
                self.get_logger().warning(f"WeChat QR detect hatasi: {exc}")

        try:
            retval, decoded_info, points, _ = self.classic_qr_detector.detectAndDecodeMulti(
                frame
            )
        except Exception as exc:
            self.get_logger().warning(f"Klasik QR detect hatasi: {exc}")
            return [], None

        if not retval:
            return [], None
        return list(decoded_info or []), points

    def _draw_center_cross(self, frame):
        h, w = frame.shape[:2]
        cx = w // 2
        cy = h // 2
        cross_size = max(12, min(w, h) // 32)

        for color, thickness in (((0, 0, 0), 5), ((255, 255, 255), 2)):
            cv2.line(
                frame,
                (cx - cross_size, cy),
                (cx + cross_size, cy),
                color,
                thickness,
                cv2.LINE_AA,
            )
            cv2.line(
                frame,
                (cx, cy - cross_size),
                (cx, cy + cross_size),
                color,
                thickness,
                cv2.LINE_AA,
            )

    def _report_stream_state(self, frame):
        if frame is not None:
            if not self._stream_ready_logged:
                self._stream_ready_logged = True
                self.get_logger().info(
                    f"{self.image_source.upper()} image akisi geldi: "
                    f"{frame.shape[1]}x{frame.shape[0]}"
                )
            return

        now = time.time()
        if now - self._last_stream_warning_time < 2.0:
            return
        self._last_stream_warning_time = now
        last_error = getattr(self.capture, "last_error", "")
        self.get_logger().warning(
            f"Henuz frame yok. source={self.image_source}, hata={last_error}"
        )

    def _draw_display(self, frame, qr_results, qr_points_list, detected_circles):
        if not self.enable_display:
            return

        display_frame = cv2.flip(frame, 1)
        if self.qr_detection_allowed:
            self.qr_cozumunu_frame_uzerine_ciz(
                display_frame,
                qr_results,
                qr_points_list,
            )
        if (self.colored_field_detection_allowed or self.xy_error_enabled) and detected_circles:
            self.renkli_daireleri_frame_uzerine_ciz(display_frame, detected_circles)

        if self.display_width > 0 and self.display_height > 0:
            display_frame = cv2.resize(
                display_frame,
                (self.display_width, self.display_height),
            )

        self._draw_center_cross(display_frame)

        if not self._display_window_ready:
            cv2.namedWindow(
                self.display_window_name,
                cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL,
            )
            if self.display_width > 0 and self.display_height > 0:
                cv2.resizeWindow(
                    self.display_window_name,
                    self.display_width,
                    self.display_height,
                )
            self._display_window_ready = True

        cv2.imshow(self.display_window_name, display_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") and rclpy.ok():
            rclpy.shutdown()

    def process_frame(self):
        if self.capture is None:
            return

        frame = self.capture.get_frame()
        self._report_stream_state(frame)
        if frame is None:
            return

        qr_results = []
        qr_points_list = None
        detected_circles = []

        if self.qr_detection_allowed:
            qr_results, qr_points_list = self.detect_qr(frame)
            for text in qr_results:
                if not isinstance(text, str) or not text.strip():
                    continue
                text = text.strip()
                if text in self.last_sent_qr:
                    continue
                self.publish_qr_result(text)
                self.last_sent_qr.add(text)

        if self.colored_field_detection_allowed or self.xy_error_enabled:
            detected_circles = self.renkli_daire_tespiti(frame, qr_points_list)
            if detected_circles:
                best_circle = max(detected_circles, key=lambda circle: circle["area"])
                if (
                    self.colored_field_detection_allowed
                    and self._is_colored_field_approaching(frame, best_circle)
                ):
                    self.publish_colored_field_result(best_circle)
                if self.xy_error_enabled:
                    self.publish_x_y_error_result(frame, best_circle)
                self.get_logger().debug(
                    "Renkli hedef yayinlandi | "
                    f"type={best_circle['type']}, "
                    f"x={best_circle['center_x']}, y={best_circle['center_y']}"
                )
            else:
                if self.xy_error_enabled:
                    self.publish_no_detection()
                if self.target_color in ("red", "blue"):
                    self._reset_color_approach_state(self.target_color)
                else:
                    self._reset_color_approach_state()

        self._draw_display(frame, qr_results, qr_points_list, detected_circles)

    def destroy_node(self):
        self._teardown_capture()
        if self.enable_display:
            cv2.destroyAllWindows()
        super().destroy_node()


# Eski public sinif adini da yeni uyarlamaya bagla; entry point disindaki
# import kullanimlari ayni isimle yeni node'u alir.
ImageProcessingNode = AdaptedImageProcessingNode


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
