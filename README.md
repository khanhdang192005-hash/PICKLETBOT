import cv2
import time
import math
import serial
import threading

from picamera2 import Picamera2
from rplidar import RPLidar


# =====================================================
# CẤU HÌNH
# =====================================================

ESP32_PORT = "/dev/ttyUSB0"
LIDAR_PORT = "/dev/ttyUSB1"

BAUDRATE = 115200

WIDTH = 640
HEIGHT = 480

FORWARD_SPEED = 75
TURN_SPEED = 60

SAFE_DISTANCE = 450
CLOSE_DISTANCE = 280

MAX_BALLS = 5


# =====================================================
# UART ESP32
# =====================================================

esp32 = serial.Serial(
    ESP32_PORT,
    BAUDRATE,
    timeout=0.1
)

time.sleep(2)


def send_cmd(cmd):

    print("ESP32 <-", cmd)

    esp32.write(
        (cmd + "\n").encode()
    )

    time.sleep(0.03)


# =====================================================
# CAMERA CSI
# =====================================================

camera = Picamera2()

config = camera.create_preview_configuration(
    main={
        "size": (WIDTH, HEIGHT),
        "format": "RGB888"
    }
)

camera.configure(config)

camera.start()

time.sleep(2)


# =====================================================
# RPLIDAR A1
# =====================================================

lidar = RPLidar(
    LIDAR_PORT
)

lidar_data = []

lidar_lock = threading.Lock()

running = True


def lidar_loop():

    global lidar_data

    try:

        for scan in lidar.iter_scans(
            max_buf_meas=1000
        ):

            if not running:
                break

            with lidar_lock:

                lidar_data = scan

    except Exception as e:

        print("LIDAR ERROR:", e)


lidar_thread = threading.Thread(
    target=lidar_loop,
    daemon=True
)

lidar_thread.start()


# =====================================================
# KHOẢNG CÁCH PHÍA TRƯỚC
# =====================================================

def front_distance():

    with lidar_lock:

        scan = list(lidar_data)

    distances = []

    for quality, angle, distance in scan:

        if distance <= 0:
            continue

        if angle <= 25 or angle >= 335:

            distances.append(distance)

    if not distances:

        return 9999

    return min(distances)


# =====================================================
# KIỂM TRA VẬT CẢN
# =====================================================

def obstacle():

    distance = front_distance()

    print(
        "LiDAR:",
        int(distance),
        "mm"
    )

    return distance < SAFE_DISTANCE


# =====================================================
# NHẬN DIỆN BÓNG
# =====================================================

def detect_ball(frame):

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_RGB2HSV
    )

    # Khoảng màu ban đầu.
    # Cần điều chỉnh theo màu bóng thực tế.

    lower = (
        15,
        50,
        60
    )

    upper = (
        45,
        255,
        255
    )

    mask = cv2.inRange(
        hsv,
        lower,
        upper
    )

    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (5, 5)
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:

        return None

    best = None
    best_score = 0

    for contour in contours:

        area = cv2.contourArea(
            contour
        )

        if area < 120:
            continue

        perimeter = cv2.arcLength(
            contour,
            True
        )

        if perimeter == 0:
            continue

        circularity = (
            4 *
            math.pi *
            area /
            (perimeter ** 2)
        )

        if circularity < 0.35:
            continue

        x, y, w, h = cv2.boundingRect(
            contour
        )

        ratio = w / float(h)

        if ratio < 0.5 or ratio > 1.8:
            continue

        score = area * circularity

        if score > best_score:

            best_score = score

            best = (
                x,
                y,
                w,
                h,
                area
            )

    if best is None:

        return None

    x, y, w, h, area = best

    return {
        "x": x + w // 2,
        "y": y + h // 2,
        "w": w,
        "h": h,
        "area": area
    }


# =====================================================
# XÁC ĐỊNH HƯỚNG BÓNG
# =====================================================

def get_direction(ball):

    if ball is None:

        return "NONE"

    x = ball["x"]

    if x < WIDTH * 0.40:

        return "LEFT"

    if x > WIDTH * 0.60:

        return "RIGHT"

    return "CENTER"


# =====================================================
# KIỂM TRA BÓNG ĐÃ GẦN
# =====================================================

def ball_close(ball):

    if ball is None:

        return False

    return ball["area"] > 4500


# =====================================================
# TÌM BÓNG
# =====================================================

def search_ball():

    print("\n--- SEARCH BALL ---")

    start = time.time()

    direction = "L"

    while running:

        frame = camera.capture_array()

        ball = detect_ball(frame)

        if ball:

            send_cmd("STOP")

            return ball

        if direction == "L":

            send_cmd(
                "L" + str(TURN_SPEED)
            )

        else:

            send_cmd(
                "R" + str(TURN_SPEED)
            )

        if time.time() - start > 3:

            direction = (
                "R"
                if direction == "L"
                else "L"
            )

            start = time.time()

    return None


# =====================================================
# TRÁNH VẬT CẢN
# =====================================================

def avoid_obstacle():

    print(
        "!!! OBSTACLE !!!"
    )

    send_cmd("STOP")

    time.sleep(0.2)

    send_cmd(
        "R" + str(TURN_SPEED)
    )

    time.sleep(0.6)

    send_cmd("STOP")


# =====================================================
# TIẾP CẬN BÓNG
# =====================================================

def approach_ball():

    print("\n--- APPROACH ---")

    while running:

        frame = camera.capture_array()

        ball = detect_ball(frame)

        if ball is None:

            send_cmd("STOP")

            return False

        if obstacle():

            avoid_obstacle()

            continue

        if ball_close(ball):

            send_cmd("STOP")

            time.sleep(0.3)

            return True

        direction = get_direction(ball)

        if direction == "LEFT":

            send_cmd(
                "L" + str(TURN_SPEED)
            )

        elif direction == "RIGHT":

            send_cmd(
                "R" + str(TURN_SPEED)
            )

        else:

            if front_distance() < CLOSE_DISTANCE:

                send_cmd("STOP")

                return True

            send_cmd(
                "F" + str(FORWARD_SPEED)
            )

        time.sleep(0.05)

    return False


# =====================================================
# THU BÓNG
# =====================================================

def collect_ball():

    print("\n--- COLLECT ---")

    send_cmd("STOP")

    time.sleep(0.3)

    send_cmd("COLLECT")

    time.sleep(1.5)

    send_cmd("COLLECT_STOP")

    time.sleep(0.5)


# =====================================================
# CẤP BÓNG
# =====================================================

def feed_ball():

    print("\n--- FEED ---")

    send_cmd("STOP")

    time.sleep(0.5)

    send_cmd("FEED")

    time.sleep(1.5)


# =====================================================
# PHÁT BÓNG
# =====================================================

def shoot_ball():

    print("\n--- SHOOT ---")

    send_cmd("SHOOT")

    time.sleep(2)

    send_cmd("SHOOT_STOP")

    time.sleep(0.5)


# =====================================================
# CHƯƠNG TRÌNH TỰ ĐỘNG
# =====================================================

def autonomous():

    ball_count = 0

    while running:

        print(
            "\n=============================="
        )

        print(
            "BALL:",
            ball_count,
            "/",
            MAX_BALLS
        )

        print(
            "=============================="
        )

        if ball_count >= MAX_BALLS:

            send_cmd("STOP")

            feed_ball()

            shoot_ball()

            ball_count = 0

            continue

        ball = search_ball()

        if ball is None:

            continue

        success = approach_ball()

        if not success:

            continue

        collect_ball()

        ball_count += 1

        print(
            "Da thu:",
            ball_count
        )

        time.sleep(0.5)


# =====================================================
# MAIN
# =====================================================

try:

    print(
        "================================="
    )

    print(
        " PICKLEBOT - FULL AUTO"
    )

    print(
        " Camera CSI + RPLIDAR A1"
    )

    print(
        " Raspberry Pi -> ESP32"
    )

    print(
        "================================="
    )

    autonomous()

except KeyboardInterrupt:

    print("STOP")

finally:

    running = False

    send_cmd("STOP")

    try:
        lidar.stop()
        lidar.disconnect()
    except:
        pass

    try:
        camera.stop()
    except:
        pass

    try:
        esp32.close()
    except:
        pass
