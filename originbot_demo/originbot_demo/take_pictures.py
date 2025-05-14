#!/usr/bin/env python3

import os
import sys
import cv2
import time
import argparse
from datetime import datetime

def is_usb_camera(device):
    try:
        cap = cv2.VideoCapture(device)
        if not cap.isOpened():
            return False
        cap.release()
        return True
    except Exception:
        return False

def find_first_usb_camera():
    video_devices = [os.path.join('/dev', dev) for dev in os.listdir('/dev') if dev.startswith('video')]
    for dev in video_devices:
        if is_usb_camera(dev):
            return dev
    return None

def main():
    parser = argparse.ArgumentParser(description="USB Camera Snapshot Script")
    parser.add_argument('--dev', type=str, help='Video device path (e.g., /dev/video0)')
    parser.add_argument('numbers', nargs='*', type=int, help='Number of images to capture and optional interval in ms')
    args = parser.parse_args()

    video_device = args.dev if args.dev else find_first_usb_camera()

    if video_device is None:
        print("No USB camera found.")
        sys.exit(-1)

    print(f"Opening video device: {video_device}")
    cap = cv2.VideoCapture(video_device)

    if not cap.isOpened():
        print(f"Failed to open video device: {video_device}")
        sys.exit(-1)

    print("Open USB camera successfully")

    # 设置 USB camera 的输出图像格式为 MJPEG，分辨率 1920 x 1080
    codec = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    cap.set(cv2.CAP_PROP_FOURCC, codec)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    
    # 获取参数：截取多少张图片和设定截取间隔(ms)
    num_images = args.numbers[0] if len(args.numbers) > 0 else 1
    interval = args.numbers[1] if len(args.numbers) > 1 else 1000

    for i in range(num_images):
        ret, frame = cap.read()

        if not ret or frame is None:
            print("Failed to get image from USB camera")
            sys.exit(-1)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[-6:]  # 使用时间戳的后6位（微秒）
        img_filename = f"img_{timestamp}_{i+1}.jpg"
        cv2.imwrite(img_filename, frame)
        print(f"Image saved as {img_filename}")

        if i < num_images - 1:
            time.sleep(interval / 1000.0)
if __name__ == '__main__':
    main()