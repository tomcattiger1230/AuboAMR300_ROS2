#!/usr/bin/env python3
"""Serve the wrist RGB camera as a bounded-memory MJPEG stream."""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraVideo(Node):
    def __init__(self):
        super().__init__('camera_video')
        for key, value in [('host', '127.0.0.1'), ('port', 8080), ('fps', 10.0),
                           ('image_topic', '/camera/color/image_raw')]:
            self.declare_parameter(key, value)
        self.frame = None
        self.updated = 0.0
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, self.get_parameter('image_topic').value,
                                            self.on_image, qos_profile_sensor_data)
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def do_GET(self):
                if self.path == '/':
                    body = b'<!doctype html><meta charset="utf-8"><title>Wrist camera</title><h1>Wrist camera</h1><img src="/stream.mjpg" style="max-width:100%;height:auto"><p>Live simulation feed</p>'
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path not in ('/stream.mjpg', '/snapshot.jpg'):
                    self.send_error(404)
                    return
                with owner.lock:
                    frame, updated = owner.frame, owner.updated
                if frame is None or time.monotonic() - updated > 3:
                    self.send_error(503, 'No fresh camera frame')
                    return
                self.send_response(200)
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Type', 'image/jpeg' if self.path == '/snapshot.jpg'
                                 else 'multipart/x-mixed-replace; boundary=frame')
                self.end_headers()
                try:
                    if self.path == '/snapshot.jpg':
                        self.wfile.write(frame)
                        return
                    last = 0.0
                    while rclpy.ok():
                        with owner.lock:
                            frame, updated = owner.frame, owner.updated
                        if time.monotonic() - updated > 3:
                            break
                        if updated != last:
                            self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                             + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                            self.wfile.flush()
                            last = updated
                        time.sleep(.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer((self.get_parameter('host').value,
                                           self.get_parameter('port').value), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f'Camera video: http://{self.server.server_address[0]}:{self.server.server_port}')

    def on_image(self, msg):
        if time.monotonic() - self.updated < 1.0 / max(.1, self.get_parameter('fps').value):
            return
        try:
            ok, jpeg = cv2.imencode('.jpg', self.bridge.imgmsg_to_cv2(msg, 'bgr8'),
                                   [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.lock:
                    self.frame, self.updated = jpeg.tobytes(), time.monotonic()
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(str(exc))


def main():
    rclpy.init()
    node = CameraVideo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.server.shutdown()
        node.server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
