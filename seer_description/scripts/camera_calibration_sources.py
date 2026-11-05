"""Raw frame sources; MVS and ROS imports are optional and loaded on demand."""
import ctypes
import importlib
import json
import os
from pathlib import Path
import sys
import time
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import cv2
import numpy as np


def load_mvs(sdk_path=None):
    candidates = [sdk_path, os.environ.get('MVS_PYTHON_PATH'),
                  '/opt/MVS/Samples/aarch64/Python/MvImport',
                  '/opt/MVS/Samples/64/Python/MvImport', '/opt/MVS/Samples/Python/MvImport']
    for candidate in candidates:
        if candidate:
            path = Path(candidate).expanduser()
            if (path / 'MvImport').is_dir():
                path = path / 'MvImport'
            if (path / 'MvCameraControl_class.py').is_file():
                sys.path.insert(0, str(path))
                break
    try:
        return importlib.import_module('MvCameraControl_class')
    except (ImportError, OSError, TypeError) as exc:
        raise RuntimeError('Cannot load MVS Python bindings/runtime. Set --sdk-path to MvImport and '
                           'source the MVS environment for your architecture. Original error: ' + str(exc)) from exc


class MvsSource:
    def __init__(self, sdk_path=None, serial=None, exposure_us=None, gain_db=None, list_only=False):
        self.sdk = load_mvs(sdk_path)
        self.camera = None
        self.created = self.opened = self.grabbing = self.initialized = False
        self.metadata = {}
        try:
            sdk = self.sdk
            if hasattr(sdk.MvCamera, 'MV_CC_Initialize'):
                self.check(sdk.MvCamera.MV_CC_Initialize(), 'Initialize')
                self.initialized = True
            self.devices = sdk.MV_CC_DEVICE_INFO_LIST()
            self.check(sdk.MvCamera.MV_CC_EnumDevices(sdk.MV_USB_DEVICE, self.devices), 'EnumDevices')
            devices = []
            for index in range(self.devices.nDeviceNum):
                info = ctypes.cast(self.devices.pDeviceInfo[index], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
                usb = info.SpecialInfo.stUsb3VInfo
                decode = lambda value: bytes(value).split(b'\0', 1)[0].decode(errors='replace')
                devices.append({'index': index, 'serial': decode(usb.chSerialNumber), 'model': decode(usb.chModelName)})
            self.available_devices = devices
            if list_only:
                return
            matching = [d for d in devices if serial is None or d['serial'] == serial]
            if len(matching) != 1:
                raise RuntimeError(f'Expected one USB camera; found {devices}. Select --serial explicitly if needed.')
            device = matching[0]
            info = ctypes.cast(self.devices.pDeviceInfo[device['index']], ctypes.POINTER(sdk.MV_CC_DEVICE_INFO)).contents
            self.camera = sdk.MvCamera()
            self.check(self.camera.MV_CC_CreateHandle(info), 'CreateHandle')
            self.created = True
            self.check(self.camera.MV_CC_OpenDevice(sdk.MV_ACCESS_Exclusive, 0), 'OpenDevice (close MVS GUI/other driver)')
            self.opened = True
            # Fixed acquisition settings; no user-set write or persistent configuration save.
            for name, value in [('TriggerMode', 0), ('ExposureAuto', 0), ('GainAuto', 0),
                                ('AcquisitionMode', 2), ('PixelFormat', 0x01080001)]:  # continuous, Mono8
                self.check(self.camera.MV_CC_SetEnumValue(name, value), 'Set ' + name)
            for name, value in [('ExposureTime', exposure_us), ('Gain', gain_db)]:
                if value is not None:
                    self.check(self.camera.MV_CC_SetFloatValue(name, value), 'Set ' + name)
            self.metadata = {'source': 'mvs', **device, 'pixel_format': 'Mono8', 'exposure_auto': False, 'gain_auto': False}
            for name in ('Width', 'Height', 'OffsetX', 'OffsetY', 'BinningHorizontal', 'BinningVertical',
                         'DecimationHorizontal', 'DecimationVertical'):
                value = sdk.MVCC_INTVALUE()
                if self.camera.MV_CC_GetIntValue(name, value) == 0:
                    self.metadata[name] = int(value.nCurValue)
            for name in ('ExposureTime', 'Gain'):
                value = sdk.MVCC_FLOATVALUE()
                if self.camera.MV_CC_GetFloatValue(name, value) == 0:
                    self.metadata[name] = float(value.fCurValue)
            for name in ('ReverseX', 'ReverseY'):
                value = ctypes.c_bool()
                if self.camera.MV_CC_GetBoolValue(name, value) == 0:
                    self.metadata[name] = bool(value.value)
            if hasattr(self.camera, 'MV_CC_SetGrabStrategy'):
                self.check(self.camera.MV_CC_SetGrabStrategy(sdk.MV_GrabStrategy_LatestImagesOnly),
                           'SetGrabStrategy LatestImagesOnly')
            self.check(self.camera.MV_CC_StartGrabbing(), 'StartGrabbing')
            self.grabbing = True
        except BaseException:
            self.close()
            raise

    @staticmethod
    def check(code, operation):
        if code != 0:
            raise RuntimeError(f'MVS {operation} failed: 0x{code & 0xffffffff:08x}')

    def read(self, timeout=1.):
        frame = self.sdk.MV_FRAME_OUT()
        ret = self.camera.MV_CC_GetImageBuffer(frame, int(timeout*1000))
        if ret & 0xffffffff == 0x80000007:  # MV_E_NODATA
            return None
        self.check(ret, 'GetImageBuffer')
        try:
            info = frame.stFrameInfo
            if info.enPixelType != 0x01080001:
                raise RuntimeError('Expected native Mono8; camera returned a different pixel format')
            width, height = int(info.nWidth), int(info.nHeight)
            if int(info.nFrameLen) < width * height:
                raise RuntimeError('Incomplete MVS image buffer')
            # Copy before releasing SDK-owned buffer; preserve full resolution.
            image = np.ctypeslib.as_array(ctypes.cast(frame.pBufAddr, ctypes.POINTER(ctypes.c_ubyte)),
                                          shape=(height*width,)).reshape(height, width).copy()
            stamp = (int(info.nDevTimeStampHigh) << 32) | int(info.nDevTimeStampLow)
            return image, {'frame_number': int(info.nFrameNum), 'device_timestamp_ticks': stamp,
                           'received_monotonic': time.monotonic()}
        finally:
            self.check(self.camera.MV_CC_FreeImageBuffer(frame), 'FreeImageBuffer')

    def close(self):
        if self.grabbing:
            self.camera.MV_CC_StopGrabbing()
            self.grabbing = False
        if self.opened:
            self.camera.MV_CC_CloseDevice()
            self.opened = False
        if self.created:
            self.camera.MV_CC_DestroyHandle()
            self.created = False
        if self.initialized:
            self.sdk.MvCamera.MV_CC_Finalize()
            self.initialized = False


class RosSource:
    def __init__(self, topic):
        try:
            import rclpy
            from cv_bridge import CvBridge
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import Image
        except ImportError as exc:
            raise RuntimeError('Source your ROS 2 environment and install cv_bridge/sensor_msgs') from exc
        self.rclpy = rclpy
        rclpy.init(args=[])
        self.node = rclpy.create_node('capture_camera_intrinsics')
        self.bridge = CvBridge()
        self.latest = None
        self.error = None
        self.counter = 0
        self.metadata = {'source': 'ros', 'topic': topic,
                         'note': 'Camera settings must be recorded manually using --notes'}
        self.subscription = self.node.create_subscription(Image, topic, self.receive, qos_profile_sensor_data)

    def receive(self, message):
        try:
            if message.encoding not in ('mono8', 'bgr8', 'rgb8', 'bgra8', 'rgba8'):
                raise ValueError('Unsupported encoding: ' + message.encoding + '; use native Mono8')
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8').copy()
            self.counter += 1
            self.latest = image, {'frame_number': self.counter, 'frame_id': message.header.frame_id,
                                  'stamp_sec': message.header.stamp.sec, 'stamp_nanosec': message.header.stamp.nanosec,
                                  'received_monotonic': time.monotonic()}
        except Exception as exc:
            self.error = exc

    def read(self, timeout=1.):
        self.latest = None
        deadline = time.monotonic() + timeout
        while self.rclpy.ok() and time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=.05)
            if self.error:
                raise RuntimeError('ROS image conversion failed: ' + str(self.error))
            if self.latest is not None:
                return self.latest
        return None

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


class PreviewSource:
    """Reuse the existing Jetson hik_web_preview's lossless snapshot API locally.

    This is deliberately not a JPEG/video reader. Run on the same host as the
    service because its API returns the local path to the native PNG snapshot.
    """
    def __init__(self, url='http://127.0.0.1:8080', interval=2.):
        parsed = urlsplit(url)
        if parsed.scheme != 'http' or parsed.hostname not in ('localhost', '127.0.0.1'):
            raise ValueError('Preview source must run on the Jetson using http://127.0.0.1:8080')
        self.url = url.rstrip('/')
        self.interval = min(interval, 2.)
        self.last_read = 0.
        status = self.request('/api/status')
        self.settings = {key: status.get(key) for key in
                         ('serial', 'width', 'height', 'exposure_us', 'gain', 'exposure_auto')}
        if self.settings['exposure_auto'] != 0:
            raise ValueError('Disable automatic exposure in the preview before calibration')
        self.metadata = {'source': 'preview_native_png', 'url': self.url, **self.settings,
                         'note': 'Native PNG via snapshot API; no JPEG or resizing. Snapshot files also remain '
                                 'in the preview service output directory. Record gain auto/ROI in --notes.'}

    def request(self, path, post=False):
        request = Request(self.url + path, data=b'{}' if post else None,
                          headers={'Content-Type': 'application/json'})
        with urlopen(request, timeout=20) as response:
            return json.load(response)

    def read(self, timeout=1.):
        remaining = self.interval - (time.monotonic() - self.last_read)
        if remaining > 0:
            time.sleep(min(timeout, remaining))
            return None
        status = self.request('/api/status')
        current = {key: status.get(key) for key in self.settings}
        if current != self.settings:
            raise RuntimeError('Preview camera settings changed; restore them or start a new dataset')
        if status.get('status') != 'LIVE' or (status.get('frame_age_s') or 0) > 2:
            raise RuntimeError('Preview is not live or frame is stale')
        snapshot = self.request('/api/snapshot', post=True)
        path = Path(snapshot['path'])
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError('Cannot read native PNG snapshot. Run the capture script on the same Jetson as the preview.')
        if image.ndim != 2 or image.dtype != np.uint8 or image.shape != (self.settings['height'], self.settings['width']):
            raise RuntimeError('Preview snapshot is not the expected native-resolution Mono8 image')
        self.last_read = time.monotonic()
        return image, {'snapshot_path': str(path), 'received_monotonic': self.last_read,
                       'status_frame_number_before_snapshot': status.get('frames')}

    def close(self):
        pass
