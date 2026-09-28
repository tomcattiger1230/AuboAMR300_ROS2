"""ROS occupancy map, map-frame path, robot marker and drag-to-set goal."""
import math
from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import QWidget

class MapCanvas(QWidget):
    goal_selected = Signal(float, float, float)
    initial_pose_selected = Signal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 360)
        self.grid = self.image = None
        self.path = []
        self.robot = None
        self.goal = None
        self.press = None
        self.selection_mode = 'goal'
        self.initial_pose = None

    def set_map(self, msg):
        if msg.header.frame_id.strip('/') != 'map':
            return
        if msg.info.width * msg.info.height != len(msg.data) or msg.info.resolution <= 0:
            return
        self.grid = msg
        data = bytes(235 if v == 0 else 45 if v > 50 else 155 for v in msg.data)
        self.image = QImage(data, msg.info.width, msg.info.height, msg.info.width, QImage.Format_Grayscale8).copy()
        self.update()

    def set_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses] if msg.header.frame_id.strip('/') == 'map' else []
        self.update()

    def transform(self):
        if not self.grid:
            return (40, self.width()/2, self.height()/2)
        info = self.grid.info
        scale = min((self.width()-40)/(info.width*info.resolution), (self.height()-40)/(info.height*info.resolution))
        return scale, (self.width()-info.width*info.resolution*scale)/2, (self.height()+info.height*info.resolution*scale)/2

    def local(self, x, y):
        if not self.grid:
            return x, y
        o = self.grid.info.origin
        q = o.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        x, y = x-o.position.x, y-o.position.y
        return math.cos(yaw)*x+math.sin(yaw)*y, -math.sin(yaw)*x+math.cos(yaw)*y

    def world(self, point):
        scale, ox, oy = self.transform()
        x, y = (point.x()-ox)/scale, (oy-point.y())/scale
        o = self.grid.info.origin
        q = o.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        return o.position.x+math.cos(yaw)*x-math.sin(yaw)*y, o.position.y+math.sin(yaw)*x+math.cos(yaw)*y

    def screen(self, x, y):
        x, y = self.local(x, y)
        s, ox, oy = self.transform()
        return QPointF(ox+x*s, oy-y*s)

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#e9eef4'))
        if self.image:
            s, ox, oy = self.transform()
            info = self.grid.info
            p.save()
            p.translate(ox, oy)
            p.scale(s*info.resolution, -s*info.resolution)
            p.drawImage(QPointF(0, 0), self.image)
            p.restore()
        else:
            p.setPen(QColor('#52657b'))
            p.drawText(self.rect(), Qt.AlignCenter, '等待 ROS 地图 /map\n拖动地图设置目标位置和朝向')
        p.setPen(QPen(QColor('#16a6a1'), 3))
        for a, b in zip(self.path, self.path[1:]):
            p.drawLine(self.screen(*a), self.screen(*b))
        for pose, color in ((self.robot, '#1769df'), (self.goal, '#ee9a2c'), (self.initial_pose, '#914ac9')):
            if pose:
                x, y, yaw = pose
                point = self.screen(x, y)
                p.setBrush(QColor(color)); p.setPen(QPen(QColor(color), 3))
                p.drawEllipse(point, 6, 6)
                p.drawLine(point, self.screen(x+.4*math.cos(yaw), y+.4*math.sin(yaw)))

    def mousePressEvent(self, event):
        if self.grid and event.button() == Qt.LeftButton:
            self.press = self.world(event.position())

    def mouseReleaseEvent(self, event):
        if self.press and event.button() == Qt.LeftButton:
            x, y = self.press
            end = self.world(event.position())
            self.press = None
            lx, ly = self.local(x, y)
            info = self.grid.info
            ix, iy = int(lx/info.resolution), int(ly/info.resolution)
            if lx < 0 or ly < 0 or not (0 <= ix < info.width and 0 <= iy < info.height):
                return
            if self.grid.data[iy*info.width+ix] < 0 or self.grid.data[iy*info.width+ix] > 50:
                return
            pose = (x, y, math.atan2(end[1]-y, end[0]-x))
            if self.selection_mode == 'initial':
                self.initial_pose = pose
                self.initial_pose_selected.emit(*pose)
            else:
                self.goal = pose
                self.goal_selected.emit(*pose)
            self.update()

    def is_free(self, x, y):
        if not self.grid: return False
        lx, ly = self.local(x, y)
        info = self.grid.info
        ix, iy = int(lx/info.resolution), int(ly/info.resolution)
        return (lx >= 0 and ly >= 0 and 0 <= ix < info.width and 0 <= iy < info.height
                and 0 <= self.grid.data[iy*info.width+ix] <= 50)
