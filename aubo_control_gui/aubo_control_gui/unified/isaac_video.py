"""Live Isaac third-person view; network starts only after robot connection."""
from PySide6.QtCore import QTimer, Qt, QUrl
from PySide6.QtGui import QPixmap
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
from PySide6.QtWidgets import QWidget, QLabel, QVBoxLayout, QSizePolicy

STAGES = {'setup':'场景就绪','01':'取筋并装车','02':'载筋运送','03':'车载取筋',
          '04':'钢筋竖直化与预定位','05':'预定位确认','06':'接近测试机',
          '07':'夹持线微调','08':'测试机接管','complete':'实验完成','failed':'实验停止，请查看报告'}


class IsaacVideo(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        layout=QVBoxLayout(self)
        self.status=QLabel('连接仿真后显示 Isaac 实时第三视角')
        self.frame=QLabel('等待 Isaac 实时画面'); self.frame.setAlignment(Qt.AlignCenter)
        self.frame.setMinimumSize(500,280)
        self.frame.setSizePolicy(QSizePolicy.Ignored,QSizePolicy.Ignored)
        self.frame.setStyleSheet('background:#182332;color:white;border-radius:8px')
        layout.addWidget(self.status); layout.addWidget(self.frame,1)
        self.manager=QNetworkAccessManager(self)
        self.timer=QTimer(self); self.timer.setInterval(400); self.timer.timeout.connect(self.fetch)
        self.reply=None; self.url=None; self.pixmap=None

    def start(self,peer):
        self.stop()
        self.url=QUrl(f'http://{peer}:8081/snapshot.jpg')
        self.timer.start(); self.fetch()

    def stop(self):
        self.timer.stop(); self.url=None; self.pixmap=None
        if self.reply:
            reply=self.reply; self.reply=None; reply.abort(); reply.deleteLater()
        self.frame.clear(); self.frame.setText('等待 Isaac 实时画面')
        self.status.setText('实时场景未连接')

    def fetch(self):
        if self.reply is not None or self.url is None: return
        request=QNetworkRequest(self.url); request.setTransferTimeout(3000)
        reply=self.manager.get(request); self.reply=reply
        reply.finished.connect(lambda:self.received(reply))

    def received(self,reply):
        if reply is not self.reply: return
        self.reply=None
        pixmap=QPixmap()
        ok=(reply.error()==QNetworkReply.NoError
            and reply.attribute(QNetworkRequest.HttpStatusCodeAttribute)==200
            and pixmap.loadFromData(bytes(reply.readAll()),'JPEG'))
        if ok:
            self.pixmap=pixmap
            self.status.setText('Isaac 实时仿真 · '+STAGES.get(bytes(reply.rawHeader('X-Isaac-Stage')).decode('ascii','replace'),'场景运行中'))
            self.draw_frame()
        else:
            self.pixmap=None; self.frame.clear(); self.frame.setText('实时画面暂不可用')
            self.status.setText('等待 Isaac 第三视角服务 · 8081')
        reply.deleteLater()

    def draw_frame(self):
        if self.pixmap: self.frame.setPixmap(self.pixmap.scaled(self.frame.size(),Qt.KeepAspectRatio,Qt.SmoothTransformation))

    def resizeEvent(self,event):
        super().resizeEvent(event); self.draw_frame()
