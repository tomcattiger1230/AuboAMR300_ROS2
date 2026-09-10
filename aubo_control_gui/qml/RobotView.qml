import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils

View3D {
    id: view
    camera: sceneCamera
    property bool ghostVisible: false
    property var ghostJoints: [0,0,0,0,0,0]
    property var ghostFingers: [0,0]
    property bool targetEnabled: false
    property vector3d targetScene: Qt.vector3d(0,0,0)
    property var targetBasis: [[1,0,0],[0,1,0],[0,0,1]]
    property var targetOrientation: [[1,0,0],[0,1,0],[0,0,1]]
    property string targetStatus: "点击同步当前末端，再拖动坐标轴或旋转环"
    signal targetDrag(int axis, double amount, bool rotation)

    property url toolMeshRoot
    property var toolVisuals: []
    property real finger1: 0
    property real finger2: 0
    property url meshRoot
    property string meshExtension: "DAE"
    property real j1: 0; property real j2: 0; property real j3: 0
    property real j4: 0; property real j5: 0; property real j6: 0
    environment: SceneEnvironment { backgroundMode: SceneEnvironment.Color; clearColor: "#101722"; antialiasingMode: SceneEnvironment.MSAA; antialiasingQuality: SceneEnvironment.High }
    Node {
        id: cameraRig; eulerRotation.x: -22; eulerRotation.y: -42
        PerspectiveCamera { id: sceneCamera; z: 230; clipFar: 2000 }
    }
    DirectionalLight { eulerRotation: Qt.vector3d(-45,-30,0); brightness: 1.4; castsShadow: true }
    DirectionalLight { eulerRotation: Qt.vector3d(40,140,0); brightness: 0.7 }
    Model { source: "#Cube"; position: Qt.vector3d(0,-45,0); scale: Qt.vector3d(1.5,.01,1.5); materials: PrincipledMaterial { baseColor: "#253447"; roughness: 1 } }
    ArmModel {
        meshRoot: view.meshRoot; meshExtension: view.meshExtension
        toolMeshRoot: view.toolMeshRoot; toolVisuals: view.toolVisuals
        j1: view.j1; j2: view.j2; j3: view.j3; j4: view.j4; j5: view.j5; j6: view.j6
        finger1: view.finger1; finger2: view.finger2
    }
    ArmModel {
        meshRoot: view.meshRoot; meshExtension: view.meshExtension
        toolMeshRoot: view.toolMeshRoot; toolVisuals: view.toolVisuals
        visible: view.ghostVisible; opacity: 0.32
        j1: view.ghostJoints[0]; j2: view.ghostJoints[1]; j3: view.ghostJoints[2]
        j4: view.ghostJoints[3]; j5: view.ghostJoints[4]; j6: view.ghostJoints[5]
        finger1: view.ghostFingers[0]; finger2: view.ghostFingers[1]
    }
    MouseArea {
        anchors.fill: parent; property real oldX; property real oldY
        onPressed: function(mouse) { oldX=mouse.x; oldY=mouse.y }
        onPositionChanged: function(mouse) { if (pressed) { cameraRig.eulerRotation.y += (mouse.x-oldX)*.35; cameraRig.eulerRotation.x=Math.max(-85,Math.min(85,cameraRig.eulerRotation.x+(mouse.y-oldY)*.35)); oldX=mouse.x; oldY=mouse.y } }
        onWheel: function(wheel) { sceneCamera.z=Math.max(80,Math.min(450,sceneCamera.z-wheel.angleDelta.y*.15)) }
    }
    EndEffectorGizmo {
        anchors.fill: parent; view3d: view
        enabled: view.targetEnabled; visible: view.targetEnabled
        center: view.targetScene; basis: view.targetBasis; orientation: view.targetOrientation
        onDragged: function(axis, amount, rotation) { view.targetDrag(axis, amount, rotation) }
    }
    Text { text: "AUBO i16H · 机械臂实时反馈"; color: "#d7e8fa"; font.pixelSize: 21; x: 16; y: 14 }
    Text { text: view.targetStatus; color: "#73d7ec"; font.pixelSize: 15; x: 16; y: 46; width: parent.width-32; wrapMode: Text.WordWrap }
    Text { text: "拖动空白旋转视角 · 滚轮缩放 · 半透明模型为目标/轨迹预览"; color: "#8fa9c2"; font.pixelSize: 18; x: 16; anchors.bottom: parent.bottom; anchors.bottomMargin: 14 }
}
