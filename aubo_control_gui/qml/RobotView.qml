import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils

View3D {
    id: view
    property url meshRoot
    property real j1: 0; property real j2: 0; property real j3: 0
    property real j4: 0; property real j5: 0; property real j6: 0
    environment: SceneEnvironment { backgroundMode: SceneEnvironment.Color; clearColor: "#101722"; antialiasingMode: SceneEnvironment.MSAA; antialiasingQuality: SceneEnvironment.High }
    Node {
        id: cameraRig; eulerRotation.x: -22; eulerRotation.y: -42
        PerspectiveCamera { id: camera; z: 185; clipFar: 2000 }
    }
    DirectionalLight { eulerRotation: Qt.vector3d(-45,-30,0); brightness: 1.4; castsShadow: true }
    DirectionalLight { eulerRotation: Qt.vector3d(40,140,0); brightness: 0.7 }
    Model { source: "#Cube"; position: Qt.vector3d(0,-45,0); scale: Qt.vector3d(1.5,.01,1.5); materials: PrincipledMaterial { baseColor: "#253447"; roughness: 1 } }
    Node {
        id: robot
        eulerRotation.x: -90
        position: Qt.vector3d(0,-45,0)
        RuntimeLoader { source: view.meshRoot + "/link0.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
        Node {
            position: Qt.vector3d(0,0,16.3); eulerRotation.z: 180
            Node {
                eulerRotation.z: view.j1
                RuntimeLoader { source: view.meshRoot + "/link1.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                Node {
                    position: Qt.vector3d(0,19.1,0)
                    Node {
                        eulerRotation.y: -90
                        Node {
                            eulerRotation.x: -90
                            Node {
                                eulerRotation.z: view.j2
                                RuntimeLoader { source: view.meshRoot + "/link2.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                Node {
                                    position: Qt.vector3d(48,0,0); eulerRotation.x: -180
                                    Node {
                                        eulerRotation.z: view.j3
                                        RuntimeLoader { source: view.meshRoot + "/link3.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                        Node {
                                            position: Qt.vector3d(37,0,0); eulerRotation.z: 90
                                            Node {
                                                eulerRotation.x: 180
                                                Node {
                                                    eulerRotation.z: view.j4
                                                    RuntimeLoader { source: view.meshRoot + "/link4.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                                    Node {
                                                        position: Qt.vector3d(0,11.75,0); eulerRotation.x: -90
                                                        Node {
                                                            eulerRotation.z: view.j5
                                                            RuntimeLoader { source: view.meshRoot + "/link5.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                                            Node {
                                                                position: Qt.vector3d(0,-10.35,0); eulerRotation.x: 90
                                                                Node {
                                                                    eulerRotation.z: view.j6
                                                                    RuntimeLoader { source: view.meshRoot + "/link6.DAE"; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    MouseArea {
        anchors.fill: parent; property real oldX; property real oldY
        onPressed: function(mouse) { oldX=mouse.x; oldY=mouse.y }
        onPositionChanged: function(mouse) { if (pressed) { cameraRig.eulerRotation.y += (mouse.x-oldX)*.35; cameraRig.eulerRotation.x=Math.max(-85,Math.min(85,cameraRig.eulerRotation.x+(mouse.y-oldY)*.35)); oldX=mouse.x; oldY=mouse.y } }
        onWheel: function(wheel) { camera.z=Math.max(80,Math.min(450,camera.z-wheel.angleDelta.y*.15)) }
    }
    Text { text: "AUBO i16 · URDF/DAE 实时模型"; color: "#d7e8fa"; font.pixelSize: 21; x: 16; y: 14 }
    Text { text: "拖动旋转视角 · 滚轮缩放"; color: "#8fa9c2"; font.pixelSize: 18; x: 16; anchors.bottom: parent.bottom; anchors.bottomMargin: 14 }
}
