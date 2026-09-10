import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils

Node {
        id: robot
    property url meshRoot
    property string meshExtension: "DAE"
    property url toolMeshRoot
    property var toolVisuals: []
    property real finger1: 0; property real finger2: 0
    property real j1: 0; property real j2: 0; property real j3: 0
    property real j4: 0; property real j5: 0; property real j6: 0

        eulerRotation.x: -90
        position: Qt.vector3d(0,-45,0)
        RuntimeLoader { source: robot.meshRoot + "/link0." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
        Node {
            position: Qt.vector3d(0,0,16.3); eulerRotation.z: 180
            Node {
                eulerRotation.z: robot.j1
                RuntimeLoader { source: robot.meshRoot + "/link1." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                Node {
                    position: Qt.vector3d(0,19.1,0)
                    Node {
                        eulerRotation.y: -90
                        Node {
                            eulerRotation.x: -90
                            Node {
                                eulerRotation.z: robot.j2
                                RuntimeLoader { source: robot.meshRoot + "/link2." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                Node {
                                    position: Qt.vector3d(48,0,0); eulerRotation.x: -180
                                    Node {
                                        eulerRotation.z: robot.j3
                                        RuntimeLoader { source: robot.meshRoot + "/link3." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                        Node {
                                            position: Qt.vector3d(37,0,0); eulerRotation.z: 90
                                            Node {
                                                eulerRotation.x: 180
                                                Node {
                                                    eulerRotation.z: robot.j4
                                                    RuntimeLoader { source: robot.meshRoot + "/link4." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                                    Node {
                                                        position: Qt.vector3d(0,11.75,0); eulerRotation.x: -90
                                                        Node {
                                                            eulerRotation.z: robot.j5
                                                            RuntimeLoader { source: robot.meshRoot + "/link5." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                                            Node {
                                                                position: Qt.vector3d(0,-10.35,0); eulerRotation.x: 90
                                                                Node {
                                                                    eulerRotation.z: robot.j6
                                                                    RuntimeLoader { source: robot.meshRoot + "/link6." + robot.meshExtension; scale: Qt.vector3d(100,100,100); eulerRotation.x: 90 }
                                                                    ToolPreview { meshRoot: robot.toolMeshRoot; visuals: robot.toolVisuals; finger1: robot.finger1; finger2: robot.finger2 }
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
