import QtQuick
import QtQuick3D
import QtQuick3D.AssetUtils

Node {
    id: tool
    property var visuals: []
    property url meshRoot
    property real finger1: 0
    property real finger2: 0
    Repeater3D {
        model: tool.visuals
        delegate: Node {
            required property var modelData
            property real displacement: modelData.finger === 0 ? tool.finger1 : (modelData.finger === 1 ? tool.finger2 : 0)
            position: Qt.vector3d(100*(modelData.position[0]+modelData.axis[0]*displacement),
                                 100*(modelData.position[1]+modelData.axis[1]*displacement),
                                 100*(modelData.position[2]+modelData.axis[2]*displacement))
            rotation: modelData.quaternion
            RuntimeLoader {
                source: tool.meshRoot + "/" + modelData.mesh
                scale: Qt.vector3d(100*modelData.scale[0],100*modelData.scale[1],100*modelData.scale[2])
                onStatusChanged: if (status === RuntimeLoader.Error) console.warn("Tool mesh:", source, errorString)
            }
        }
    }
}
