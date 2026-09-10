from pathlib import Path
import struct
def test_real_i16_meshes_and_qml_are_wired():
    package=Path(__file__).parents[2]
    qml=(package/"aubo_control_gui"/"qml"/"ArmModel.qml").read_text(encoding="utf-8")
    mesh_dir=package/"aubo_student_description"/"meshes"/"aubo_i16"/"visual"
    for index in range(7):
        assert (mesh_dir/f"link{index}.DAE").is_file()
        assert f'"/link{index}." + robot.meshExtension' in qml
        glb = (mesh_dir/f"link{index}.glb").read_bytes()
        magic, version, length = struct.unpack('<4sII', glb[:12])
        assert (magic, version, length) == (b'glTF', 2, len(glb))
    assert 'property string meshExtension: "DAE"' in qml
    assert qml.count("scale: Qt.vector3d(100,100,100)") == 7
    assert qml.count("eulerRotation.x: 90") == 8  # root-frame compensation plus wrist3 origin
