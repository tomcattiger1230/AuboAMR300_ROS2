from pathlib import Path
def test_real_i16_meshes_and_qml_are_wired():
    package=Path(__file__).parents[2]
    qml=(package/"aubo_control_gui"/"qml"/"RobotView.qml").read_text(encoding="utf-8")
    mesh_dir=package/"aubo_student_description"/"meshes"/"aubo_i16"/"visual"
    for index in range(7):
        assert (mesh_dir/f"link{index}.DAE").is_file()
        assert f"link{index}.DAE" in qml
    assert qml.count("scale: Qt.vector3d(100,100,100)") == 7
    assert qml.count("eulerRotation.x: 90") == 8  # root-frame compensation plus wrist3 origin
