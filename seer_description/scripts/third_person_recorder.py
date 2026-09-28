"""Optional third-person frame capture for the rebar automation demonstration.

Set REBAR_VIDEO_DIR before launching Isaac. The stage marker file can be changed
by the workflow script while the simulation runs; each captured frame records
the active stage in a JSONL manifest for editing the final video.
"""

import json
import os
import time

from PIL import Image
import omni.replicator.core as rep
from pxr import Gf, UsdGeom


CAMERA_VIEWS = {
    "pickup": ((1.6, -3.5, 3.5), (0.3, 0.3, 0.9)),
    "navigate": ((6.8, -5.4, 5.6), (2.8, 1.6, 0.9)),
    "tester": ((8.2, 0.2, 3.5), (5.8, 3.2, 1.15)),
}


class ThirdPersonRecorder:
    def __init__(self, stage, output_dir, fps=4.0, resolution=(1280, 720)):
        self.output_dir = os.path.abspath(output_dir)
        self.marker_path = os.environ.get(
            "REBAR_VIDEO_STAGE_FILE", os.path.join(self.output_dir, "stage.txt")
        )
        self.interval = 1.0 / fps
        self.last_capture = 0.0
        self.frame_number = 0
        self.current_stage = "setup"
        self.current_view = None
        self.metadata = None
        os.makedirs(os.path.join(self.output_dir, "frames"), exist_ok=True)

        camera = UsdGeom.Camera.Define(stage, "/World/RebarObserverCamera")
        camera.CreateFocalLengthAttr(28.0)
        camera.CreateHorizontalApertureAttr(36.0)
        self.camera_transform = UsdGeom.Xformable(camera).AddTransformOp()
        self._set_view("pickup")
        render_product = rep.create.render_product(camera.GetPath().pathString, resolution)
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach([render_product])
        self.metadata = open(
            os.path.join(self.output_dir, "frames.jsonl"), "w", encoding="utf-8"
        )
        with open(os.path.join(self.output_dir, "settings.json"), "w", encoding="utf-8") as settings:
            json.dump({"fps": fps, "resolution": resolution, "views": CAMERA_VIEWS}, settings, indent=2)
        print(f"Third-person recording frames: {self.output_dir}", flush=True)

    def _set_view(self, name):
        if name == self.current_view:
            return
        eye, target = CAMERA_VIEWS[name]
        camera_to_world = Gf.Matrix4d().SetLookAt(
            Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)
        ).GetInverse()
        self.camera_transform.Set(camera_to_world)
        self.current_view = name
        print(f"Third-person camera: {name}", flush=True)

    def capture_if_due(self):
        now = time.monotonic()
        if now - self.last_capture < self.interval:
            return
        self.last_capture = now
        try:
            with open(self.marker_path, encoding="utf-8") as marker:
                stage = marker.read().strip() or "setup"
        except FileNotFoundError:
            stage = "setup"
        view = "pickup" if stage in ("setup", "01") else "navigate" if stage == "02" else "tester"
        self._set_view(view)
        data = self.annotator.get_data()
        if data is None or getattr(data, "size", 0) == 0:
            return
        frame = Image.fromarray(data[:, :, :3], "RGB")
        filename = f"{self.frame_number:06d}.jpg"
        frame.save(os.path.join(self.output_dir, "frames", filename), quality=83)
        self.metadata.write(json.dumps({
            "frame": self.frame_number, "file": filename, "stage": stage,
            "view": view, "monotonic_time": now,
        }) + "\n")
        self.metadata.flush()
        self.frame_number += 1
        if self.frame_number % 100 == 0:
            print(f"Third-person frames captured: {self.frame_number}", flush=True)

    def close(self):
        if self.metadata is not None:
            self.metadata.close()
            print(f"Third-person recording complete: {self.frame_number} frames", flush=True)
