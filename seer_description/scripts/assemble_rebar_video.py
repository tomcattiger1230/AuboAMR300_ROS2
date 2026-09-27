#!/usr/bin/env python3
"""Turn Isaac third-person JPEG frames into a labeled workflow video."""

import argparse
import json
from pathlib import Path
import subprocess


STAGES = [
    ("01", "取筋并装车", 10),
    ("02", "载筋导航", 10),
    ("03", "车载取筋", 7),
    ("04", "竖直化钢筋", 7),
    ("05", "测试台前预定位", 7),
    ("06", "竖直持筋接近", 7),
    ("07", "夹持线微调", 7),
    ("08", "测试机接管并松爪", 7),
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    rows = [json.loads(line) for line in (args.record_dir / "frames.jsonl").read_text().splitlines()]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    work = args.record_dir / "encoded"
    work.mkdir(exist_ok=True)
    font = subprocess.check_output(
        ["fc-match", "-f", "%{file}", "Noto Sans CJK SC"], text=True
    ).strip()
    clips = []
    manifest = []
    for stage, label, input_fps in STAGES:
        frames = [row["frame"] for row in rows
                  if row["stage"] == stage or (stage == "08" and row["stage"] == "complete")]
        if not frames:
            raise RuntimeError(f"Stage {stage} has no captured frames")
        start, end = frames[0], frames[-1]
        if len(frames) != end - start + 1:
            raise RuntimeError(f"Stage {stage} frames are not contiguous")
        clip = work / f"stage_{stage}.mp4"
        label_text = f"{stage}  {label}"
        # Use a solid translucent bar so labels remain legible on bright warehouse walls.
        vf = (
            "drawbox=x=18:y=18:w=400:h=62:color=black@0.72:t=fill,"
            f"drawtext=fontfile={font}:text='{label_text}':"
            "fontcolor=white:fontsize=30:x=34:y=31,"
            "fps=24,format=yuv420p"
        )
        subprocess.run([
            "ffmpeg", "-y", "-loglevel", "error", "-framerate", str(input_fps),
            "-start_number", str(start), "-i", str(args.record_dir / "frames" / "%06d.jpg"),
            "-t", str(len(frames) / input_fps), "-vf", vf, "-an", "-c:v", "libx264",
            "-preset", "veryfast", "-crf", "21", "-pix_fmt", "yuv420p", str(clip),
        ], check=True)
        clips.append(clip)
        manifest.append({"stage": stage, "label": label, "frames": len(frames),
                         "playback_seconds": round(len(frames) / input_fps, 2)})
        print(f"{stage} {label}: {len(frames)} frames, {len(frames)/input_fps:.1f}s")

    concat = work / "concat.txt"
    concat.write_text("".join(f"file '{clip}'\n" for clip in clips))
    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0",
        "-i", str(concat), "-c", "copy", "-movflags", "+faststart", str(args.output)
    ], check=True)
    (args.output.with_suffix(".json")).write_text(
        json.dumps({"video": args.output.name, "stages": manifest}, ensure_ascii=False, indent=2)
    )
    print(args.output)


if __name__ == "__main__":
    main()
