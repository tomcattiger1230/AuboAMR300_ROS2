#!/usr/bin/env python3
"""从 hf-mirror.com 下载 ROI-1555 钢筋检测/实例分割数据集(镜像安全版)。

数据集: tsrobcvai/ROI-1555_Rebar_Detection_and_Instance_Segmentation_Dataset
论文:  Sun et al. 2025, Advanced Engineering Informatics 65:103224.
内容:  1555 张钢筋图片, LabelMe 格式标注(多边形掩码), 含直条(straight)
       与箍筋(hoop)两类, 多场景多视角(含侧面)。

为什么不用 huggingface_hub.snapshot_download:
  本仓 3000+ 文件, HF API 分页的 Link 头会指向 huggingface.co 原始地址,
  直连环境跟随即被墙; 本脚本把列表与下载全部钉死在镜像域名上。

用法:
    python scripts/01_download_roi1555.py [--out datasets/roi1555_raw] [-j 8]
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from urllib.parse import urlparse

import requests

MIRROR = os.environ.get("HF_ENDPOINT", "https://hf-mirror.com").rstrip("/")
REPO_ID = "tsrobcvai/ROI-1555_Rebar_Detection_and_Instance_Segmentation_Dataset"
# IDE 配置与论文插图目录不需要
SKIP_PREFIXES = (".idea/", "doc/")


def pin_mirror(url: str) -> str:
    """把任何 huggingface.co 域名替换为镜像, 其余原样返回。"""
    parsed = urlparse(url)
    if parsed.hostname in ("huggingface.co", "hf.co"):
        return f"{MIRROR}{parsed.path}" + (
            f"?{parsed.query}" if parsed.query else ""
        )
    return url


def list_files(session: requests.Session) -> list[dict]:
    """递归列出仓库全部文件, 手工跟随分页 Link 并钉在镜像上。"""
    url: str | None = (
        f"{MIRROR}/api/datasets/{REPO_ID}/tree/main"
        "?recursive=true&expand=false&limit=1000"
    )
    entries: list[dict] = []
    while url:
        resp = session.get(url, timeout=60)
        resp.raise_for_status()
        page = resp.json()
        entries.extend(e for e in page if e.get("type") == "file")
        link = resp.links.get("next", {}).get("url")
        url = pin_mirror(link) if link else None
    return entries


def download_one(
    session: requests.Session, entry: dict, out_dir: Path, delay: float = 0.0
) -> tuple[str, str]:
    rel: str = entry["path"]
    size: int = int(entry.get("size", 0))
    dest = out_dir / rel
    if dest.exists() and dest.stat().st_size == size:
        return rel, "skip"
    if delay:
        time.sleep(delay * (0.5 + os.urandom(1)[0] / 255))  # 抖动避免同步打点
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    url = f"{MIRROR}/datasets/{REPO_ID}/resolve/main/{rel}"
    for attempt in range(5):
        try:
            with session.get(url, timeout=120, stream=True,
                             allow_redirects=True) as r:
                r.raise_for_status()
                with open(tmp, "wb") as f:
                    for chunk in r.iter_content(1 << 20):
                        f.write(chunk)
            if size and tmp.stat().st_size != size:
                raise IOError(
                    f"size mismatch {tmp.stat().st_size} != {size}")
            tmp.replace(dest)
            return rel, "ok"
        except Exception as e:  # noqa: BLE001 重试所有网络/IO 异常
            if attempt == 4:
                return rel, f"FAIL: {e}"
            time.sleep(2 * (attempt + 1))
    return rel, "unreachable"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default="datasets/roi1555_raw")
    parser.add_argument("-j", "--jobs", type=int, default=8)
    parser.add_argument("--delay", type=float, default=0.0,
                        help="每个下载前的等待秒数(限流时用, 如 1.5)")
    args = parser.parse_args()

    out_dir = Path(args.out).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    session = requests.Session()
    session.headers["User-Agent"] = "aubo-object-detection/1.0"

    print(f"[download] repo: {REPO_ID}")
    print(f"[download] mirror: {MIRROR}")
    entries = list_files(session)
    wanted = [e for e in entries if not e["path"].startswith(SKIP_PREFIXES)]
    total = sum(int(e.get("size", 0)) for e in wanted)
    print(f"[download] 文件 {len(wanted)} 个 (全仓 {len(entries)}, "
          f"已剔除 {'/'.join(SKIP_PREFIXES)}), 共 {total / 1e6:.1f} MB")

    ok = skip = failed = 0
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = {pool.submit(download_one, session, e, out_dir, args.delay): e
                   for e in wanted}
        for i, fut in enumerate(as_completed(futures), 1):
            rel, status = fut.result()
            ok += status == "ok"
            skip += status == "skip"
            failed += status.startswith("FAIL")
            if i % 100 == 0 or status.startswith("FAIL"):
                print(f"[download] {i}/{len(wanted)} "
                      f"(ok={ok} skip={skip} fail={failed}) {status!r} {rel}")
    print(f"[download] 完成: ok={ok} skip={skip} fail={failed} -> {out_dir}")
    if failed:
        sys.exit(1)


if __name__ == "__main__":
    main()
