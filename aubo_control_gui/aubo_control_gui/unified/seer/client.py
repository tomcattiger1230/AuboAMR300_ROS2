from __future__ import annotations

import json
import os
import platform
import socket
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional

from .protocol import ApiResult, pack_message, unpack_message

STATUS_PORT = 19204
CONTROL_PORT = 19205
NAVIGATION_PORT = 19206
CONFIG_PORT = 19207
OTHER_PORT = 19210
PUSH_PORT = 19301


class AgvClient:
    def __init__(self, ip: str = "192.168.3.250", timeout: float = 3.0, log_dir: str | Path = "logs"):
        self.ip = ip
        self.timeout = float(timeout)
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_dir / time.strftime("api_%Y%m%d_%H%M%S.jsonl")

    def set_config(self, ip: str, timeout: float) -> None:
        self.ip = ip.strip()
        self.timeout = float(timeout)

    def _log(self, record: Dict[str, Any]) -> None:
        try:
            record = dict(record)
            record["time"] = time.strftime("%Y-%m-%d %H:%M:%S")
            with self.log_file.open("a", encoding="utf-8") as f:
                f.write(json.dumps(record, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def ping(self) -> Dict[str, Any]:
        system = platform.system().lower()
        if "windows" in system:
            # Windows ping: -w 等待毫秒
            cmd = ["ping", "-n", "1", "-w", str(int(self.timeout * 1000)), self.ip]
        elif system == "darwin":
            # macOS BSD ping: -W 是毫秒（与 Linux iputils 不同！Linux -W 是秒）
            # https://www.unix.com/man-page/osx/8/ping/
            cmd = ["ping", "-c", "1", "-W", str(int(self.timeout * 1000)), self.ip]
        else:
            # Linux iputils ping: -W 是秒
            cmd = ["ping", "-c", "1", "-W", str(max(1, int(self.timeout))), self.ip]
        t0 = time.time()
        try:
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=self.timeout + 2)
            ok = proc.returncode == 0
            res = {
                "ok": ok,
                "cmd": " ".join(cmd),
                "returncode": proc.returncode,
                "elapsed_ms": round((time.time() - t0) * 1000, 2),
                "stdout": proc.stdout[-2000:],
                "stderr": proc.stderr[-2000:],
            }
        except Exception as e:
            res = {"ok": False, "cmd": " ".join(cmd), "error": f"{type(e).__name__}: {e}"}
        self._log({"kind": "ping", **res})
        return res

    def check_port(self, port: int) -> Dict[str, Any]:
        t0 = time.time()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect((self.ip, port))
            res = {"ok": True, "ip": self.ip, "port": port, "elapsed_ms": round((time.time() - t0) * 1000, 2)}
        except Exception as e:
            res = {"ok": False, "ip": self.ip, "port": port, "error": f"{type(e).__name__}: {e}", "elapsed_ms": round((time.time() - t0) * 1000, 2)}
        finally:
            try:
                sock.close()
            except Exception:
                pass
        self._log({"kind": "check_port", **res})
        return res

    def check_ports(self) -> Dict[str, Any]:
        ports = {
            "status_19204": STATUS_PORT,
            "control_19205": CONTROL_PORT,
            "navigation_19206": NAVIGATION_PORT,
            "config_19207": CONFIG_PORT,
        }
        return {name: self.check_port(port) for name, port in ports.items()}

    def request(self, port: int, msg_type: int, payload: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None) -> ApiResult:
        if payload is None:
            payload = {}
        timeout = self.timeout if timeout is None else float(timeout)
        t0 = time.time()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            sock.connect((self.ip, port))
            msg = pack_message(1, msg_type, payload)
            sock.sendall(msg)
            resp, meta = unpack_message(sock)
            ok = resp.get("ret_code", 0) == 0
            result = ApiResult(ok=ok, ip=self.ip, port=port, msg_type=msg_type, payload=payload, response=resp, elapsed_ms=round((time.time() - t0) * 1000, 2), response_meta=meta)
        except Exception as e:
            result = ApiResult(ok=False, ip=self.ip, port=port, msg_type=msg_type, payload=payload, response={}, error=f"{type(e).__name__}: {e}", elapsed_ms=round((time.time() - t0) * 1000, 2))
        finally:
            try:
                sock.close()
            except Exception:
                pass
        self._log({"kind": "api", **result.to_json()})
        return result

    # 状态 API
    def get_position(self) -> ApiResult:
        return self.request(STATUS_PORT, 1004, {})

    def get_speed(self) -> ApiResult:
        return self.request(STATUS_PORT, 1005, {})

    def get_navigation_status(self, simple: bool = False) -> ApiResult:
        return self.request(STATUS_PORT, 1020, {"simple": True} if simple else {})

    def get_task_status(self) -> ApiResult:
        return self.request(STATUS_PORT, 1110, {})

    def get_reloc_status(self) -> ApiResult:
        return self.request(STATUS_PORT, 1021, {})

    def get_loadmap_status(self) -> ApiResult:
        return self.request(STATUS_PORT, 1022, {})

    def get_alarm_status(self) -> ApiResult:
        return self.request(STATUS_PORT, 1050, {})

    def get_estop_status(self) -> ApiResult:
        return self.request(STATUS_PORT, 1012, {})

    def get_maps_info(self) -> ApiResult:
        return self.request(STATUS_PORT, 1300, {})

    def get_stations_info(self) -> ApiResult:
        return self.request(STATUS_PORT, 1301, {})

    # 配置 API
    def download_map(self, map_name: str) -> ApiResult:
        return self.request(CONFIG_PORT, 4011, {"map_name": map_name}, timeout=max(20, self.timeout))

    def upload_map_json(self, map_json: Dict[str, Any]) -> ApiResult:
        return self.request(CONFIG_PORT, 4010, map_json, timeout=max(180, self.timeout))

    # 定位/重定位控制 API
    def relocalize_auto(self) -> ApiResult:
        return self.request(CONTROL_PORT, 2002, {"isAuto": True}, timeout=max(10, self.timeout))

    def relocalize_manual(self, x: float, y: float, angle: float, length: float | None = None, home: bool | None = None) -> ApiResult:
        payload: Dict[str, Any] = {"x": float(x), "y": float(y), "angle": float(angle)}
        if length is not None:
            payload["length"] = float(length)
        if home is not None:
            payload["home"] = bool(home)
        return self.request(CONTROL_PORT, 2002, payload, timeout=max(10, self.timeout))

    def confirm_localization(self) -> ApiResult:
        return self.request(CONTROL_PORT, 2003, {}, timeout=max(10, self.timeout))

    def cancel_relocalize(self) -> ApiResult:
        return self.request(CONTROL_PORT, 2004, {}, timeout=max(10, self.timeout))

    # 路径导航 API
    def path_navigation(self, source_id: str, target_id: str, task_id: str | None = "") -> ApiResult:
        payload: Dict[str, Any] = {"source_id": str(source_id), "id": str(target_id)}
        if task_id is not None:
            payload["task_id"] = str(task_id)
        return self.request(NAVIGATION_PORT, 3051, payload, timeout=max(10, self.timeout))

    def pause_navigation(self) -> ApiResult:
        return self.request(NAVIGATION_PORT, 3001, {}, timeout=max(10, self.timeout))

    def continue_navigation(self) -> ApiResult:
        return self.request(NAVIGATION_PORT, 3002, {}, timeout=max(10, self.timeout))

    def cancel_navigation(self) -> ApiResult:
        return self.request(NAVIGATION_PORT, 3003, {}, timeout=max(10, self.timeout))

    # 手动开环运动 API
    def open_loop_motion(self, vx: float = 0.0, vy: float = 0.0, w: float = 0.0) -> ApiResult:
        # 旧接口：控制端口 19205 / msg_type 2010，payload 直接给 vx/vy/w
        return self.request(CONTROL_PORT, 2010, {"vx": float(vx), "vy": float(vy), "w": float(w)}, timeout=max(5, self.timeout))

    def stop_open_loop_motion(self) -> ApiResult:
        # 旧接口：控制端口 19205 / msg_type 2000，payload 空 dict
        return self.request(CONTROL_PORT, 2000, {}, timeout=max(5, self.timeout))
