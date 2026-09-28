from __future__ import annotations

import json
import socket
import struct
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

PACK_FMT_STR = "!BBHLH6s"
HEADER_SIZE = 16


class AgvProtocolError(RuntimeError):
    pass


def pack_message(req_id: int, msg_type: int, payload: Optional[Dict[str, Any]] = None) -> bytes:
    if payload is None:
        payload = {}
    if payload:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    else:
        body = b""
    header = struct.pack(PACK_FMT_STR, 0x5A, 0x01, req_id, len(body), msg_type, b"\x00\x00\x00\x00\x00\x00")
    return header + body


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = []
    got = 0
    while got < size:
        chunk = sock.recv(size - got)
        if not chunk:
            raise ConnectionError(f"socket closed while receiving {size} bytes, got {got}")
        chunks.append(chunk)
        got += len(chunk)
    return b"".join(chunks)


def unpack_message(sock: socket.socket) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    header_bytes = recv_exact(sock, HEADER_SIZE)
    try:
        sync1, sync2, req_id, json_len, msg_type, reserved = struct.unpack(PACK_FMT_STR, header_bytes)
    except Exception as e:
        raise AgvProtocolError(f"协议头解析失败: {type(e).__name__}: {e}") from e
    if sync1 != 0x5A or sync2 != 0x01:
        raise AgvProtocolError(f"协议头同步字错误: sync1={sync1:#x}, sync2={sync2:#x}, raw={header_bytes.hex(' ')}")
    meta = {"req_id": req_id, "json_len": json_len, "msg_type": msg_type}
    if json_len <= 0:
        return {"ret_code": 0}, meta
    body = recv_exact(sock, json_len)
    try:
        text = body.decode("utf-8")
    except UnicodeDecodeError:
        text = body.decode("ascii", errors="replace")
    try:
        data = json.loads(text)
        if isinstance(data, dict):
            return data, meta
        return {"ret_code": 0, "data": data}, meta
    except Exception as e:
        raise AgvProtocolError(
            f"响应 JSON 解析失败: {type(e).__name__}: {e}; 前500字符={text[:500]!r}"
        ) from e


@dataclass
class ApiResult:
    ok: bool
    ip: str
    port: int
    msg_type: int
    payload: Dict[str, Any]
    response: Dict[str, Any]
    error: str = ""
    elapsed_ms: float = 0.0
    response_meta: Optional[Dict[str, Any]] = None

    def to_json(self) -> Dict[str, Any]:
        return {
            "ok": self.ok,
            "ip": self.ip,
            "port": self.port,
            "msg_type": self.msg_type,
            "payload": self.payload,
            "response": self.response,
            "error": self.error,
            "elapsed_ms": self.elapsed_ms,
            "response_meta": self.response_meta or {},
        }
