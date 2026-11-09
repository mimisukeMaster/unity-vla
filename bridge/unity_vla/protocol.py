"""Wire format between the bridge and Unity's SimServer (protocol version 1).

Every message (both directions) is one frame:

    uint32 LE  frame_len            bytes that follow
    uint32 LE  header_len
    bytes      header (UTF-8 JSON)
    bytes      blob section (concatenated binary blobs)

The header lists blobs as {"name", "offset", "length", "dtype", "shape"} relative to the
start of the blob section.  Images are raw RGB24, row 0 = top of the image.
Requests carry {"cmd": ...}; responses carry {"ok": true|false, "error"?: str}.
See docs/protocol.md for the command list.
"""

from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass, field

import numpy as np

PROTOCOL_VERSION = 1
MAX_FRAME = 64 * 1024 * 1024

_DTYPES = {"uint8": np.uint8, "float32": np.float32, "int32": np.int32}


@dataclass
class Message:
    header: dict
    blobs: dict[str, np.ndarray] = field(default_factory=dict)


def encode(msg: Message) -> bytes:
    header = dict(msg.header)
    descs, parts, offset = [], [], 0
    for name, arr in msg.blobs.items():
        arr = np.ascontiguousarray(arr)
        dtype = arr.dtype.name
        if dtype not in _DTYPES:
            raise ValueError(f"unsupported blob dtype {dtype} for {name}")
        raw = arr.tobytes()
        descs.append(
            {"name": name, "offset": offset, "length": len(raw), "dtype": dtype, "shape": list(arr.shape)}
        )
        parts.append(raw)
        offset += len(raw)
    if descs:
        header["blobs"] = descs
    hbytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    body = struct.pack("<I", len(hbytes)) + hbytes + b"".join(parts)
    return struct.pack("<I", len(body)) + body


def decode_body(body: bytes) -> Message:
    (hlen,) = struct.unpack_from("<I", body, 0)
    header = json.loads(body[4 : 4 + hlen].decode("utf-8"))
    blob_section = memoryview(body)[4 + hlen :]
    blobs = {}
    for d in header.pop("blobs", []):
        raw = blob_section[d["offset"] : d["offset"] + d["length"]]
        arr = np.frombuffer(raw, dtype=_DTYPES[d["dtype"]]).reshape(d["shape"]).copy()
        blobs[d["name"]] = arr
    return Message(header, blobs)


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(min(n - len(buf), 1 << 20))
        if not chunk:
            raise ConnectionError("connection closed by peer")
        buf.extend(chunk)
    return bytes(buf)


def send(sock: socket.socket, msg: Message) -> None:
    sock.sendall(encode(msg))


def recv(sock: socket.socket) -> Message:
    (n,) = struct.unpack("<I", _recv_exact(sock, 4))
    if n > MAX_FRAME:
        raise ValueError(f"frame too large: {n}")
    return decode_body(_recv_exact(sock, n))
