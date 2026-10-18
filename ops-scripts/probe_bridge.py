# -*- coding: utf-8 -*-
"""地面台桥接层实机联测探针：连 WS → 发起无人机连接 → 收约 60~80s 快照 →
按"rows 最多的已连接窗口"打印验证点（抗 NCM 链路抖动：不看最后一帧看峰值）。"""
import asyncio
import json
import sys

import websockets

async def main() -> int:
    got_dash = 0
    best = None
    events = []
    async with websockets.connect("ws://127.0.0.1:7822") as ws:
        await ws.send(json.dumps({
            "type": "connect", "host": "192.168.55.1",
            "user": "duzhong", "password": "duzhong"}))
        try:
            for _ in range(400):
                raw = await asyncio.wait_for(ws.recv(), timeout=20)
                msg = json.loads(raw)
                if msg["type"] == "dash":
                    got_dash += 1
                    if msg["connected"] and (
                            best is None or msg["rows"] > best["rows"]):
                        best = msg
                elif msg["type"] == "event":
                    events.append(f"{msg['lvl']}: {msg['text']}")
        except asyncio.TimeoutError:
            pass
    print(f"dash 快照 {got_dash} 帧")
    for e in events[:12]:
        print(" 事件:", e)
    if not best:
        print("FAIL: 全程未能连通")
        return 1
    print(f"[峰值窗口] rows={best['rows']} lag_ms={best['lag_ms']}")
    print(f"armed={best['armed']} nav={best['nav_state']} failsafe={best['failsafe']}")
    print(f"att={best['att_deg']}")
    print(f"pos={best['pos']}")
    print(f"battery={best['battery']} estimator_ok={best['estimator_ok']} clip={best['clip']}")
    print(f"dzuav={best['dzuav']}")
    ok = (best["rows"] > 40 and best["att_deg"] is not None
          and best["pos"] is not None and best["battery"] is not None)
    print("LIVE_OK" if ok else "LIVE_INCOMPLETE")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
