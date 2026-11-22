"""Node `web_server`: serves the web app and bridges browsers <-> dataflow.

Inputs : tick (20 Hz), estimated_state, ee_pose, safety_status, device_status,
         battery, planner_status, recorder_status, ik_result
Outputs: ui_command (to planner, safety, recorder), ik_query (to kinematics)

The HTTP + WebSocket server (aiohttp) runs in its OWN thread with its own
asyncio loop, so a slow phone can never block the dora event loop. The two
sides exchange data through thread-safe queues only.

Multi-client rule (PROJECT_SPEC 8.3): any number of browsers may watch; exactly
one holds the control lock. Commands from the others are ignored, except STOP,
which anyone may press. The lock is released when its holder disconnects.
"""
import asyncio
import json
import queue
import socket
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from aiohttp import WSMsgType, web  # noqa: E402

from exxeta_arm.robot import PROJECT_ROOT  # noqa: E402
from exxeta_arm.runtime import Logic, load_config, run  # noqa: E402

WEBAPP_DIR = PROJECT_ROOT / "webapp"
DESCRIPTION_DIR = PROJECT_ROOT / "description"
TOPICS = ("estimated_state", "ee_pose", "safety_status", "device_status", "battery", "planner_status", "recorder_status")
ANYONE_MAY = ("stop",)          # safety: STOP works from every client


def lan_ip() -> str:
    """The address other devices on the LAN can reach us at (no packet is actually sent)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class WebServer(Logic):
    def __init__(self, config=None):
        super().__init__()
        cfg = (config or load_config()).get("web", {})
        self.port = int(cfg.get("port", 8080))
        self.token = str(cfg.get("access_token", "") or "")
        self.state = {}                      # newest message per topic
        self.to_dora = queue.Queue()         # commands from browsers
        self.clients = {}                    # id -> websocket
        self.controller = None               # id of the client holding the control lock
        self._next_id = 1
        self.loop = asyncio.new_event_loop()
        self.url = f"http://{lan_ip()}:{self.port}" + (f"/?token={self.token}" if self.token else "")
        threading.Thread(target=self._serve, daemon=True).start()
        self.print_banner()

    def print_banner(self):
        print(f"\n  exxeta-arm web app:  {self.url}\n  (same Wi-Fi/LAN: open it on your phone, or scan the code)\n")
        try:
            import qrcode
            qr = qrcode.QRCode(border=1)
            qr.add_data(self.url)
            qr.print_ascii(invert=True)
        except Exception:
            pass                             # the QR code is a convenience only

    # --------------------------------------------------------- server thread
    def _serve(self):
        asyncio.set_event_loop(self.loop)
        app = web.Application()
        app.router.add_get("/ws", self._ws)
        app.router.add_get("/", self._index)
        app.router.add_get("/api/info", self._info)
        app.router.add_static("/description", DESCRIPTION_DIR)
        app.router.add_static("/", WEBAPP_DIR)
        runner = web.AppRunner(app)
        self.loop.run_until_complete(runner.setup())
        self.loop.run_until_complete(web.TCPSite(runner, "0.0.0.0", self.port).start())
        self.loop.run_forever()

    def _allowed(self, request) -> bool:
        return not self.token or request.query.get("token") == self.token

    async def _index(self, request):
        if not self._allowed(request):
            return web.Response(status=403, text="access token required: add ?token=... to the address")
        return web.FileResponse(WEBAPP_DIR / "index.html")

    async def _info(self, request):
        return web.json_response({"url": self.url})

    async def _ws(self, request):
        if not self._allowed(request):
            return web.Response(status=403)
        ws = web.WebSocketResponse(heartbeat=5)
        await ws.prepare(request)
        cid = self._next_id
        self._next_id += 1
        self.clients[cid] = ws
        await ws.send_json({"t": "hello", "id": cid, "url": self.url})
        try:
            async for msg in ws:
                if msg.type != WSMsgType.TEXT:
                    continue
                try:
                    d = json.loads(msg.data)
                except ValueError:
                    continue
                if d.get("t") == "take_control":
                    self.controller = cid
                elif d.get("t") == "release" and self.controller == cid:
                    self.controller = None
                elif d.get("t") == "cmd" and isinstance(d.get("d"), dict):
                    if self.controller == cid or d["d"].get("cmd") in ANYONE_MAY:
                        self.to_dora.put(("ui_command", d["d"]))
                elif d.get("t") == "ik" and self.controller == cid:
                    self.to_dora.put(("ik_query", d["d"]))
        finally:
            self.clients.pop(cid, None)
            if self.controller == cid:                     # lock is released automatically on disconnect
                self.controller = None
                self.to_dora.put(("ui_command", {"cmd": "jog_joint", "vel": [0] * 6}))
        return ws

    async def _broadcast(self, text_for):
        for cid, ws in list(self.clients.items()):
            try:
                await ws.send_str(text_for(cid))
            except Exception:
                self.clients.pop(cid, None)

    # ------------------------------------------------------------- dora side
    def on_input(self, name, data):
        if name in TOPICS:
            if name == "recorder_status" and "program" not in data and "program" in self.state.get(name, {}):
                data["program"] = self.state[name]["program"]      # keep the last loaded program for the editor
            self.state[name] = data
        elif name == "ik_result":
            msg = json.dumps({"t": "ik", "d": data})
            asyncio.run_coroutine_threadsafe(self._broadcast(lambda cid: msg), self.loop)
        elif name == "tick":
            while True:                                            # forward what the browsers sent
                try:
                    out, d = self.to_dora.get_nowait()
                except queue.Empty:
                    break
                self.emit(out, d)
            snap = dict(self.state)
            n, ctrl = len(self.clients), self.controller

            def text_for(cid):
                return json.dumps({"t": "state", "d": snap, "clients": n, "controller": ctrl, "you": cid})
            asyncio.run_coroutine_threadsafe(self._broadcast(text_for), self.loop)

    def close(self):
        self.loop.call_soon_threadsafe(self.loop.stop)


if __name__ == "__main__":
    run(WebServer())
