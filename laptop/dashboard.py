"""Tiny local web dashboard: serves dashboard.html and the live arm state as JSON.

Runs in a daemon thread, so it stops automatically when arm.py exits.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

PAGE = Path(__file__).with_name("dashboard.html")


def serve(get_state, port=8765):
    """Start serving on 127.0.0.1. Returns (server, url). get_state() must return a JSON-able dict."""

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/state":
                body, ctype = json.dumps(get_state()).encode(), "application/json"
            elif self.path in ("/", "/index.html"):
                body, ctype = PAGE.read_bytes(), "text/html; charset=utf-8"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):  # keep the terminal clean
            pass

    try:
        srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)  # localhost only
    except OSError:
        srv = ThreadingHTTPServer(("127.0.0.1", 0), Handler)  # port busy: take any free one
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}/"
