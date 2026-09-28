#!/usr/bin/env python3
"""Expose the recorder's atomic latest JPEG for the Isaac GUI observer."""
import argparse
import time
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8081)
    args = parser.parse_args(); directory = Path(args.directory)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass

        def do_GET(self):
            if self.path.split('?')[0] != '/snapshot.jpg':
                self.send_error(404); return
            try:
                image = directory/'latest.jpg'
                if time.time()-image.stat().st_mtime > 4:
                    self.send_error(503, 'No fresh Isaac frame'); return
                frame = image.read_bytes()
                marker = directory/'stage.txt'
                stage = marker.read_text().strip() if marker.exists() else 'setup'
                if stage not in ('setup','01','02','03','04','05','06','07','08','complete','failed'):
                    stage='setup'
            except OSError:
                self.send_error(503, 'Waiting for Isaac frame'); return
            self.send_response(200)
            self.send_header('Content-Type','image/jpeg')
            self.send_header('Content-Length',str(len(frame)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Isaac-Stage',stage)
            self.end_headers()
            try: self.wfile.write(frame)
            except (BrokenPipeError,ConnectionResetError): pass

    server = ThreadingHTTPServer((args.host,args.port),Handler)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: server.server_close()


if __name__ == '__main__': main()
