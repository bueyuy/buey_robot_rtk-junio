"""Stream de camara ZED X via HTTP MJPEG - modo liviano sin SDK.

Usa nvarguscamerasrc (ISP del Jetson) para captura directa sin ZED SDK.
Levanta en ~1s vs ~30s del SDK completo. Ambos lentes side-by-side.

Abrir http://<jetson-ip>:8089 desde cualquier browser.

Uso dentro del docker:
    python3 src/buey_robot/tools/camera_stream.py
    python3 src/buey_robot/tools/camera_stream.py --port 8089 --quality 40 --fps 15
"""

import argparse
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

# cv2 4.5.4 necesita numpy 1.x del sistema, no el 2.x de pip
# Forzar /usr/lib/python3/dist-packages primero para que use numpy 1.21
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import cv2
import numpy as np

# Pipeline GStreamer: nvargus (ISP hw) -> nvvidconv (gpu scale) -> BGR para OpenCV
# 960x600 es el modo nativo mas liviano del sensor ZED X
GST_PIPELINE = (
    "nvarguscamerasrc sensor-id={sensor_id} sensor-mode=1 "
    "! video/x-raw(memory:NVMM),width=960,height=600,framerate=120/1 "
    "! nvvidconv ! video/x-raw,format=BGRx,width={width},height={height} "
    "! videoconvert ! video/x-raw,format=BGR ! appsink drop=1"
)

HTML_PAGE = b"""<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ZED X Live</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; }
  body { background:#000; display:flex; justify-content:center; align-items:center;
         height:100vh; width:100vw; overflow:hidden; }
  img { width:100vw; height:100vh; object-fit:contain; }
</style>
</head><body>
<img src="/stream">
</body></html>"""

latest_jpeg = None
frame_lock = threading.Lock()


def capture_loop(caps, quality, fps):
    """Hilo de captura: graba frames de ambos lentes, concatena y encodea JPEG."""
    global latest_jpeg
    interval = 1.0 / fps
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    stereo = len(caps) == 2

    while all(c.isOpened() for c in caps):
        t0 = time.monotonic()

        frames = []
        for c in caps:
            ret, frame = c.read()
            if ret:
                frames.append(frame)

        if not frames:
            time.sleep(0.01)
            continue

        if stereo and len(frames) == 2:
            combined = np.hstack(frames)
        else:
            combined = frames[0]

        ok, jpg = cv2.imencode('.jpg', combined, encode_params)
        if ok:
            with frame_lock:
                latest_jpeg = jpg.tobytes()

        elapsed = time.monotonic() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)


class MJPEGHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ('/', ''):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE)
            return

        if self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        data = latest_jpeg
                    if data is not None:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(('Content-Length: %d\r\n\r\n' % len(data)).encode())
                        self.wfile.write(data)
                        self.wfile.write(b'\r\n')
                    time.sleep(0.066)  # ~15fps al browser
            except (BrokenPipeError, ConnectionResetError):
                pass
            return

        self.send_error(404)

    def log_message(self, format, *args):
        pass


def open_camera(sensor_id, width, height):
    pipeline = GST_PIPELINE.format(sensor_id=sensor_id, width=width, height=height)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    return cap


def main():
    parser = argparse.ArgumentParser(description='ZED X camera MJPEG stream (lightweight, no SDK)')
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument('--width', type=int, default=480, help='Output width per lente')
    parser.add_argument('--height', type=int, default=300, help='Output height per lente')
    parser.add_argument('--quality', type=int, default=40, help='JPEG quality 1-100')
    parser.add_argument('--fps', type=int, default=15, help='Capture FPS target')
    parser.add_argument('--left', action='store_true', help='Solo lente izquierdo')
    parser.add_argument('--right', action='store_true', help='Solo lente derecho')
    args = parser.parse_args()

    if args.left:
        sensor_ids = [0]
    elif args.right:
        sensor_ids = [1]
    else:
        sensor_ids = [1, 0]  # right=1 queda a la izq en pantalla, left=0 a la der
    caps = []

    print('Abriendo camara(s)...')
    t0 = time.monotonic()
    for i, sid in enumerate(sensor_ids):
        cap = open_camera(sid, args.width, args.height)
        if not cap.isOpened():
            print('ERROR: No se pudo abrir sensor %d. Verificar:' % sid)
            print('  - GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0')
            print('  - /dev/video%d existe' % sid)
            for c in caps:
                c.release()
            return
        # Leer un frame para confirmar que el pipeline esta estable
        ret, _ = cap.read()
        if not ret:
            print('WARN: sensor %d abrio pero no dio frame, reintentando...' % sid)
            time.sleep(1)
            ret, _ = cap.read()
        caps.append(cap)
        print('  sensor %d OK' % sid)
        # Esperar entre sensores para que nvargus-daemon estabilice
        if i < len(sensor_ids) - 1:
            time.sleep(2)
    print('Camaras listas en %.1fs' % (time.monotonic() - t0))

    mode = 'sensor %d' % sensor_ids[0] if len(sensor_ids) == 1 else 'stereo %dx%d' % (args.width * 2, args.height)
    cap_thread = threading.Thread(target=capture_loop, args=(caps, args.quality, args.fps), daemon=True)
    cap_thread.start()

    server = HTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    import socket
    try:
        ip = socket.gethostbyname(socket.gethostname())
    except socket.gaierror:
        ip = '0.0.0.0'
    print('\n  >>> http://%s:%d <<<\n' % (ip, args.port))
    print('  %s @ %dfps, JPEG q=%d' % (mode, args.fps, args.quality))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nStop')
    finally:
        server.server_close()
        for c in caps:
            c.release()


if __name__ == '__main__':
    main()
