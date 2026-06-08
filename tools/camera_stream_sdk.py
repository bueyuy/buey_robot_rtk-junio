"""Stream de camara ZED X via HTTP MJPEG - usando ZED SDK minimo.

Abre la ZED X con el SDK pero sin depth, sin tracking, sin nada pesado.
Solo graba imagen RGB y la sirve por HTTP.

Abrir http://<jetson-ip>:8089 desde cualquier browser.

Uso dentro del docker:
    python3 src/buey_robot/tools/camera_stream_sdk.py
    python3 src/buey_robot/tools/camera_stream_sdk.py --port 8089 --quality 40 --fps 15
"""

import argparse
import socket
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

sys.path.insert(0, '/usr/lib/python3/dist-packages')

import cv2
import pyzed.sl as sl

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


def capture_loop(cam, quality, fps, view):
    """Hilo de captura: graba frames del SDK y encodea JPEG."""
    global latest_jpeg
    interval = 1.0 / fps
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
    runtime = sl.RuntimeParameters()
    import numpy as np

    if view == 'stereo':
        img_left = sl.Mat()
        img_right = sl.Mat()
    else:
        img = sl.Mat()
        sl_view = sl.VIEW.LEFT if view == 'left' else sl.VIEW.RIGHT

    while True:
        t0 = time.monotonic()
        err = cam.grab(runtime)
        if err != sl.ERROR_CODE.SUCCESS:
            time.sleep(0.01)
            continue

        if view == 'stereo':
            cam.retrieve_image(img_left, sl.VIEW.LEFT)
            cam.retrieve_image(img_right, sl.VIEW.RIGHT)
            frame = np.hstack([
                img_left.get_data()[:, :, :3],
                img_right.get_data()[:, :, :3],
            ])
        else:
            cam.retrieve_image(img, sl_view)
            frame = img.get_data()[:, :, :3]  # BGRA -> BGR

        ok, jpg = cv2.imencode('.jpg', frame, encode_params)
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


def main():
    parser = argparse.ArgumentParser(description='ZED X camera MJPEG stream (SDK minimal)')
    parser.add_argument('--port', type=int, default=8089)
    parser.add_argument('--quality', type=int, default=40, help='JPEG quality 1-100')
    parser.add_argument('--fps', type=int, default=15, help='Capture FPS target')
    parser.add_argument('--left', action='store_true', help='Solo lente izquierdo')
    parser.add_argument('--right', action='store_true', help='Solo lente derecho')
    args = parser.parse_args()

    if args.left:
        view = 'left'
    elif args.right:
        view = 'right'
    else:
        view = 'stereo'

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.SVGA        # 960x600, la mas liviana
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.NONE                # SIN depth
    init_params.sdk_verbose = 0                                # Sin logs del SDK
    init_params.enable_image_enhancement = False               # Sin post-proceso
    init_params.grab_compute_capping_fps = args.fps            # Limitar grab al FPS pedido

    print('Abriendo ZED X (SDK modo minimo, sin depth)...')
    t0 = time.monotonic()
    cam = sl.Camera()
    err = cam.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print('ERROR: %s' % err)
        print('  Verificar que ZEDX_Daemon esta corriendo')
        return
    print('Camara lista en %.1fs' % (time.monotonic() - t0))

    cap_thread = threading.Thread(target=capture_loop, args=(cam, args.quality, args.fps, view), daemon=True)
    cap_thread.start()

    server = HTTPServer(('0.0.0.0', args.port), MJPEGHandler)
    try:
        ip = socket.gethostbyname(socket.gethostname())
    except socket.gaierror:
        ip = '0.0.0.0'
    print('\n  >>> http://%s:%d <<<\n' % (ip, args.port))
    print('  SVGA %s @ %dfps, JPEG q=%d, depth=NONE' % (view, args.fps, args.quality))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nStop')
    finally:
        server.server_close()
        cam.close()


if __name__ == '__main__':
    main()
