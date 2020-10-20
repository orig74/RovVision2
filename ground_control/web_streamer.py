import http.server
from socketserver import ThreadingMixIn
from threading import Thread, Condition
from functools import partial
import urllib


class Broadcast(object):
  def __init__(self):
    self.condition = Condition()
    self.value = None

  def wait(self):
    with self.condition:
      self.condition.wait()
      return self.value

  def put(self, value):
    with self.condition:
      self.value = value
      self.condition.notify_all()


class MJPEGHandler(http.server.BaseHTTPRequestHandler):
  def __init__(self, image_buffer, *args, **kwargs):
    self.image_buffer = image_buffer
    http.server.BaseHTTPRequestHandler.__init__(self, *args, **kwargs)

  def stream_camera(self):
    self.send_response(200)
    self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=jpgboundary')
    self.end_headers()

    img = self.image_buffer.wait()
    while img is not None:
      self.wfile.write("--jpgboundary\r\n".encode("utf-8"))
      self.send_header('Content-type', 'image/jpeg')
      self.send_header('Content-length', len(img))
      self.end_headers()
      self.wfile.write(img)
      img = self.image_buffer.wait()

  def send_ok(self):
    self.send_response(200)
    self.send_header('Content-type', 'text/html')
    self.end_headers()
    self.wfile.write("OK")

  def do_GET(self):
    url = urllib.parse.urlparse(self.path)
    query = urllib.parse.parse_qs(url.query)
    if self.path.endswith('.mjpg'):
      self.stream_camera()
    print(url.path, query)


class ThreadedHTTPServer(ThreadingMixIn, http.server.HTTPServer):
  """Handle requests in a separate thread."""

def run_server(image_buffer):
  server = ThreadedHTTPServer(('', 8085), partial(MJPEGHandler, image_buffer))
  server.daemon_threads = True
  server.request_queue_size = 32
  print("server started")
  thread = Thread(target=server.serve_forever)
  thread.start()

  return server, thread

def defer(f, args):
  thread = Thread(target=f, args=args)
  thread.start()

  return thread