from http.server import BaseHTTPRequestHandler
from http.server import HTTPServer

class HttpSensorHandler(BaseHTTPRequestHandler):
    """Обработчик с реализованным методом do_GET."""

    def do_POST(self):
        print(f'POST! {self.path}')
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        if self.path == '/sensor':
            self.wfile.write('{"laser": {"1": 10, "2": 20, "3": 20, "4": 20, "5": 20, "6": 20}, "imu": {"roll": 0, "pitch": 0, "yaw": 0}}'.encode())
        print('POST processed')

    def do_PUT(self):
        print(f'PUT! {self.path}')
        self.send_response(200)
        self.send_header("Content-type", "application/json")
        self.end_headers()
        if self.path == '/motor':
            self.wfile.write(''.encode())
        if self.path == '/move':
            self.wfile.write(''.encode())
        print('PUT processed')


def run(server_class=HTTPServer, handler_class=HttpSensorHandler):
  server_address = ('127.0.0.1', 8000)
  httpd = server_class(server_address, handler_class)
  try:
      httpd.serve_forever()
  except KeyboardInterrupt:
      httpd.server_close()

run()
