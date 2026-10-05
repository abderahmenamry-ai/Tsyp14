"""transport.py - byte transports for the host link (USB serial on the real robot, TCP for tests)."""
import socket


class Transport:
    def read(self) -> bytes:        # return whatever bytes are available now (non-blocking), maybe b""
        raise NotImplementedError

    def write(self, data: bytes):
        raise NotImplementedError

    def close(self):
        pass


class SerialTransport(Transport):
    """USB serial to the ESP32 (921600 8N1). Needs `pip install pyserial`."""

    def __init__(self, port: str, baud: int = 921600):
        import serial                                   # imported lazily: not needed for simulation
        self.ser = serial.Serial(port, baud, timeout=0)

    def read(self):
        n = self.ser.in_waiting
        return self.ser.read(n) if n else b""

    def write(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()


class TcpTransport(Transport):
    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port))
        self.sock.setblocking(False)

    def read(self):
        try:
            return self.sock.recv(65536)
        except BlockingIOError:
            return b""

    def write(self, data):
        self.sock.sendall(data)

    def close(self):
        self.sock.close()
