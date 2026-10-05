"""cli.py - shared bits of the real-hardware runners (run_writer.py, run_executor.py, run_hub.py)."""
import argparse
import time

from . import paths  # noqa: F401
from .transport import SerialTransport, TcpTransport


def add_transport_args(ap):
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--port", help="serial port of the ESP32, e.g. COM5 or /dev/ttyUSB0 (921600 baud)")
    g.add_argument("--tcp", help="host:port of a TCP bridge instead of a serial port")
    return ap


def open_transport(args):
    if args.port:
        return SerialTransport(args.port)
    host, port = args.tcp.rsplit(":", 1)
    return TcpTransport(host, int(port))


def loop(step, period_s=0.002):
    """Call step() until it returns False or Ctrl+C."""
    try:
        while step() is not False:
            time.sleep(period_s)
    except KeyboardInterrupt:
        print("\nstopped by user")
