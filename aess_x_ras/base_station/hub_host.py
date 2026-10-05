"""
hub_host.py - connects the outside-network node (hub firmware) to the command post.

Hub firmware -> here : UP_BEACON_RX (beacon + GPS already translated by the firmware), UP_TARGET_REACHED,
                       UP_MISSION_REQ, UP_NODE_STATUS
Here -> hub firmware : DN_SET_CALIB (entrance GPS + bearing), DN_MISSION_PUSH (the command post's
                       priority-sorted target list, so the hub can hand it to the Executor over the radio)
"""
import struct

from . import paths  # noqa: F401
from . import link as L
from outside_network import GPSCoord


class HubHost:
    def __init__(self, transport, command_post, entry_lat, entry_lon, heading_deg):
        self.tp, self.cp = transport, command_post
        self.parser = L.FrameParser()
        self.calib = (entry_lat, entry_lon, heading_deg)
        self.nodes = {}                  # node id -> last NODE_STATUS dict
        self.reached = []
        self.rx_log = []
        self._sent_calib = False

    def _send(self, frame):
        self.tp.write(frame)

    def send_calibration(self):
        lat, lon, hdg = self.calib
        self._send(L.encode(L.DN_SET_CALIB, struct.pack(L.CALIB_FMT, int(round(lat * 1e7)), int(round(lon * 1e7)), hdg)))
        self._sent_calib = True

    def push_mission(self):
        targets = [b for b, _ in self.cp.build_mission()][:8]
        payload = struct.pack("<B", len(targets)) + b"".join(L.pack_beacon(b) for b in targets)
        self._send(L.encode(L.DN_MISSION_PUSH, payload))

    def poll(self):
        if not self._sent_calib:
            self.send_calibration()
        for mtype, p in self.parser.feed(self.tp.read()):
            if mtype == L.UP_BEACON_RX:
                src, linkq = p[0], p[1]
                beacon = L.unpack_beacon(p[2:18])
                lat_e7, lon_e7 = struct.unpack("<ii", p[18:26])
                gps = GPSCoord(lat_e7 / 1e7, lon_e7 / 1e7)
                self.cp.ingest(beacon, gps)
                self.rx_log.append((src, linkq, beacon, gps))
                self.push_mission()
            elif mtype == L.UP_TARGET_REACHED:
                bid = p[0]
                self.cp.mark_visited(bid)
                self.reached.append(bid)
                self.push_mission()
            elif mtype == L.UP_NODE_STATUS:
                node, state, x, y, batt, age, q = struct.unpack(L.NODE_STATUS_FMT, p[:struct.calcsize(L.NODE_STATUS_FMT)])
                self.nodes[node] = dict(state=state, x_cm=x, y_cm=y, battery_mv=batt, link_q=q)
            elif mtype == L.UP_MISSION_REQ:
                self.push_mission()
