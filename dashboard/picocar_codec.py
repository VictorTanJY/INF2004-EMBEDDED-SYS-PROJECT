"""PicoCar IS23 binary MQTT frames (Change Requirement 1, minimum-bit data).

The firmware sends every MQTT payload as bit fields, each only as wide as
its valid range, written least significant bit first (app_program/
app_pack.h, layouts in app_program/subsystem1/comms_proto.h).  This module
decodes them into the same dictionaries the dashboard page uses, and
encodes commands.  Pure Python 3.9+, no packages.  Tested by
testing/subsystem1/host/test_codec.py against the C encoder's bytes.
"""

VERSION = 1
T_TELEMETRY, T_EVENT, T_STATUS, T_ACK, T_CMD = 1, 2, 3, 4, 5

SYMBOLS = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%'
STATES = ('BOOT', 'IDLE', 'CALIBRATING', 'LINE_FOLLOW', 'EXECUTE_NAV', 'OBSTACLE_HANDLING', 'AVOIDING',
          'LINE_RECOVERY', 'LINE_SEARCH', 'MISSION_COMPLETE', 'FAULT')
EVENTS = ('NONE', 'BTN_START_STOP', 'BTN_CALIBRATE', 'BTN_RESET', 'REMOTE_START', 'REMOTE_STOP', 'REMOTE_RESET',
          'REMOTE_SET', 'BARCODE', 'JUNCTION', 'LINE_LOST', 'LINE_FOUND', 'MOTION_DONE', 'MOTION_FAILED',
          'OBSTACLE', 'SLOW_DOWN', 'PROFILE_DONE', 'CONTINUE', 'BYPASS_DONE', 'RECOVERY_DONE', 'RECOVERY_FAILED',
          'BLOCKED_TIMEOUT', 'HUMP_START', 'HUMP_END', 'MOTION_CLASS', 'FAULT', 'CAL_DONE', 'REMOTE_MAP', 'SAFETY_OK')
ERRORS = ('', 'bad_frame', 'no_id', 'bad_cmd', 'bad_param', 'range', 'busy', 'full', 'id_reused',
          'not_calibrated', 'bad_map')
# "set" parameters, in the firmware whitelist order (frame index = position).
PARAMS = ('speed_mm_s', 'slow_cm', 'stop_cm', 'lost_mm', 'junction_mm',
          'bench_move', 'bench_turn', 'bench_speed', 'bench_servo')
COMMANDS = {'start': 1, 'stop': 2, 'reset': 4, 'set': 5, 'map': 6}
MAP_KEYS = ('left', 'right', 'straight', 'uturn')
EV_BARCODE = 8


class Reader:
    """Reads fields LSB first, as pk_take() does."""

    def __init__(self, data):
        self.data, self.pos = bytes(data), 0

    def take(self, width):
        if self.pos + width > 8 * len(self.data):
            raise ValueError('frame too short')
        v = 0
        for i in range(width):
            if self.data[(self.pos + i) >> 3] >> ((self.pos + i) & 7) & 1:
                v |= 1 << i
        self.pos += width
        return v

    def off(self, width):
        return self.take(width) - (1 << (width - 1))

    def signed(self, width):
        v = self.take(width)
        return v - (1 << width) if v >> (width - 1) else v


class Writer:
    def __init__(self):
        self.bits = []

    def put(self, value, width):
        self.bits += [(value >> i) & 1 for i in range(width)]

    def bytes(self):
        out = bytearray((len(self.bits) + 7) // 8)
        for i, b in enumerate(self.bits):
            out[i >> 3] |= b << (i & 7)
        return bytes(out)


def symbol(i):
    return SYMBOLS[i - 1] if 1 <= i <= len(SYMBOLS) else None


def header(data):
    """(type, version) of a frame, or (None, None) if it is not one."""
    if not data:
        return None, None
    return data[0] & 0x1F, data[0] >> 5


class Unwrap:
    """Rebuilds a running count from its wrapping low bits."""

    def __init__(self, width):
        self.width, self.value, self.raw = width, None, None

    def update(self, raw):
        if self.value is None:
            self.value = raw - (1 << self.width) if raw >> (self.width - 1) else raw
        else:
            d = (raw - self.raw) & ((1 << self.width) - 1)
            self.value += d - (1 << self.width) if d >> (self.width - 1) else d
        self.raw = raw
        return self.value


class Decoder:
    """One per robot: keeps the unwrap state of the 16-bit counters."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.enc = [Unwrap(16), Unwrap(16)]
        self.dist = Unwrap(16)
        self.last_t = None

    def telemetry(self, data):
        r = Reader(data)
        r.take(8)
        seq, t = r.take(8), r.take(24) * 10
        if self.last_t is not None and t < self.last_t and self.last_t - t < (1 << 23) * 10:
            self.reset()                    # time went back: the robot restarted
        self.last_t = t
        state = r.take(4)
        vl, vr = r.off(10), r.off(10)
        el, er, d = r.take(16), r.take(16), r.take(16)
        err, lost, bc = r.off(11), bool(r.take(1)), r.take(10)
        barcode, nav = symbol(r.take(6)), symbol(r.take(6))
        pitch, roll, hump = r.off(11) / 10, r.off(12) / 10, r.take(8)
        d_cm, w_mm, cl, cr = r.take(9), r.take(10), r.take(7), r.take(7)
        prof = w_mm != 1023
        return {'seq': seq, 't_ms': t, 'state': STATES[state] if state < len(STATES) else 'UNKNOWN',
                'v_mm_s': [vl, vr], 'enc': [self.enc[0].update(el), self.enc[1].update(er)],
                'dist_mm': self.dist.update(d), 'line': {'err': err, 'lost': lost, 'bc': bc},
                'barcode': barcode, 'pitch_deg': pitch, 'roll_deg': roll, 'hump_max_mm': hump,
                'obstacle': {'d_cm': None if d_cm == 511 else d_cm,
                             'w_cm': round(w_mm / 10) if prof else None, 'w_mm': w_mm if prof else None,
                             'clr_l_cm': None if cl == 127 else cl, 'clr_r_cm': None if cr == 127 else cr},
                'nav': nav, 'frame_bytes': len(data)}

    @staticmethod
    def event(data):
        r = Reader(data)
        r.take(8)
        seq, t = r.take(8), r.take(24) * 10
        kind, src, code, dlen = r.take(2), r.take(3), r.take(7), r.take(2)
        value = r.signed((0, 8, 16, 32)[dlen]) if dlen else 0
        p = {'seq': seq, 't_ms': t}
        if kind == 0:
            p['type'] = EVENTS[code] if code < len(EVENTS) else 'UNKNOWN'
            p['data'] = chr(value) if code == EV_BARCODE and 0x20 <= value < 0x7F else value
        elif kind == 1:
            p.update(type='FAULT', data=value)
        elif kind == 2:
            p.update(type='STATE', data=STATES[code] if code < len(STATES) else 'UNKNOWN')
        else:
            p.update(type='LOG', data=value, code=code)
        p['src'] = src
        return p

    @staticmethod
    def status(data):
        if len(data) == 1:
            return {'online': False}         # the Last Will
        r = Reader(data)
        r.take(8)
        p = {'online': True, 'uptime_s': r.take(17), 'rssi': -r.take(7), 'drops': r.take(8), 'map': None}
        if r.take(1):
            p['map'] = {k: symbol(r.take(6)) for k in MAP_KEYS}
        return p

    @staticmethod
    def ack(data):
        r = Reader(data)
        r.take(8)
        has_id, cid, err = r.take(1), r.take(8), r.take(4)
        p = {'id': cid if has_id else None, 'ok': err == 0}
        if err:
            p['err'] = ERRORS[err] if err < len(ERRORS) else 'unknown'
        return p

    def decode(self, data):
        """Dictionary for one payload, or None if it is not a v1 frame."""
        kind, version = header(data)
        if version != VERSION:
            return None
        try:
            return {T_TELEMETRY: self.telemetry, T_EVENT: self.event, T_STATUS: self.status,
                    T_ACK: self.ack}[kind](data)
        except (KeyError, ValueError, IndexError):
            return None


def encode_command(cid, body):
    """Binary command frame for {'cmd': ..., 'param': ..., 'value': ...} or a map."""
    cmd = body['cmd']
    w = Writer()
    w.put(T_CMD, 5)
    w.put(VERSION, 3)
    w.put(cid & 0xFF, 8)
    w.put(COMMANDS[cmd], 3)
    if cmd == 'set':
        w.put(PARAMS.index(body['param']), 4)
        w.put((int(body['value']) + 32768) & 0xFFFF, 16)
    elif cmd == 'map':
        for k in MAP_KEYS:
            w.put(SYMBOLS.index(body[k]) + 1, 6)
    return w.bytes()


def main(argv):
    """Command-line use, for the manual tests without the dashboard:

    decode:  mosquitto_sub -h <broker> -t 'picocar/is23/#' -F '%t %x' | python3 picocar_codec.py
    command: python3 picocar_codec.py cmd <id> start|stop|reset > cmd.bin
             python3 picocar_codec.py cmd <id> set <param> <value> > cmd.bin
             python3 picocar_codec.py cmd <id> map <left> <right> <straight> <uturn> > cmd.bin
             mosquitto_pub -h <broker> -t picocar/is23/cmd -q 1 -f cmd.bin
    """
    import json, sys
    if len(argv) >= 3 and argv[0] == 'cmd':
        cid, cmd, rest = int(argv[1]), argv[2], argv[3:]
        body = {'cmd': cmd}
        if cmd == 'set':
            body.update(param=rest[0], value=int(rest[1]))
        elif cmd == 'map':
            body.update(dict(zip(MAP_KEYS, [c.upper() for c in rest[:4]])))
        sys.stdout.buffer.write(encode_command(cid, body))
        return 0
    dec = Decoder()
    for line in sys.stdin:
        try:
            topic, raw = line.split()
            p = dec.decode(bytes.fromhex(raw))
        except ValueError:
            continue
        print(topic, json.dumps(p) if p is not None else '(not a v1 frame) ' + raw, flush=True)
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv[1:]))
