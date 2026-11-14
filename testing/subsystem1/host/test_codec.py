"""S1-H21: the laptop codec (dashboard/picocar_codec.py) against the C encoder.

The golden frames in golden.h were produced by comms_proto.c; here they are
decoded by the independent Python implementation and must give back the
inputs used in test_s1.c, and the Python command encoder must produce the
same bytes as the C test's encoder.  Run: python3 test_codec.py
"""
import pathlib, re, sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[2] / 'dashboard'))
import picocar_codec as codec  # noqa: E402

GOLD = dict(re.findall(r'#define (GOLD_\w+)\s+"([0-9a-f]+)"', (HERE / 'golden.h').read_text()))
passed = failed = 0


def check(cond, what):
    global passed, failed
    if cond:
        passed += 1
    else:
        failed += 1
        print('FAIL S1-H21', what)


def frame(name):
    return bytes.fromhex(GOLD[name])


d = codec.Decoder()
t = d.decode(frame('GOLD_TELEMETRY'))
check(t == {'seq': 12, 't_ms': 3400, 'state': 'LINE_FOLLOW', 'v_mm_s': [250, -10], 'enc': [1000, 990],
            'dist_mm': 555, 'line': {'err': -120, 'lost': False, 'bc': 0}, 'barcode': 'A', 'pitch_deg': -12.3,
            'roll_deg': 8.8, 'hump_max_mm': 25,
            'obstacle': {'d_cm': 38, 'w_cm': None, 'w_mm': None, 'clr_l_cm': None, 'clr_r_cm': None},
            'nav': None, 'frame_bytes': 27}, f'telemetry {t}')
check(d.decode(frame('GOLD_EVENT_BARCODE')) == {'seq': 5, 't_ms': 770, 'type': 'BARCODE', 'data': 'B', 'src': 3},
      'barcode event')
check(d.decode(frame('GOLD_EVENT_LOG')) == {'seq': 8, 't_ms': 770, 'type': 'LOG', 'data': 0x12345678, 'code': 81,
                                            'src': 3}, 'log event')
check(d.decode(frame('GOLD_STATUS')) == {'online': True, 'uptime_s': 61, 'rssi': -58, 'drops': 3, 'map': None},
      'status')
check(d.decode(frame('GOLD_STATUS_MAP'))['map'] == {'left': 'Q', 'right': 'F', 'straight': 'Z', 'uturn': 'W'},
      'status map')
check(d.decode(bytes([0x23])) == {'online': False}, 'Last Will = offline')
check(d.decode(frame('GOLD_ACK_OK')) == {'id': 42, 'ok': True}, 'ack ok')
check(d.decode(frame('GOLD_ACK_NO_ID')) == {'id': None, 'ok': False, 'err': 'no_id'}, 'ack no id')

check(codec.encode_command(7, {'cmd': 'start'}).hex() == GOLD['GOLD_CMD_START'], 'start frame')
check(codec.encode_command(9, {'cmd': 'set', 'param': 'speed_mm_s', 'value': 250}).hex() == GOLD['GOLD_CMD_SET'],
      'set frame')
check(codec.encode_command(5, {'cmd': 'map', 'left': 'A', 'right': 'F', 'straight': 'Z', 'uturn': 'W'}).hex()
      == GOLD['GOLD_CMD_MAP'], 'map frame')
check(codec.encode_command(256 + 7, {'cmd': 'start'}).hex() == GOLD['GOLD_CMD_START'], 'id wraps at 8 bits')

# Not a v1 frame (e.g. an old JSON payload still retained on the broker): ignored.
check(d.decode(b'{"online":false}') is None, 'JSON ignored')
check(d.decode(b'') is None and d.decode(bytes([0x41])) is None, 'empty / version 2 ignored')
check(d.decode(bytes.fromhex(GOLD['GOLD_TELEMETRY'])[:10]) is None, 'truncated telemetry ignored')

# The 16-bit encoder counts and distance are unwrapped into running totals.
u = codec.Unwrap(16)
seq = [65530, 65535, 4, 100, 65500]
got = [u.update(x) for x in seq]
check(got == [-6, -1, 4, 100, -36], f'unwrap {got}')

print(f'host tests: {passed} passed, {failed} failed')
sys.exit(1 if failed else 0)
