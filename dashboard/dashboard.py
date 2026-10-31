"""PicoCar IS23 local dashboard. Python 3.9+ and Mosquitto; no pip packages.

Reads every picocar/is23/* message through mosquitto_sub, keeps the latest
telemetry and status, a 120 s telemetry history (5 Hz), the event log and
the last obstacle bypass, and serves them to the page at 127.0.0.1:8765.
Commands are published with mosquitto_pub on picocar/is23/cmd in the
firmware's JSON format and matched to their ack by id.
"""
import atexit, collections, http.server, json, os, pathlib, secrets, shutil, socket, subprocess, threading, time, webbrowser

ROOT = pathlib.Path(__file__).resolve().parent
LOCK = threading.RLock()
TOKEN = secrets.token_urlsafe(32)
HISTORY_S = 120                     # telemetry kept for the graphs
FRESH_S = 3                         # telemetry older than this is stale
BYPASS_STATES = ('OBSTACLE_HANDLING', 'AVOIDING', 'LINE_RECOVERY')

# Code 39 characters the firmware accepts in a map (comms_map_char_ok).
MAP_CHARS = set('0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%')
MAP_KEYS = ('left', 'right', 'straight', 'uturn')
# Bench moves: only accepted by firmware built with APP_BENCH=1, in IDLE.
BENCH = {'bench_move': (-2000, 2000), 'bench_turn': (-3600, 3600), 'bench_servo': (0, 180)}

DATA = {'broker': False, 'host': '127.0.0.1', 'telemetry': None, 'status': None, 'received': 0,
        'events': [], 'commands': [], 'history': [], 'bypass': None, 'error': '', 'revision': 0, '_prev_sample': None}
SUB = None
BROKER = None
GEN = 0
LAST_ID = int(time.time()) % 2000000000


def executable(name):
    found = shutil.which(name)
    if found:
        return found
    for base in (os.environ.get('PROGRAMFILES(X86)', r'C:\Program Files (x86)'),
                 os.environ.get('PROGRAMFILES', r'C:\Program Files')):
        p = pathlib.Path(base) / 'mosquitto' / (name + '.exe')
        if p.exists():
            return str(p)
    raise RuntimeError('Mosquitto is not installed or could not be found. Install Mosquitto, then reopen the launcher.')


def add_event(topic, payload):
    DATA['revision'] += 1
    DATA['events'].append({'key': DATA['revision'], 'time': time.time(), 'topic': topic, 'payload': payload})
    DATA['events'] = DATA['events'][-600:]


def num(x):
    return x if isinstance(x, (int, float)) and not isinstance(x, bool) else None


def track_bypass(t, now):
    """Record the telemetry of each obstacle episode (OBSTACLE_HANDLING ->
    AVOIDING -> LINE_RECOVERY) so the page can redraw the driven path."""
    state = t.get('state')
    ob = t.get('obstacle') if isinstance(t.get('obstacle'), dict) else {}
    v = t.get('v_mm_s') if isinstance(t.get('v_mm_s'), list) and len(t['v_mm_s']) == 2 else [None, None]
    enc = t.get('enc') if isinstance(t.get('enc'), list) and len(t['enc']) == 2 else [None, None]
    ep = DATA['bypass']
    prev = DATA.get('_prev_sample')
    DATA['_prev_sample'] = {'tms': num(t.get('t_ms')), 'vl': num(v[0]), 'vr': num(v[1]), 'el': num(enc[0]), 'er': num(enc[1]), 'st': state}
    if state in BYPASS_STATES:
        if ep is None or ep.get('done'):
            ep = DATA['bypass'] = {'start': now, 'start_tms': num(t.get('t_ms')), 'end': None, 'done': False,
                                   'end_state': None, 'd_cm': num(ob.get('d_cm')), 'w_mm': None, 'samples': [prev] if prev else []}
        if num(ob.get('w_mm')) is not None:
            ep['w_mm'] = ob['w_mm']
        if ep['d_cm'] is None and num(ob.get('d_cm')) is not None:
            ep['d_cm'] = ob['d_cm']
        if len(ep['samples']) < 1500:
            ep['samples'].append({'tms': num(t.get('t_ms')), 'vl': num(v[0]), 'vr': num(v[1]),
                                  'el': num(enc[0]), 'er': num(enc[1]), 'st': state})
    elif ep is not None and not ep.get('done'):
        ep.update(done=True, end=now, end_state=state)


def ingest(line):
    try:
        topic, raw = line.strip().split(' ', 1)
        if not topic.startswith('picocar/is23/'):
            return
        p = json.loads(raw)
        if not isinstance(p, dict):
            return
    except (ValueError, TypeError):
        return
    with LOCK:
        now = time.time()
        if topic.endswith('/telemetry'):
            DATA['telemetry'] = p
            DATA['received'] = now
            v = p.get('v_mm_s') if isinstance(p.get('v_mm_s'), list) and len(p['v_mm_s']) == 2 else [None, None]
            ob = p.get('obstacle') if isinstance(p.get('obstacle'), dict) else {}
            DATA['history'].append({'t': now, 'vl': num(v[0]), 'vr': num(v[1]), 'd': num(ob.get('d_cm')),
                                    'p': num(p.get('pitch_deg')), 'st': p.get('state')})
            while DATA['history'] and now - DATA['history'][0]['t'] > HISTORY_S:
                DATA['history'].pop(0)
            track_bypass(p, now)
        elif topic.endswith('/status'):
            DATA['status'] = p
        elif topic.endswith('/ack'):
            for c in DATA['commands']:
                if c['id'] == p.get('id'):
                    c['result'] = 'accepted' if p.get('ok') is True else 'rejected'
                    c['ack'] = p
            add_event(topic, p)
        elif topic.endswith('/event'):
            add_event(topic, p)


def stop_sub():
    global SUB, GEN
    GEN += 1
    if SUB and SUB.poll() is None:
        SUB.terminate()
    SUB = None
    DATA['broker'] = False


def connect(host):
    global SUB, GEN
    if not isinstance(host, str) or not host or len(host) > 253 or any(
            c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.-:' for c in host):
        raise ValueError('Enter an IP address or hostname without a port or URL.')
    with LOCK:
        stop_sub()
        DATA.update(host=host, error='', received=0, telemetry=None, status=None, history=[])
        args = [executable('mosquitto_sub'), '-h', host, '-p', '1883', '-i', 'dashboard-' + secrets.token_hex(5),
                '-t', 'picocar/is23/#', '-v', '-R']
        SUB = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                               encoding='utf-8', errors='replace', bufsize=1)
        proc, generation = SUB, GEN

    def reader():
        for line in proc.stdout:
            with LOCK:
                if generation != GEN:
                    return
                DATA['broker'] = True
                DATA['error'] = ''
            ingest(line)
        with LOCK:
            if generation == GEN:
                DATA['broker'] = False

    def errors():
        for line in proc.stderr:
            with LOCK:
                if generation != GEN:
                    return
                DATA['error'] = line.strip()
                DATA['broker'] = False

    threading.Thread(target=reader, daemon=True).start()
    threading.Thread(target=errors, daemon=True).start()


def build_command(cmd, args):
    """Validate a request from the page and return the firmware command body
    (without id) plus a short label for the command log."""
    t = DATA['telemetry'] or {}
    fresh = time.time() - DATA['received'] <= FRESH_S
    state = t.get('state')
    if cmd in ('estop', 'stop'):
        return {'cmd': cmd}, cmd.upper()
    if not fresh:
        raise ValueError('This command needs fresh telemetry. Check the robot connection first.')
    if cmd == 'reset':
        return {'cmd': 'reset'}, 'RESET'
    if state != 'IDLE':
        raise ValueError(f'The robot only accepts this in IDLE (now {state or "unknown"}).')
    if cmd == 'start':
        return {'cmd': 'start'}, 'START'
    if cmd == 'map':
        chars = []
        for k in MAP_KEYS:
            c = str(args.get(k, '')).strip().upper()
            if len(c) != 1 or c not in MAP_CHARS:
                raise ValueError(f'"{k}" needs one Code 39 character (A-Z, 0-9 or - . space $ / + %).')
            chars.append(c)
        if len(set(chars)) != 4:
            raise ValueError('Each movement needs a different character.')
        return dict({'cmd': 'map'}, **dict(zip(MAP_KEYS, chars))), 'MAP ' + ''.join(chars)
    if cmd == 'bench':
        param = args.get('param')
        value = args.get('value')
        if param not in BENCH or not isinstance(value, int) or isinstance(value, bool):
            raise ValueError('Unsupported bench move.')
        lo, hi = BENCH[param]
        if not lo <= value <= hi:
            raise ValueError(f'{param} must be {lo}..{hi}.')
        return {'cmd': 'set', 'param': param, 'value': value}, f'{param.upper()} {value}'
    raise ValueError('Unsupported command.')


def command(cmd, args):
    global LAST_ID
    with LOCK:
        if not DATA['broker']:
            raise ValueError('No messages received from the broker yet.')
        body, label = build_command(cmd, args if isinstance(args, dict) else {})
        LAST_ID = (LAST_ID + 1) % 2147483647
        record = {'id': LAST_ID, 'cmd': cmd, 'label': label, 'time': time.time(), 'result': 'pending'}
        DATA['commands'].append(record)
        DATA['commands'] = DATA['commands'][-30:]
        host = DATA['host']
        payload = json.dumps(dict({'id': LAST_ID}, **body), separators=(',', ':'))
    try:
        r = subprocess.run([executable('mosquitto_pub'), '-h', host, '-p', '1883', '-t', 'picocar/is23/cmd',
                            '-q', '1', '-l'], input=payload + '\n', text=True, capture_output=True, timeout=4)
        if r.returncode:
            raise RuntimeError(r.stderr.strip() or 'Publish failed')
    except Exception as e:
        with LOCK:
            record['result'] = 'publish failed'
            record['error'] = str(e)
    return record


def snapshot():
    with LOCK:
        for c in DATA['commands']:
            if c['result'] == 'pending' and time.time() - c['time'] > 4:
                c['result'] = 'no acknowledgement'
        return json.loads(json.dumps(dict(DATA, token=TOKEN, now=time.time(), mode='local')))


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, obj):
        body = json.dumps(obj, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(body)

    def valid_host(self):
        return self.headers.get('Host') in ('127.0.0.1:8765', 'localhost:8765')

    def do_GET(self):
        if not self.valid_host():
            return self.reply(403, {'error': 'Invalid host'})
        if self.path == '/api/status':
            return self.reply(200, snapshot())
        if self.path not in ('/', '/index.html', '/app.js', '/style.css'):
            return self.reply(404, {'error': 'Not found'})
        name = 'index.html' if self.path == '/' else self.path[1:]
        data = (ROOT / name).read_bytes()
        types = {'html': 'text/html; charset=utf-8', 'js': 'text/javascript; charset=utf-8', 'css': 'text/css; charset=utf-8'}
        self.send_response(200)
        self.send_header('Content-Type', types[name.split('.')[-1]])
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        self.wfile.write(data)

    def do_POST(self):
        if (not self.valid_host() or self.headers.get('Origin') not in ('http://127.0.0.1:8765', 'http://localhost:8765')
                or self.headers.get('X-Dashboard-Token') != TOKEN):
            return self.reply(403, {'error': 'Open the dashboard on this laptop to send controls.'})
        try:
            n = int(self.headers.get('Content-Length', '0'))
            if n > 4096:
                raise ValueError('Request too large')
            body = json.loads(self.rfile.read(n))
            if self.path == '/api/connect':
                connect(body.get('host'))
                result = {'ok': True}
            elif self.path == '/api/command':
                result = command(body.get('cmd'), body.get('args'))
            elif self.path == '/api/disconnect':
                with LOCK:
                    stop_sub()
                result = {'ok': True}
            else:
                return self.reply(404, {'error': 'Not found'})
            self.reply(200, result)
        except Exception as e:
            self.reply(400, {'error': str(e)})


def cleanup():
    with LOCK:
        stop_sub()
    if BROKER and BROKER.poll() is None:
        BROKER.terminate()


atexit.register(cleanup)


def main():
    global BROKER
    try:
        server = http.server.ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    except OSError:
        print('Port 8765 is already in use. If a dashboard is already running, open http://127.0.0.1:8765.')
        return
    try:
        try:
            with socket.create_connection(('127.0.0.1', 1883), timeout=1):
                pass
            print('Using the broker already running on this laptop.')
        except OSError:
            config = ROOT / 'dashboard-broker.conf'
            config.write_text('listener 1883 0.0.0.0\nallow_anonymous true\n', encoding='ascii')
            BROKER = subprocess.Popen([executable('mosquitto'), '-c', str(config)],
                                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(.6)
            if BROKER.poll() is not None:
                raise RuntimeError('Could not start Mosquitto. Check that port 1883 is available.')
            print('Started Mosquitto on port 1883. Allow it through Windows Firewall on your hotspot if asked.')
        connect('127.0.0.1')
    except Exception as e:
        DATA['error'] = str(e)
        print(str(e))
    print('\nPicoCar dashboard: http://127.0.0.1:8765\nLeave this window open. Ctrl+C closes the dashboard.\n'
          'The robot firmware still needs the correct hotspot and laptop broker IP.\n')
    if not os.environ.get('PICOCAR_NO_BROWSER'):
        webbrowser.open('http://127.0.0.1:8765')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        cleanup()


if __name__ == '__main__':
    main()
