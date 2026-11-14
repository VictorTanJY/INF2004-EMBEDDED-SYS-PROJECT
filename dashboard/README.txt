PicoCar IS23 - local dashboard
==============================
1. Extract this entire ZIP (do not run files inside the ZIP).
2. Install Python 3 if needed: https://www.python.org/downloads/windows/
   Select "Add Python to PATH". No pip packages are needed.
3. Mosquitto must be installed (the same broker used in your tests).
4. Connect the laptop to the robot's hotspot.
5. Double-click Start-Dashboard.cmd. Leave that ONE window open.
6. Your browser opens http://127.0.0.1:8765.

The launcher starts Mosquitto on port 1883 if no broker is running there,
and reuses an existing broker without changing its configuration. Allow
Mosquitto through Windows Firewall on the hotspot network if needed.
The firmware must contain the hotspot name/password and this laptop's
Wi-Fi IPv4 address as MQTT_BROKER_IP.

What it does
------------
Controls (all over MQTT, each command waits for the robot's ack):
- Stop (Space): always available. Lifting or overturning the robot during
  a run also stops it.
- Reset fault (with a confirm step): needs fresh telemetry.
- Start run (with a confirm step that shows the map): IDLE only. The robot
  refuses a start until it is calibrated (tap GP21 on the robot).
- Barcode map: type the four characters the instructor gives for left,
  right, straight and U-turn, then Send map. IDLE only; the robot forgets
  the map on reboot, so send it after every power-up.
- Manual moves (forward/back 200 mm, left/right 90 deg, U-turn 180 deg,
  scan servo left/centre/right): only firmware built with APP_BENCH 1
  accepts them, in IDLE; otherwise the ack says bad_param.

Readings:
- Robot state, connection, Wi-Fi, uptime, drops; what the wheels are doing.
- Map on the robot, last barcode, the character waiting for the next junction.
- 60 s graphs of wheel speed, ultrasonic range (20/40 cm lines), and pitch
  with roll (roll = side lean, + = right side up; a hump under one wheel
  shows mostly as roll).
- Obstacle width, left/right clearance. Type the real box width to see the
  error and its grade band (A <= 10 %, B <= 20 %, C <= 30 %).
- Last bypass time, OBSTACLE event -> back on the line, with its band
  (A < 3 s, B < 5 s, C < 7 s), and a drawing of the driven bypass path
  rebuilt from the encoder counts. Copy your measured track width and
  wheel/encoder values from subsystem2/motion_cfg.h into the top of
  app.js so the drawing is to scale. The box's sideways position is not
  sent by the robot, so it is drawn centred on the line.
- Barcode scans (read, latched -> movement, not in map, rejected + reason).
- Humps (peak height, max pitch and max roll seen, duration) and the run's
  highest hump.
- Event history (filters: faults, state, mission, commands, diagnostics)
  and the command log. Fault codes: 1 lifted, 2 too close, 4 stall,
  8 overturned, 32 heartbeat, 64 encoder, 128 IMU, 256 no echo,
  512 Wi-Fi loss, 1024 start-up self-test; codes can combine.

"Try demo" plays a simulated run (barcode, junction turn, obstacle
bypass, hump) without sending anything to a robot.

Safety notes
------------
Do not rely on MQTT Stop if the link is down: switch off motor power.
Closing the launcher does NOT stop the robot. A command's "accepted" means
the firmware took it, not that the wheels have stopped: watch the robot.
Do manual moves on a stand with the wheels clear.

The robot's MQTT payloads are binary bit-field frames (minimum-bit data,
Change Requirement 1). dashboard.py reads them in hex from mosquitto_sub
and decodes them with picocar_codec.py; commands are encoded the same way
and sent with mosquitto_pub -f. To watch the raw topics without the page:
  mosquitto_sub -h <broker> -t "picocar/is23/#" -F "%t %x" | python picocar_codec.py
