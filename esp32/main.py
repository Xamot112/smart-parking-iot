"""
IoT Smart Parking & Vehicle Monitoring System  (ESP32 + MicroPython, Wokwi)

Features
  * 5 slot sensors + 1 entry-gate sensor (HC-SR04)
  * Median-of-3 filtering + time-based debounce (no flickering slots)
  * Non-blocking gate state machine (barrier only closes after the car has passed)
  * Bay reservation: a bay assigned at the gate is held for RESERVE_TIMEOUT_MS
    so two cars are never sent to the same bay
  * Vehicle monitoring: entry / exit time, duration and fee per vehicle
  * WiFi + MQTT publishing (works offline, buffers events, auto-reconnects)

MQTT topics (all under BASE_TOPIC)
  status   (retained)  "online" / "offline"  (offline is sent as Last Will)
  slots    (retained)  live snapshot: free count, per-slot state, gate, totals
  events               entry / exit / gate_open / denied_full / reservation_expired
                       booked / booking_cancelled / booking_refused
  reserve  (incoming)  phone commands {"action":"reserve|arrive|cancel","name":"..."}
  reply                answer to the phone: ok, slot, reason
"""
import time
import json
import machine
import network
from machine import Pin, PWM, I2C
from esp32_i2c_lcd import I2cLcd

# ==========================================
# 1. CONFIGURATION
# ==========================================

TOTAL_SLOTS = 5

# Distance rules (cm).
# NOTE: 300 cm suits the Wokwi demo (idle sensors read 400 cm). On real hardware
# mount sensors above each bay and use roughly 30-150 cm depending on mounting.
OCCUPIED_THRESHOLD_CM = 300.0   # slot occupied when filtered reading <= this
GATE_THRESHOLD_CM = 10.0        # car at entry gate when filtered reading <= this

# Timing (ms)
SLOT_DEBOUNCE_MS = 2000         # a slot must read the new state this long to change
GATE_DEBOUNCE_MS = 500          # a car must be at the gate this long to be counted
GATE_CLEAR_MS = 1500            # gate sensor must be clear this long before closing
MIN_OPEN_MS = 2000              # barrier stays open at least this long
RESERVE_TIMEOUT_MS = 60000      # bay assigned at the gate is held this long
PHONE_HOLD_MS = 300000          # bay booked from a phone is held this long (5 min)
HEARTBEAT_MS = 10000            # publish a status snapshot at least this often
RECONNECT_MS = 15000            # retry WiFi/MQTT this often while offline
LOOP_DELAY_MS = 50

# Billing (demo rate: per started minute so it is visible in a short simulation)
RATE_PER_MIN = 1

# Servo angles
SERVO_CLOSED = 0
SERVO_OPEN = 90

# WiFi / MQTT  (Wokwi provides the open "Wokwi-GUEST" network with internet access)
WIFI_SSID = "Wokwi-GUEST"
WIFI_PASSWORD = ""
MQTT_BROKER = "broker.hivemq.com"   # public broker: anyone can read your topics!
MQTT_PORT = 1883
BASE_TOPIC = "iot_parking_skillLab/lot1"   # <-- make this unique to you

# ==========================================
# 2. HARDWARE SETUP  (pins unchanged from your diagram.json)
# ==========================================

# I2C LCD (SCL: GPIO 22, SDA: GPIO 21)
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
devices = i2c.scan()
lcd_addr = devices[0] if devices else 0x27
lcd = I2cLcd(i2c, lcd_addr, 2, 16)

# Status LEDs
led_yellow = Pin(2, Pin.OUT)    # idle, space available
led_green = Pin(15, Pin.OUT)    # gate open, car being admitted
led_red = Pin(13, Pin.OUT)      # parking full

# Servo barrier on GPIO 18
servo_pwm = PWM(Pin(18), freq=50)


def set_servo_angle(angle):
    """Sets servo position (0 to 180 degrees)."""
    duty = int(1800 + (angle / 180.0) * 6200)
    servo_pwm.duty_u16(duty)


# Entry gate sensor (Trig: GPIO 5, Echo: GPIO 19)
gate_trig = Pin(5, Pin.OUT)
gate_echo = Pin(19, Pin.IN)
gate_trig.value(0)

# Slot sensors (Trig, Echo)
slot_pins = [(12, 14), (27, 26), (25, 33), (32, 16), (4, 17)]
ultrasonic_sensors = []
for trig_pin, echo_pin in slot_pins:
    trig = Pin(trig_pin, Pin.OUT)
    echo = Pin(echo_pin, Pin.IN)
    trig.value(0)
    ultrasonic_sensors.append((trig, echo))

# ==========================================
# 3. STATE
# ==========================================

# Slots
slot_occupied = [False] * TOTAL_SLOTS      # confirmed (debounced) state
slot_candidate = [False] * TOTAL_SLOTS     # state currently being observed
slot_cand_since = [0] * TOTAL_SLOTS
slot_hist = [[999.0] * 3 for _ in range(TOTAL_SLOTS)]
slot_reserved = [False] * TOTAL_SLOTS
slot_reserved_at = [0] * TOTAL_SLOTS
slot_entry_ts = [0] * TOTAL_SLOTS
slot_hold_ms = [RESERVE_TIMEOUT_MS] * TOTAL_SLOTS   # how long each reservation lasts
slot_res_by = [None] * TOTAL_SLOTS                  # name of the phone user holding a bay
arrivals = []                                       # bays whose owner tapped "I've arrived"

# Gate
G_IDLE, G_OPEN, G_DENIED = 0, 1, 2
GATE_NAMES = ("idle", "open", "denied")
gate_state = G_IDLE
gate_hist = [999.0] * 3
gate_present = False
gate_cand = False
gate_cand_since = 0
gate_last_present = 0
gate_opened_at = 0
assigned_bay = None

# Totals
total_entries = 0
total_denied = 0
total_revenue = 0

# Connectivity
wlan = network.WLAN(network.STA_IF)
mqtt = None
mqtt_ok = False
ntp_ok = False
pending_events = []
last_reconnect_ms = 0
last_status_ms = 0
status_sig = None
lcd_cache = ["", ""]

# ==========================================
# 4. HELPERS
# ==========================================


def unix_now():
    """MicroPython's epoch is 2000-01-01; convert to Unix time for dashboards."""
    return int(time.time()) + 946684800


def read_distance(trig, echo):
    """Measures distance in cm using pulse timing. Returns 999.0 on timeout."""
    trig.value(0)
    time.sleep_us(2)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    timeout = 30000
    start = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), start) > timeout:
            return 999.0

    high_start = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), high_start) > timeout:
            return 999.0

    duration = time.ticks_diff(time.ticks_us(), high_start)
    return (duration * 0.0343) / 2


def median3(values):
    return sorted(values)[1]


def set_led_status(yellow, green, red):
    led_yellow.value(1 if yellow else 0)
    led_green.value(1 if green else 0)
    led_red.value(1 if red else 0)


def lcd_show(row, text):
    """Writes a 16-char line, only touching the LCD when the text changed."""
    text = (text + " " * 16)[:16]
    if lcd_cache[row] != text:
        lcd.move_to(0, row)
        lcd.putstr(text)
        lcd_cache[row] = text


def slot_state(i):
    if slot_occupied[i]:
        return "occupied"
    if slot_reserved[i]:
        return "reserved"
    return "free"


def available_slots():
    """Bays that are neither occupied nor already assigned to an incoming car."""
    return [i for i in range(TOTAL_SLOTS) if not slot_occupied[i] and not slot_reserved[i]]


# ==========================================
# 5. WIFI / MQTT  (all failures are non-fatal: the gate keeps working offline)
# ==========================================


def wifi_connect(timeout_ms=10000):
    wlan.active(True)
    if wlan.isconnected():
        return True
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    start = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), start) > timeout_ms:
            return False
        time.sleep_ms(200)
    return True


def sync_time():
    global ntp_ok
    try:
        import ntptime
        ntptime.settime()
        ntp_ok = True
    except Exception as e:
        print("NTP failed:", e)
        ntp_ok = False


def mqtt_connect():
    global mqtt, mqtt_ok
    try:
        from umqtt.simple import MQTTClient
        uid = "".join("{:02x}".format(b) for b in machine.unique_id())
        client = MQTTClient("esp32-parking-" + uid, MQTT_BROKER, port=MQTT_PORT, keepalive=30)
        client.set_last_will(BASE_TOPIC + "/status", "offline", retain=True)
        client.set_callback(on_message)
        client.connect()
        client.subscribe(BASE_TOPIC + "/reserve")
        client.publish(BASE_TOPIC + "/status", "online", retain=True)
        mqtt = client
        mqtt_ok = True
        print("MQTT connected")
    except Exception as e:
        print("MQTT connect failed:", e)
        mqtt_ok = False
    return mqtt_ok


def mqtt_publish(subtopic, payload, retain=False):
    global mqtt_ok
    if not mqtt_ok:
        return False
    try:
        mqtt.publish(BASE_TOPIC + "/" + subtopic, json.dumps(payload), retain)
        return True
    except Exception as e:
        print("MQTT publish failed:", e)
        mqtt_ok = False
        return False


def publish_event(event):
    """Sends an event now, or buffers it (max 20) if the cloud is unreachable."""
    event["ts"] = unix_now()
    print("EVENT", event)
    if not mqtt_publish("events", event):
        if len(pending_events) >= 20:
            pending_events.pop(0)
        pending_events.append(event)


def flush_pending():
    while pending_events and mqtt_ok:
        if mqtt_publish("events", pending_events[0]):
            pending_events.pop(0)
        else:
            break


def publish_status_if_needed(now):
    """Retained snapshot: sent on any change and at least every HEARTBEAT_MS."""
    global status_sig, last_status_ms
    states = [slot_state(i) for i in range(TOTAL_SLOTS)]
    sig = (tuple(states), gate_state)
    if sig == status_sig and time.ticks_diff(now, last_status_ms) < HEARTBEAT_MS:
        return
    payload = {
        "ts": unix_now(),
        "time_synced": ntp_ok,
        "total": TOTAL_SLOTS,
        "free": len(available_slots()),
        "slots": states,
        "gate": GATE_NAMES[gate_state],
        "entries": total_entries,
        "denied": total_denied,
        "revenue": total_revenue,
    }
    if mqtt_publish("slots", payload, retain=True):
        status_sig = sig
        last_status_ms = now


# ---- phone reservations (commands arrive on BASE_TOPIC/reserve) ----


def find_booking(name):
    for i in range(TOTAL_SLOTS):
        if slot_res_by[i] == name:
            return i
    return None


def release_booking(i, why):
    name = slot_res_by[i]
    slot_reserved[i] = False
    slot_res_by[i] = None
    if i in arrivals:
        arrivals.remove(i)
    publish_event({"event": why, "slot": i + 1, "by": name})


def reply(name, ok, **info):
    info["name"] = name
    info["ok"] = ok
    mqtt_publish("reply", info)


def handle_command(cmd):
    now = time.ticks_ms()
    name = str(cmd.get("name", "guest"))[:12]
    action = cmd.get("action")
    i = find_booking(name)
    if action == "reserve":
        if i is not None:
            reply(name, True, slot=i + 1, note="already booked")
            return
        free = available_slots()
        if not free:
            reply(name, False, reason="lot full")
            publish_event({"event": "booking_refused", "by": name})
            return
        i = free[0]
        slot_reserved[i] = True
        slot_reserved_at[i] = now
        slot_hold_ms[i] = PHONE_HOLD_MS
        slot_res_by[i] = name
        reply(name, True, slot=i + 1, hold_s=PHONE_HOLD_MS // 1000)
        publish_event({"event": "booked", "slot": i + 1, "by": name})
    elif i is None:
        reply(name, False, reason="no booking found")
    elif action == "arrive":
        if i not in arrivals:
            arrivals.append(i)
        reply(name, True, slot=i + 1, note="drive to the gate")
    elif action == "cancel":
        release_booking(i, "booking_cancelled")
        reply(name, True, note="cancelled")


def on_message(topic, msg):
    try:
        handle_command(json.loads(msg))
    except Exception as e:
        print("Bad command:", e)


def poll_mqtt():
    """Checks for incoming phone commands without waiting."""
    global mqtt_ok
    if not mqtt_ok:
        return
    try:
        mqtt.check_msg()
    except Exception as e:
        print("MQTT poll failed:", e)
        mqtt_ok = False


def maintain_connection(now):
    global last_reconnect_ms
    if mqtt_ok or time.ticks_diff(now, last_reconnect_ms) < RECONNECT_MS:
        return
    last_reconnect_ms = now
    if not wlan.isconnected():
        wifi_connect(5000)
    if wlan.isconnected():
        if not ntp_ok:
            sync_time()
        if mqtt_connect():
            flush_pending()


# ==========================================
# 6. PARKING LOGIC
# ==========================================


def on_slot_change(i, occupied):
    """Called once when a slot's debounced state flips. Logs entry / exit + fee."""
    global total_entries, total_revenue
    slot_occupied[i] = occupied
    if occupied:
        slot_reserved[i] = False            # the assigned car has arrived
        slot_res_by[i] = None
        if i in arrivals:
            arrivals.remove(i)
        slot_entry_ts[i] = unix_now()
        total_entries += 1
        publish_event({"event": "entry", "slot": i + 1})
    else:
        exit_ts = unix_now()
        duration = int(max(0, exit_ts - slot_entry_ts[i]))
        minutes = max(1, (duration + 59) // 60)
        fee = minutes * RATE_PER_MIN
        total_revenue += fee
        publish_event({
            "event": "exit", "slot": i + 1,
            "entry_ts": slot_entry_ts[i], "duration_s": duration, "fee": fee,
        })


def update_slot(i, now):
    trig, echo = ultrasonic_sensors[i]
    hist = slot_hist[i]
    hist.pop(0)
    hist.append(read_distance(trig, echo))
    raw = median3(hist) <= OCCUPIED_THRESHOLD_CM

    if raw != slot_candidate[i]:
        slot_candidate[i] = raw
        slot_cand_since[i] = now
    elif raw != slot_occupied[i] and time.ticks_diff(now, slot_cand_since[i]) >= SLOT_DEBOUNCE_MS:
        on_slot_change(i, raw)


def update_gate_sensor(now):
    global gate_present, gate_cand, gate_cand_since, gate_last_present
    gate_hist.pop(0)
    gate_hist.append(read_distance(gate_trig, gate_echo))
    raw = median3(gate_hist) <= GATE_THRESHOLD_CM

    if raw != gate_cand:
        gate_cand = raw
        gate_cand_since = now
    elif raw != gate_present and time.ticks_diff(now, gate_cand_since) >= GATE_DEBOUNCE_MS:
        gate_present = raw
    if gate_present:
        gate_last_present = now


def expire_reservations(now):
    for i in range(TOTAL_SLOTS):
        if slot_reserved[i] and time.ticks_diff(now, slot_reserved_at[i]) > slot_hold_ms[i]:
            release_booking(i, "reservation_expired")


def run_gate(now):
    """Non-blocking entry-gate state machine: IDLE -> OPEN -> IDLE  /  IDLE -> DENIED -> IDLE."""
    global gate_state, gate_opened_at, assigned_bay, total_denied

    if gate_state == G_IDLE:
        if gate_present:
            free = available_slots()
            if arrivals or free:
                # a car whose owner tapped "I've arrived" gets their booked bay first
                bay = arrivals.pop(0) if arrivals else free[0]
                slot_reserved[bay] = True
                slot_reserved_at[bay] = now
                slot_hold_ms[bay] = RESERVE_TIMEOUT_MS
                assigned_bay = bay
                gate_opened_at = now
                gate_state = G_OPEN
                set_servo_angle(SERVO_OPEN)
                publish_event({"event": "gate_open", "slot": bay + 1})
            else:
                gate_state = G_DENIED
                total_denied += 1
                publish_event({"event": "denied_full"})

    elif gate_state == G_OPEN:
        # Safety: never close while the car is still in front of the sensor
        if (time.ticks_diff(now, gate_opened_at) >= MIN_OPEN_MS
                and time.ticks_diff(now, gate_last_present) >= GATE_CLEAR_MS):
            set_servo_angle(SERVO_CLOSED)
            assigned_bay = None
            gate_state = G_IDLE

    elif gate_state == G_DENIED:
        # Wait for the refused car to leave before accepting a new one
        if not gate_present and time.ticks_diff(now, gate_last_present) >= GATE_CLEAR_MS:
            gate_state = G_IDLE


def update_outputs():
    free = len(available_slots())

    if gate_state == G_OPEN:
        set_led_status(False, True, False)
    elif free == 0:
        set_led_status(False, False, True)
    else:
        set_led_status(True, False, False)

    lcd_show(0, "Free Slots: {}/{}".format(free, TOTAL_SLOTS))
    if gate_state == G_OPEN:
        lcd_show(1, "Go to Bay #{}".format(assigned_bay + 1))
    elif gate_state == G_DENIED:
        lcd_show(1, "DENIED: Full!")
    else:
        lcd_show(1, "{:<8}Cloud:{}".format("FULL!" if free == 0 else "Ready",
                                           "ON" if mqtt_ok else "--"))


def step(now):
    """One pass of the main loop."""
    for i in range(TOTAL_SLOTS):
        update_slot(i, now)
        time.sleep_ms(5)            # small gap between sensors to avoid echo cross-talk
    update_gate_sensor(now)
    expire_reservations(now)
    run_gate(now)
    update_outputs()
    poll_mqtt()
    publish_status_if_needed(now)
    maintain_connection(now)


# ==========================================
# 7. MAIN
# ==========================================


def main():
    global last_reconnect_ms
    set_servo_angle(SERVO_CLOSED)
    set_led_status(False, False, False)

    lcd.clear()
    lcd_show(0, "Smart Parking")
    lcd_show(1, "Connecting WiFi")
    if wifi_connect():
        sync_time()
        mqtt_connect()
    last_reconnect_ms = time.ticks_ms()

    lcd.clear()
    lcd_cache[0] = lcd_cache[1] = ""

    while True:
        step(time.ticks_ms())
        time.sleep_ms(LOOP_DELAY_MS)


if __name__ == "__main__":
    main()