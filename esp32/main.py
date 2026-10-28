# IoT smart parking - ESP32 + MicroPython (Wokwi)
# 5 bay sensors, entry gate, exit gate, LCD and 3 LEDs.
# Sends everything to MQTT and takes bookings from a phone.
#
# topics (all under BASE_TOPIC):
#   status  - online / offline (retained, offline is the last will)
#   slots   - live snapshot (retained)
#   events  - entry, exit, gate_open, gate_exit, denied_full, booked ...
#   reserve - commands coming in: {"action": "reserve|arrive|cancel", "name": "..."}
#   reply   - answer sent back to the phone

import time
import json
import machine
import network
from machine import Pin, PWM, I2C
from esp32_i2c_lcd import I2cLcd

TOTAL_SLOTS = 5

OCCUPIED_CM = 300        # ok for wokwi (idle sensor reads 400), use ~30-150 on a real build
GATE_CM = 10             # car is at a gate when closer than this

# times in ms
SLOT_DEBOUNCE = 2000     # bay must read the new state this long before we believe it
GATE_DEBOUNCE = 500
GATE_CLEAR = 1500        # gate sensor must be clear this long before the barrier closes
MIN_OPEN = 2000
GATE_HOLD = 60000        # bay given at the gate is kept for 1 min
PHONE_HOLD = 300000      # bay booked on the phone is kept for 5 min
HEARTBEAT = 10000
RECONNECT = 15000

RATE_PER_MIN = 1         # low on purpose so you see a fee in a short simulation
MAX_BILLS = 10           # cars that left a bay but haven't reached the exit yet

OPEN_ANGLE = 90
CLOSED_ANGLE = 0

WIFI_SSID = "Wokwi-GUEST"
WIFI_PASSWORD = ""
MQTT_BROKER = "broker.hivemq.com"      # public broker, anyone can read the topics
MQTT_PORT = 1883
BASE_TOPIC = "iot_parking_skillLab/lot1"

IDLE, OPEN, DENIED = 0, 1, 2
STATE_NAMES = ("idle", "open", "denied")


# ---- hardware ----

i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
found = i2c.scan()
lcd = I2cLcd(i2c, found[0] if found else 0x27, 2, 16)

led_yellow = Pin(2, Pin.OUT)    # idle, space free
led_green = Pin(15, Pin.OUT)    # a gate is open
led_red = Pin(13, Pin.OUT)      # full

entry_servo = PWM(Pin(18), freq=50)
exit_servo = PWM(Pin(23), freq=50)


def move_servo(servo, angle):
    servo.duty_u16(int(1800 + angle / 180 * 6200))


def lights(yellow, green, red):
    led_yellow.value(1 if yellow else 0)
    led_green.value(1 if green else 0)
    led_red.value(1 if red else 0)


def read_distance(trig, echo):
    # returns cm, or 999 if the echo never comes back
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

    return time.ticks_diff(time.ticks_us(), high_start) * 0.0343 / 2


class Detector:
    # one ultrasonic sensor with a median filter and a debounce,
    # so a single bad reading can't flip a bay or open a gate
    def __init__(self, trig_pin, echo_pin, limit_cm, debounce_ms):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.value(0)
        self.limit = limit_cm
        self.debounce = debounce_ms
        self.readings = [999.0, 999.0, 999.0]
        self.state = False       # what we believe
        self.maybe = False       # what the sensor says right now
        self.since = 0
        self.last_seen = 0

    def update(self, now):
        # returns True on the loop where self.state changes
        self.readings.pop(0)
        self.readings.append(read_distance(self.trig, self.echo))
        close = sorted(self.readings)[1] <= self.limit

        changed = False
        if close != self.maybe:
            self.maybe = close
            self.since = now
        elif close != self.state and time.ticks_diff(now, self.since) >= self.debounce:
            self.state = close
            changed = True
        if self.state:
            self.last_seen = now
        return changed


slot_pins = [(12, 14), (27, 26), (25, 33), (32, 16), (4, 17)]
slots = [Detector(t, e, OCCUPIED_CM, SLOT_DEBOUNCE) for t, e in slot_pins]
entry_sensor = Detector(5, 19, GATE_CM, GATE_DEBOUNCE)
exit_sensor = Detector(0, 34, GATE_CM, GATE_DEBOUNCE)


# ---- state ----

reserved = [False] * TOTAL_SLOTS
res_time = [0] * TOTAL_SLOTS
res_hold = [GATE_HOLD] * TOTAL_SLOTS
res_name = [None] * TOTAL_SLOTS      # who booked it from the phone
entry_time = [0] * TOTAL_SLOTS
arrivals = []                        # bays whose owner pressed "I've arrived"
bills = []                           # cars that left a bay, waiting to pay at the exit

entry_state = IDLE
entry_opened = 0
next_bay = None

exit_state = IDLE
exit_opened = 0
exit_bill = None

n_entries = 0
n_exits = 0
n_denied = 0
revenue = 0

wlan = network.WLAN(network.STA_IF)
mqtt = None
mqtt_ok = False
ntp_ok = False
pending = []                         # events we couldn't send yet
last_reconnect = 0
last_status = 0
last_sig = None
lcd_text = ["", ""]


def unix_now():
    # micropython counts from 2000, dashboards want unix time
    return int(time.time()) + 946684800


def show(row, text):
    # only rewrite the lcd when the text changed, avoids flicker
    text = (text + " " * 16)[:16]
    if lcd_text[row] != text:
        lcd.move_to(0, row)
        lcd.putstr(text)
        lcd_text[row] = text


def bay_status(i):
    if slots[i].state:
        return "occupied"
    if reserved[i]:
        return "reserved"
    return "free"


def free_bays():
    return [i for i in range(TOTAL_SLOTS) if not slots[i].state and not reserved[i]]


# ---- wifi / mqtt ----
# if any of this fails the gates keep working, we just retry later

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
        print("ntp failed:", e)
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
        print("mqtt connected")
    except Exception as e:
        print("mqtt connect failed:", e)
        mqtt_ok = False
    return mqtt_ok


def send(subtopic, data, retain=False):
    global mqtt_ok
    if not mqtt_ok:
        return False
    try:
        mqtt.publish(BASE_TOPIC + "/" + subtopic, json.dumps(data), retain)
        return True
    except Exception as e:
        print("publish failed:", e)
        mqtt_ok = False
        return False


def log_event(event):
    event["ts"] = unix_now()
    print("EVENT", event)
    if not send("events", event):
        if len(pending) >= 20:
            pending.pop(0)
        pending.append(event)


def flush_pending():
    while pending and mqtt_ok:
        if send("events", pending[0]):
            pending.pop(0)
        else:
            break


def send_status(now):
    # snapshot goes out when something changed, or every HEARTBEAT ms
    global last_sig, last_status
    states = [bay_status(i) for i in range(TOTAL_SLOTS)]
    sig = (tuple(states), entry_state, exit_state, len(bills))
    if sig == last_sig and time.ticks_diff(now, last_status) < HEARTBEAT:
        return
    data = {
        "ts": unix_now(),
        "time_synced": ntp_ok,
        "total": TOTAL_SLOTS,
        "free": len(free_bays()),
        "slots": states,
        "gate": STATE_NAMES[entry_state],
        "exit_gate": STATE_NAMES[exit_state],
        "entries": n_entries,
        "exits": n_exits,
        "bills": len(bills),
        "denied": n_denied,
        "revenue": revenue,
    }
    if send("slots", data, retain=True):
        last_sig = sig
        last_status = now


def reconnect(now):
    global last_reconnect
    if mqtt_ok or time.ticks_diff(now, last_reconnect) < RECONNECT:
        return
    last_reconnect = now
    if not wlan.isconnected():
        wifi_connect(5000)
    if wlan.isconnected():
        if not ntp_ok:
            sync_time()
        if mqtt_connect():
            flush_pending()


# ---- phone bookings ----

def find_booking(name):
    for i in range(TOTAL_SLOTS):
        if res_name[i] == name:
            return i
    return None


def drop_booking(i, why):
    name = res_name[i]
    reserved[i] = False
    res_name[i] = None
    if i in arrivals:
        arrivals.remove(i)
    log_event({"event": why, "slot": i + 1, "by": name})


def reply(name, ok, **info):
    info["name"] = name
    info["ok"] = ok
    send("reply", info)


def handle_command(cmd):
    name = str(cmd.get("name", "guest"))[:12]
    action = cmd.get("action")
    i = find_booking(name)

    if action == "reserve":
        if i is not None:
            reply(name, True, slot=i + 1, note="already booked")
            return
        free = free_bays()
        if not free:
            reply(name, False, reason="lot full")
            log_event({"event": "booking_refused", "by": name})
            return
        i = free[0]
        reserved[i] = True
        res_time[i] = time.ticks_ms()
        res_hold[i] = PHONE_HOLD
        res_name[i] = name
        reply(name, True, slot=i + 1, hold_s=PHONE_HOLD // 1000)
        log_event({"event": "booked", "slot": i + 1, "by": name})
    elif i is None:
        reply(name, False, reason="no booking found")
    elif action == "arrive":
        if i not in arrivals:
            arrivals.append(i)
        reply(name, True, slot=i + 1, note="drive to the gate")
    elif action == "cancel":
        drop_booking(i, "booking_cancelled")
        reply(name, True, note="cancelled")
    else:
        reply(name, False, reason="unknown action")


def on_message(topic, msg):
    print("phone command:", msg)
    try:
        handle_command(json.loads(msg))
    except Exception as e:
        print("bad command:", e)


def check_phone():
    global mqtt_ok
    if not mqtt_ok:
        return
    try:
        mqtt.check_msg()       # doesn't wait if nothing is there
    except Exception as e:
        print("mqtt poll failed:", e)
        mqtt_ok = False


# ---- parking logic ----

def slot_changed(i):
    global n_entries
    if slots[i].state:
        # a car just parked
        reserved[i] = False
        res_name[i] = None
        if i in arrivals:
            arrivals.remove(i)
        entry_time[i] = unix_now()
        n_entries += 1
        log_event({"event": "entry", "slot": i + 1})
    else:
        # car left the bay, work out the bill and wait for it at the exit
        secs = max(0, unix_now() - entry_time[i])
        fee = max(1, (secs + 59) // 60) * RATE_PER_MIN
        bills.append({"slot": i + 1, "duration_s": secs, "fee": fee})
        if len(bills) > MAX_BILLS:
            bills.pop(0)
        log_event({"event": "exit", "slot": i + 1, "entry_ts": entry_time[i],
                   "duration_s": secs, "fee": fee})


def expire_reservations(now):
    for i in range(TOTAL_SLOTS):
        if reserved[i] and time.ticks_diff(now, res_time[i]) > res_hold[i]:
            drop_booking(i, "reservation_expired")


def run_entry(now):
    global entry_state, entry_opened, next_bay, n_denied
    car = entry_sensor.state

    if entry_state == IDLE:
        if not car:
            return
        free = free_bays()
        if arrivals or free:
            # someone who pressed "arrived" gets their own bay first
            bay = arrivals.pop(0) if arrivals else free[0]
            reserved[bay] = True
            res_time[bay] = now
            res_hold[bay] = GATE_HOLD
            next_bay = bay
            entry_opened = now
            entry_state = OPEN
            move_servo(entry_servo, OPEN_ANGLE)
            log_event({"event": "gate_open", "slot": bay + 1})
        else:
            entry_state = DENIED
            n_denied += 1
            log_event({"event": "denied_full"})

    elif entry_state == OPEN:
        # don't close on the car
        if (time.ticks_diff(now, entry_opened) >= MIN_OPEN
                and time.ticks_diff(now, entry_sensor.last_seen) >= GATE_CLEAR):
            move_servo(entry_servo, CLOSED_ANGLE)
            next_bay = None
            entry_state = IDLE

    elif entry_state == DENIED:
        # wait until the refused car has gone
        if not car and time.ticks_diff(now, entry_sensor.last_seen) >= GATE_CLEAR:
            entry_state = IDLE


def run_exit(now):
    # a car only gets out if there's a bill waiting for it
    global exit_state, exit_opened, exit_bill, n_exits, revenue
    car = exit_sensor.state

    if exit_state == IDLE:
        if not car:
            return
        if bills:
            exit_bill = bills.pop(0)
            n_exits += 1
            revenue += exit_bill["fee"]
            exit_opened = now
            exit_state = OPEN
            move_servo(exit_servo, OPEN_ANGLE)
            log_event({"event": "gate_exit", "slot": exit_bill["slot"],
                       "duration_s": exit_bill["duration_s"], "fee": exit_bill["fee"]})
        else:
            exit_state = DENIED
            log_event({"event": "exit_denied", "reason": "no bill"})

    elif exit_state == OPEN:
        if (time.ticks_diff(now, exit_opened) >= MIN_OPEN
                and time.ticks_diff(now, exit_sensor.last_seen) >= GATE_CLEAR):
            move_servo(exit_servo, CLOSED_ANGLE)
            exit_bill = None
            exit_state = IDLE

    elif exit_state == DENIED:
        if not car and time.ticks_diff(now, exit_sensor.last_seen) >= GATE_CLEAR:
            exit_state = IDLE


def update_outputs():
    free = len(free_bays())

    if entry_state == OPEN or exit_state == OPEN:
        lights(False, True, False)
    elif free == 0:
        lights(False, False, True)
    else:
        lights(True, False, False)

    show(0, "Free Slots: {}/{}".format(free, TOTAL_SLOTS))
    if entry_state == OPEN:
        show(1, "Go to Bay #{}".format(next_bay + 1))
    elif exit_state == OPEN:
        show(1, "Bay {} Fee Rs {}".format(exit_bill["slot"], exit_bill["fee"]))
    elif entry_state == DENIED:
        show(1, "DENIED: Full!")
    elif exit_state == DENIED:
        show(1, "No bill found")
    else:
        show(1, "{:<8}Cloud:{}".format("FULL!" if free == 0 else "Ready",
                                       "ON" if mqtt_ok else "--"))


def step(now):
    for i in range(TOTAL_SLOTS):
        if slots[i].update(now):
            slot_changed(i)
        time.sleep_ms(5)            # tiny gap so the sensors don't hear each other
    entry_sensor.update(now)
    exit_sensor.update(now)
    expire_reservations(now)
    run_entry(now)
    run_exit(now)
    update_outputs()
    check_phone()
    send_status(now)
    reconnect(now)


def main():
    global last_reconnect
    move_servo(entry_servo, CLOSED_ANGLE)
    move_servo(exit_servo, CLOSED_ANGLE)
    lights(False, False, False)

    lcd.clear()
    show(0, "Smart Parking")
    show(1, "Connecting WiFi")
    if wifi_connect():
        sync_time()
        mqtt_connect()
    last_reconnect = time.ticks_ms()

    lcd.clear()
    lcd_text[0] = lcd_text[1] = ""

    while True:
        step(time.ticks_ms())
        time.sleep_ms(50)


if __name__ == "__main__":
    main()