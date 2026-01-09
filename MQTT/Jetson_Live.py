#!/usr/bin/env python3
import os
import time
import serial
import paho.mqtt.client as mqtt

# ----------------------------
# Serial (Bluetooth SPP)
# ----------------------------
SER_PORT = "/dev/rfcomm0"
SER_BAUD = 230400

# ----------------------------
# MQTT settings
# ----------------------------
BROKER_HOST = os.getenv("MQTT_HOST", "test.mosquitto.org")
BROKER_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USER = os.getenv("MQTT_USER", "")
MQTT_PASS = os.getenv("MQTT_PASS", "")

TOPIC_RAW = "imu/jetson01/raw"  # publishes the same CSV line as received

# ----------------------------
# Publish rate control
# ----------------------------
PUB_HZ = 20.0                 # publish rate for live view
PUB_PERIOD = 1.0 / PUB_HZ

# If you want to publish every received line, set PUB_HZ = 0
# (not recommended on LTE if IMU sends very fast)
# ----------------------------

EXPECTED_FIELDS = 12


def connect_serial():
    """Open rfcomm serial. Retry until success."""
    while True:
        try:
            ser = serial.Serial(SER_PORT, SER_BAUD, timeout=0)  # non-blocking
            print(f"✅ Serial connected: {SER_PORT}", flush=True)
            return ser
        except Exception as e:
            print(f"⚠️  Serial retry: {e}", flush=True)
            time.sleep(1)


def make_mqtt_client():
    """Create and connect an MQTT client with auto-reconnect behavior."""
    client_id = f"jetson01-{int(time.time())}"
    c = mqtt.Client(client_id=client_id, clean_session=True, protocol=mqtt.MQTTv311)

    if MQTT_USER:
        c.username_pw_set(MQTT_USER, MQTT_PASS)

    # Backoff for reconnections
    c.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("✅ MQTT connected", flush=True)
        else:
            print(f"⚠️  MQTT connect failed rc={rc}", flush=True)

    def on_disconnect(client, userdata, rc):
        print(f"⚠️  MQTT disconnected rc={rc} (will retry)", flush=True)

    c.on_connect = on_connect
    c.on_disconnect = on_disconnect

    while True:
        try:
            c.connect(BROKER_HOST, BROKER_PORT, keepalive=30)
            c.loop_start()
            return c
        except Exception as e:
            print(f"⚠️  MQTT retry: {e}", flush=True)
            time.sleep(2)


def is_valid_csv12(line: str) -> bool:
    """Light validation: 14 comma-separated fields."""
    parts = [p.strip() for p in line.split(",")]
    return len(parts) == EXPECTED_FIELDS


def main():
    ser = connect_serial()
    mqttc = make_mqtt_client()

    rx_buf = ""
    last_line = None

    last_pub_t = time.monotonic()

    try:
        while True:
            try:
                n = ser.in_waiting
                if n:
                    rx_buf += ser.read(n).decode("utf-8", errors="ignore")

                    # Process all complete lines; keep only the latest valid line
                    while "\n" in rx_buf:
                        line, rx_buf = rx_buf.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        if is_valid_csv12(line):
                            last_line = line

                now = time.monotonic()

                # Publish control:
                # - If PUB_HZ > 0: publish at fixed rate using the latest sample
                # - If PUB_HZ == 0: publish every time we get a valid line
                if PUB_HZ > 0:
                    if last_line is not None and (now - last_pub_t) >= PUB_PERIOD:
                        last_pub_t = now
                        # Publish the exact same CSV line (no JSON, same format)
                        mqttc.publish(TOPIC_RAW, last_line, qos=0, retain=False)
                else:
                    # publish every valid line
                    if last_line is not None:
                        mqttc.publish(TOPIC_RAW, last_line, qos=0, retain=False)
                        last_line = None  # consume it so we don't republish

                if n == 0:
                    time.sleep(0.001)

            except (serial.SerialException, OSError) as e:
                print(f"⚠️  Serial error: {e} -> reconnecting serial...", flush=True)
                try:
                    ser.close()
                except Exception:
                    pass
                time.sleep(1)
                ser = connect_serial()

            except Exception as e:
                # Keep running; MQTT loop thread handles reconnect attempts
                print(f"⚠️  Runtime warning: {e}", flush=True)
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Stopped.", flush=True)
    finally:
        try:
            ser.close()
        except Exception:
            pass
        try:
            mqttc.loop_stop()
            mqttc.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
