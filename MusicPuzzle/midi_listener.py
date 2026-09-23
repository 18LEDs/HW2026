"""
Children of the Pier - MIDI Box Puzzle Listener
-------------------------------------------------
Listens to a USB MIDI keyboard. When the correct sequence of notes is
played in a row, sends a trigger byte to the ESP32 over serial, which
opens the box via a servo.

SETUP (one-time):
    pip install mido python-rtmidi pyserial

HOW TO USE:
    1. Run this script with no changes first:  python midi_listener.py
       It will print a list of available MIDI input ports and serial
       ports, then exit. Copy the exact names into the CONFIG section
       below.
    2. Set TARGET_SEQUENCE to the notes you want guests to play.
    3. Run again. Leave it running for the duration of the party.
"""

import sys
import time
import mido
import serial
import serial.tools.list_ports

# ============================================================
# CONFIG - edit these before running for real
# ============================================================

# Exact name of your MIDI input port. Run this script once first to
# see the list, then paste the exact string here.
MIDI_PORT_NAME = "MPKmini2 0"

# Exact name of the ESP32's serial port (e.g. "COM5" on Windows, or
# "/dev/cu.usbserial-0001" / "/dev/ttyUSB0" on Mac/Linux).
SERIAL_PORT_NAME = "COM7"
SERIAL_BAUD = 115200

# The secret sequence, as MIDI note numbers, in the order they must be
# played. Middle C = 60. Each white/black key going up from there is
# +1. Easiest way to find note numbers for your chosen tune: run this
# script in "listen only" mode (see LISTEN_ONLY_MODE below) and just
# play the notes you want - it'll print the numbers as you play them.
TARGET_SEQUENCE = [60, 64, 67, 72]  # placeholder: C, E, G, high C

# If notes come in with too long a pause between them, the buffer
# resets, so someone can't just noodle through every key eventually.
# Tune this to feel fair for tipsy party guests - not so tight they
# get punished for a slightly slow, deliberate performance.
MAX_GAP_SECONDS = 4.0

# If True, the script only prints note numbers as you play them and
# never checks against TARGET_SEQUENCE or talks to the ESP32. Use this
# first to figure out the MIDI note numbers for the tune you want,
# before locking in TARGET_SEQUENCE above.
LISTEN_ONLY_MODE = False

# Byte sent to the ESP32 when the sequence is matched. Must match
# whatever the ESP32 firmware is checking for.
TRIGGER_BYTE = b"1"

# ============================================================
# Helpers
# ============================================================


def list_available_ports():
    print("\n--- Available MIDI input ports ---")
    midi_ports = mido.get_input_names()
    if not midi_ports:
        print("  (none found - is the keyboard plugged in and powered on?)")
    for p in midi_ports:
        print(f"  {p}")

    print("\n--- Available serial ports ---")
    serial_ports = list(serial.tools.list_ports.comports())
    if not serial_ports:
        print("  (none found - is the ESP32 plugged in?)")
    for p in serial_ports:
        print(f"  {p.device}  ({p.description})")
    print()


def connect_serial(port_name, baud):
    try:
        ser = serial.Serial(port_name, baud, timeout=1)
        time.sleep(2)  # give the ESP32 a moment to reset after the port opens
        print(f"Connected to ESP32 on {port_name}")
        return ser
    except Exception as e:
        print(f"Could not open serial port '{port_name}': {e}")
        print("Double check the port name and that nothing else "
              "(like the Arduino IDE serial monitor) has it open.")
        sys.exit(1)


def connect_midi(port_name):
    try:
        inport = mido.open_input(port_name)
        print(f"Listening on MIDI port: {port_name}")
        return inport
    except Exception as e:
        print(f"Could not open MIDI port '{port_name}': {e}")
        print("Double check the exact port name from the list above.")
        sys.exit(1)


# ============================================================
# Main
# ============================================================


def main():
    if MIDI_PORT_NAME.startswith("PASTE_") or SERIAL_PORT_NAME.startswith("PASTE_"):
        list_available_ports()
        print("Edit MIDI_PORT_NAME and SERIAL_PORT_NAME in this script "
              "with the exact names above, then run again.")
        return

    inport = connect_midi(MIDI_PORT_NAME)
    ser = None if LISTEN_ONLY_MODE else connect_serial(SERIAL_PORT_NAME, SERIAL_BAUD)

    buffer = []
    last_note_time = None

    mode_label = "LISTEN ONLY (no trigger, no target check)" if LISTEN_ONLY_MODE else "ARMED"
    print(f"Mode: {mode_label}")
    if not LISTEN_ONLY_MODE:
        print(f"Target sequence: {TARGET_SEQUENCE}")
    print("Waiting for notes... (Ctrl+C to quit)\n")

    try:
        for msg in inport:
            if msg.type != "note_on" or msg.velocity == 0:
                # note_on with velocity 0 is how many keyboards send "note off"
                continue

            now = time.time()

            if LISTEN_ONLY_MODE:
                print(f"Note played: {msg.note}")
                continue

            # Reset the buffer if too much time passed since the last note
            if last_note_time is not None and (now - last_note_time) > MAX_GAP_SECONDS:
                if buffer:
                    print("  (too slow, resetting)")
                buffer = []

            last_note_time = now
            buffer.append(msg.note)
            print(f"Note played: {msg.note}   buffer: {buffer}")

            # Keep buffer from growing longer than the target sequence
            if len(buffer) > len(TARGET_SEQUENCE):
                buffer = buffer[-len(TARGET_SEQUENCE):]

            if buffer == TARGET_SEQUENCE:
                print("\n*** CORRECT SEQUENCE - TRIGGERING BOX ***\n")
                ser.write(TRIGGER_BYTE)
                buffer = []
                last_note_time = None

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        inport.close()
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()