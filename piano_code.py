from gpiozero import OutputDevice, InputDevice
from piano_mapping import KEY_MAP, NOTE_NAMES
import time
import mido
import threading
import math

# MIDI output setup
outport = mido.open_output('Midi Through:Midi Through Port-0 14:0')

# Constants
LEFT = 0
RIGHT = 1

# Pin Assignments
LEFT_A0_PIN, LEFT_A1_PIN, LEFT_A2_PIN = 23, 24, 25
LEFT_PL_PIN, LEFT_CLK_PIN, LEFT_DATA_PIN = 16, 20, 21
LEFT_NUM_COLS, LEFT_NUM_ROWS = 8, 10

RIGHT_A0_PIN, RIGHT_A1_PIN, RIGHT_A2_PIN = 17, 27, 22
RIGHT_PL_PIN, RIGHT_CLK_PIN, RIGHT_DATA_PIN = 19, 26, 5
RIGHT_NUM_COLS, RIGHT_NUM_ROWS = 8, 12

def delta_time_to_velocity(dt, curve='default'):
    dt_ms = dt * 1000
    if dt_ms < 1:
        return 127
    elif dt_ms > 100:
        return 1
    if curve == 'linear':
        velocity = 127 - int((dt_ms / 100) * 126)
    elif curve == 'log':
        velocity = int(127 * math.exp(-0.03 * dt_ms))
    elif curve == 'hard':
        velocity = int(127 * pow(1 - (dt_ms / 100), 2))
    elif curve == 'soft':
        velocity = int(127 * pow(1 - (dt_ms / 100), 0.5))
    else:
        velocity = int(127 * pow(1 - (dt_ms / 100), 0.8))
    return max(1, min(127, velocity))

class KeyboardScanner:
    def __init__(self, side, side_value, a0, a1, a2, pl, clk, data, cols, rows):
        self.side = side
        self.side_value = side_value
        self.num_cols = cols
        self.num_rows = rows

        self.A0 = OutputDevice(a0, initial_value=False)
        self.A1 = OutputDevice(a1, initial_value=False)
        self.A2 = OutputDevice(a2, initial_value=False)
        self.PL = OutputDevice(pl, initial_value=True)
        self.CLK = OutputDevice(clk, initial_value=False)
        self.DATA = InputDevice(data, pull_up=False)

        self.active_keys = set()
        self.top_timestamps = {}
        self.bottom_timestamps = {}
        self.pending_notes = {}  # key_id -> (key_tuple, timestamp)

    def set_column(self, col):
        self.A0.value = bool(col & 0x01)
        self.A1.value = bool(col & 0x02)
        self.A2.value = bool(col & 0x04)

    def read_shift_registers_16bits(self):
        self.PL.value = False
        time.sleep(0.0000001)
        self.PL.value = True

        value = 0
        for _ in range(16):
            value = (value << 1) | self.DATA.value
            self.CLK.value = True
            time.sleep(0.0000005)
            self.CLK.value = False
            time.sleep(0.0000005)
        return value

    def scan_keyboard(self):
        old_key_states = [False] * (self.num_cols * self.num_rows)

        try:
            while True:
                current_time = time.time()
                new_key_states = []

                for col in range(self.num_cols):
                    self.set_column(col)
                    data_16 = self.read_shift_registers_16bits()
                    new_key_states.extend([(data_16 >> row) & 1 == 1 for row in range(self.num_rows)])

                changed = [i for i, (o, n) in enumerate(zip(old_key_states, new_key_states)) if o != n]

                for idx in changed:
                    col = idx // self.num_rows
                    row = idx % self.num_rows
                    state = new_key_states[idx]
                    key_id = (self.side_value, col)
                    key_tuple = (self.side_value, col, row)

                    if state:
                        if row % 2 == 1:
                            self.top_timestamps[key_tuple] = current_time
                            print(f"[DEBUG] Top contact at {key_tuple} time={current_time:.6f}")
                        else:
                            self.bottom_timestamps[key_tuple] = current_time
                            print(f"[DEBUG] Bottom contact at {key_tuple} time={current_time:.6f}")
                            self.pending_notes[key_id] = (key_tuple, current_time)
                    else:
                        if row % 2 == 0:
                            self.active_keys.discard(key_id)
                            midi_note = KEY_MAP.get(key_tuple)
                            if midi_note is not None:
                                print(f"[-] Note OFF: {NOTE_NAMES.get(midi_note, midi_note)}")
                                outport.send(mido.Message('note_off', note=midi_note, velocity=0))
                        self.top_timestamps.pop(key_tuple, None)
                        self.bottom_timestamps.pop(key_tuple, None)

                # Process pending bottom contacts
                to_remove = []
                for key_id, (bottom_key, bottom_time) in self.pending_notes.items():
                    col = bottom_key[1]
                    row = bottom_key[2]
                    top_key = (self.side_value, col, row + 1)
                    if top_key in self.top_timestamps:
                        dt = bottom_time - self.top_timestamps[top_key]
                        print(f"[DEBUG] dt between contacts: {dt:.6f} sec")
                        velocity = delta_time_to_velocity(dt)
                        midi_note = KEY_MAP.get(bottom_key)
                        if midi_note is not None:
                            print(f"[+] Note ON: {NOTE_NAMES.get(midi_note, midi_note)} Velocity={velocity}")
                            outport.send(mido.Message('note_on', note=midi_note, velocity=velocity))
                            self.active_keys.add(key_id)
                        to_remove.append(key_id)
                    elif current_time - bottom_time > 0.03:
                        # Timeout: discard if nothing happened for 30ms
                        to_remove.append(key_id)

                for key_id in to_remove:
                    self.pending_notes.pop(key_id, None)

                old_key_states = new_key_states


        except KeyboardInterrupt:
            print(f"{self.side} scanner exiting...")

if __name__ == "__main__":
    left = KeyboardScanner("LEFT", LEFT, LEFT_A0_PIN, LEFT_A1_PIN, LEFT_A2_PIN,
                           LEFT_PL_PIN, LEFT_CLK_PIN, LEFT_DATA_PIN, LEFT_NUM_COLS, LEFT_NUM_ROWS)
    right = KeyboardScanner("RIGHT", RIGHT, RIGHT_A0_PIN, RIGHT_A1_PIN, RIGHT_A2_PIN,
                            RIGHT_PL_PIN, RIGHT_CLK_PIN, RIGHT_DATA_PIN, RIGHT_NUM_COLS, RIGHT_NUM_ROWS)

    lt = threading.Thread(target=left.scan_keyboard)
    rt = threading.Thread(target=right.scan_keyboard)

    lt.start()
    rt.start()

    try:
        lt.join()
        rt.join()
    except KeyboardInterrupt:
        print("Main program exiting...")