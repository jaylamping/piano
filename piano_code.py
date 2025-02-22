from gpiozero import OutputDevice, InputDevice
from piano_mapping import KEY_MAP, NOTE_NAMES
import time
import math

#
# PIN ASSIGNMENTS
#

# CD74HC238 address pins
A0_PIN = 17
A1_PIN = 27
A2_PIN = 22

# SN74HC165 control pins
PL_PIN   = 19  # /PL (active-low parallel load)
CLK_PIN  = 26  # CP (shift clock)
DATA_PIN = 5  # Q7 output from the second SN74HC165

# Number of columns and rows
NUM_COLS = 8
NUM_ROWS = 12 

#
# Data Structures
#
velocity_timings = {}
NOTES = [
    "C", "C#", "D", "D#", "E", "F",
    "F#", "G", "G#", "A", "A#", "B"
]
LEFT = 0
RIGHT = 1

#
# SETUP GPIOZERO DEVICES
#

# Define output devices for the 74HC238 address lines
A0 = OutputDevice(A0_PIN, initial_value=False)
A1 = OutputDevice(A1_PIN, initial_value=False)
A2 = OutputDevice(A2_PIN, initial_value=False)

PL  = OutputDevice(PL_PIN,  initial_value=True)
CLK = OutputDevice(CLK_PIN, initial_value=False)

# The shift register data line is an input
DATA = InputDevice(DATA_PIN, pull_up=False)

#
# HELPER FUNCTIONS
#

def set_column(col_idx):
    """
    Drive A0..A2 so that on the CD74HC238, one column output is HIGH (others LOW).
    """
    A0.value = bool(col_idx & 0x01)
    A1.value = bool(col_idx & 0x02)
    A2.value = bool(col_idx & 0x04)

def read_shift_registers_16bits():
    """
    1) Pulse /PL low briefly to latch the parallel inputs of both SN74HC165 chips.
    2) Shift out 16 bits by toggling the clock and reading DATA each time.
    3) Return an integer where the 16 bits represent row states (0..15).
    """
    # Latch data: pull /PL low momentarily
    PL.value = False
    time.sleep(0.000002)  # brief pause
    PL.value = True

    value = 0
    for _ in range(16):
        # Shift left (bits come in from LSB)
        value <<= 1
        
        bit_in = DATA.value  # read the current data bit
        value |= bit_in

        # Pulse clock
        CLK.value = True
        time.sleep(0.000001)
        CLK.value = False
        time.sleep(0.000001)

    return value

def delta_time_to_velocity(dt):
    # Very simplistic scaling; you might use a more sophisticated formula or thresholds
    # The constant 0.01 is arbitrary and depends on how fast your scan loop is
    velocity = 127 - int(dt / 0.01)
    # Clamp to [1..127]
    return max(1, min(127, velocity))

#
# MAIN SCAN LOOP
#

def right_main():
    print("Starting keyboard scan with gpiozero. Press Ctrl+C to exit.")
    
    # Track previous state for each key
    old_key_states = [False] * (NUM_COLS * NUM_ROWS)
    
    try:
        while True:
            new_key_states = []

            for col in range(NUM_COLS):
                set_column(col)
                
                # Read the 16 bits from the shift registers
                data_16 = read_shift_registers_16bits()
                
                for row in range(NUM_ROWS):
                    bit_val = (data_16 >> row) & 1
                    pressed = (bit_val == 1)
                    new_key_states.append(pressed)

            # Compare new_key_states with old_key_states
            for idx, (old_state, new_state) in enumerate(zip(old_key_states, new_key_states)):
                if old_state != new_state:
                    # Compute row/col for convenience
                    col_idx = idx // NUM_ROWS
                    row_idx = idx % NUM_ROWS
                    
                    if new_state:
                        # key pressed
                        if row_idx % 2 == 0: # only print for non velocity switches
                            print(f"Key pressed at col={col_idx}, row={row_idx}")

                        # mapping data
                        if (RIGHT, col_idx, row_idx) in KEY_MAP:
                            midi_note = KEY_MAP[(RIGHT, col_idx, row_idx)]
                            note_name = NOTE_NAMES.get(midi_note, "Unknown")
                            print(f"Note: {note_name} (MIDI: {midi_note})")

                        # velocity data
                        velocity_timings[(RIGHT, col_idx, row_idx)] = time.time()

                        if row_idx % 2 == 0:
                            top_row = row_idx + 1
                        else:
                            top_row = row_idx - 1

                        if (RIGHT, col_idx, top_row) in velocity_timings:
                            dt = time.time() - velocity_timings[(RIGHT, col_idx, top_row)]
                            velocity = delta_time_to_velocity(dt)
                            print(f"Velocity for (col={col_idx}): {velocity}")

                    else:
                        # key released
                        print(f"Key released at col={col_idx}, row={row_idx}")

                        # clear velocity data
                        if (RIGHT, col_idx, row_idx) in velocity_timings:
                            del velocity_timings[(RIGHT, col_idx, row_idx)]

            old_key_states = new_key_states
            time.sleep(0.05)

    except KeyboardInterrupt:
        pass
    print("Exiting...")

if __name__ == "__main__":
    right_main()
