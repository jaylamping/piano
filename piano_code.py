from gpiozero import OutputDevice, InputDevice
from piano_mapping import KEY_MAP, NOTE_NAMES
import time
import math

#
# PIN ASSIGNMENTS
#

# Left CD74HC238 address pins
LEFT_A0_PIN = 1
LEFT_A1_PIN = 2
LEFT_A2_PIN = 3

# Left SN74HC165 control pins
LEFT_PL_PIN   = 4  # /PL (active-low parallel load)
LEFT_CLK_PIN  = 5  # CP (shift clock)
LEFT_DATA_PIN = 6  # Q7 output from the second SN74HC165

# Left Number of columns and rows
LEFT_NUM_COLS = 8
LEFT_NUM_ROWS = 10

# Right CD74HC238 address pins
RIGHT_A0_PIN = 17
RIGHT_A1_PIN = 27
RIGHT_A2_PIN = 22

# Right SN74HC165 control pins
RIGHT_PL_PIN   = 19  # /PL (active-low parallel load)
RIGHT_CLK_PIN  = 26  # CP (shift clock)
RIGHT_DATA_PIN = 5  # Q7 output from the second SN74HC165

# Right Number of columns and rows
RIGHT_NUM_COLS = 8
RIGHT_NUM_ROWS = 12 

#
# Variables
#

velocity_timings = {}
LEFT = 0
RIGHT = 1

#
# Helper Functions
#

def delta_time_to_velocity(dt):
    # Very simplistic scaling; you might use a more sophisticated formula or thresholds
    # The constant 0.01 is arbitrary and depends on how fast your scan loop is
    velocity = 127 - int(dt / 0.01)
    # Clamp to [1..127]
    return max(1, min(127, velocity))

#
# KeyboardScanner Class
#

class KeyboardScanner:
    def __init__(self, side, side_value, a0_pin, a1_pin, a2_pin, pl_pin, clk_pin, data_pin, num_cols, num_rows):
        self.side = side
        self.side_value = side_value
        self.num_cols = num_cols
        self.num_rows = num_rows

        # Initialize GPIO devices
        self.A0 = OutputDevice(a0_pin, initial_value=False)
        self.A1 = OutputDevice(a1_pin, initial_value=False)
        self.A2 = OutputDevice(a2_pin, initial_value=False)
        self.PL = OutputDevice(pl_pin, initial_value=True)
        self.CLK = OutputDevice(clk_pin, initial_value=False)
        self.DATA = InputDevice(data_pin, pull_up=False)

    def set_column(self, col_idx):
        self.A0.value = bool(col_idx & 0x01)
        self.A1.value = bool(col_idx & 0x02)
        self.A2.value = bool(col_idx & 0x04)

    def read_shift_registers_16bits(self):
        self.PL.value = False
        time.sleep(0.000002)
        self.PL.value = True

        value = 0
        for _ in range(16):
            value <<= 1
            bit_in = self.DATA.value
            value |= bit_in
            self.CLK.value = True
            time.sleep(0.000001)
            self.CLK.value = False
            time.sleep(0.000001)
        return value

    def scan_keyboard(self):
        print(f"Starting {self.side} keyboard scan. Press Ctrl+C to exit.")
        old_key_states = [False] * (self.num_cols * self.num_rows)
        
        try:
            while True:
                new_key_states = []
                for col in range(self.num_cols):
                    self.set_column(col)
                    data_16 = self.read_shift_registers_16bits()
                    
                    for row in range(self.num_rows):
                        bit_val = (data_16 >> row) & 1
                        pressed = (bit_val == 1)
                        new_key_states.append(pressed)

                # Compare states and handle changes
                for idx, (old_state, new_state) in enumerate(zip(old_key_states, new_key_states)):
                    if old_state != new_state:
                        col_idx = idx // self.num_rows
                        row_idx = idx % self.num_rows
                        
                        if new_state:
                            if row_idx % 2 == 0:
                                print(f"{self.side} key pressed at col={col_idx}, row={row_idx}")

                            if (self.side_value, col_idx, row_idx) in KEY_MAP:
                                midi_note = KEY_MAP[(self.side_value, col_idx, row_idx)]
                                note_name = NOTE_NAMES.get(midi_note, "Unknown")
                                print(f"Note: {note_name} (MIDI: {midi_note})")

                            velocity_timings[(self.side_value, col_idx, row_idx)] = time.time()

                            if row_idx % 2 == 0:
                                top_row = row_idx + 1
                            else:
                                top_row = row_idx - 1

                            if (self.side_value, col_idx, top_row) in velocity_timings:
                                dt = time.time() - velocity_timings[(self.side_value, col_idx, top_row)]
                                velocity = delta_time_to_velocity(dt)
                                print(f"{self.side} Velocity for (col={col_idx}): {velocity}")
                        else:
                            if row_idx % 2 == 0:
                                print(f"{self.side} key released at col={col_idx}, row={row_idx}")
                            if (self.side_value, col_idx, row_idx) in velocity_timings:
                                del velocity_timings[(self.side_value, col_idx, row_idx)]

                old_key_states = new_key_states
                time.sleep(0.05)

        except KeyboardInterrupt:
            pass
        print(f"{self.side} scanner exiting...")





#
# SETUP GPIOZERO DEVICES
#

# Define output devices for the 74HC238 address lines
# A0 = OutputDevice(RIGHTA0_PIN, initial_value=False)
# A1 = OutputDevice(A1_PIN, initial_value=False)
# A2 = OutputDevice(A2_PIN, initial_value=False)

# PL  = OutputDevice(PL_PIN,  initial_value=True)
# CLK = OutputDevice(CLK_PIN, initial_value=False)

# # The shift register data line is an input
# DATA = InputDevice(DATA_PIN, pull_up=False)

# #
# # HELPER FUNCTIONS
# #

# def set_column(col_idx):
#     """
#     Drive A0..A2 so that on the CD74HC238, one column output is HIGH (others LOW).
#     """
#     A0.value = bool(col_idx & 0x01)
#     A1.value = bool(col_idx & 0x02)
#     A2.value = bool(col_idx & 0x04)

# def read_shift_registers_16bits():
#     """
#     1) Pulse /PL low briefly to latch the parallel inputs of both SN74HC165 chips.
#     2) Shift out 16 bits by toggling the clock and reading DATA each time.
#     3) Return an integer where the 16 bits represent row states (0..15).
#     """
#     # Latch data: pull /PL low momentarily
#     PL.value = False
#     time.sleep(0.000002)  # brief pause
#     PL.value = True

#     value = 0
#     for _ in range(16):
#         # Shift left (bits come in from LSB)
#         value <<= 1
        
#         bit_in = DATA.value  # read the current data bit
#         value |= bit_in

#         # Pulse clock
#         CLK.value = True
#         time.sleep(0.000001)
#         CLK.value = False
#         time.sleep(0.000001)

#     return value

# def delta_time_to_velocity(dt):
#     # Very simplistic scaling; you might use a more sophisticated formula or thresholds
#     # The constant 0.01 is arbitrary and depends on how fast your scan loop is
#     velocity = 127 - int(dt / 0.01)
#     # Clamp to [1..127]
#     return max(1, min(127, velocity))

# #
# # MAIN SCAN LOOP
# #

# def right_main():
#     print("Starting keyboard scan with gpiozero. Press Ctrl+C to exit.")
    
#     # Track previous state for each key
#     old_key_states = [False] * (NUM_COLS * NUM_ROWS)
    
#     try:
#         while True:
#             new_key_states = []

#             for col in range(NUM_COLS):
#                 set_column(col)
                
#                 # Read the 16 bits from the shift registers
#                 data_16 = read_shift_registers_16bits()
                
#                 for row in range(NUM_ROWS):
#                     bit_val = (data_16 >> row) & 1
#                     pressed = (bit_val == 1)
#                     new_key_states.append(pressed)

#             # Compare new_key_states with old_key_states
#             for idx, (old_state, new_state) in enumerate(zip(old_key_states, new_key_states)):
#                 if old_state != new_state:
#                     # Compute row/col for convenience
#                     col_idx = idx // NUM_ROWS
#                     row_idx = idx % NUM_ROWS
                    
#                     if new_state:
#                         # key pressed
#                         if row_idx % 2 == 0: # only print for non velocity switches
#                             print(f"Key pressed at col={col_idx}, row={row_idx}")

#                         # mapping data
#                         if (RIGHT, col_idx, row_idx) in KEY_MAP:
#                             midi_note = KEY_MAP[(RIGHT, col_idx, row_idx)]
#                             note_name = NOTE_NAMES.get(midi_note, "Unknown")
#                             print(f"Note: {note_name} (MIDI: {midi_note})")

#                         # velocity data
#                         velocity_timings[(RIGHT, col_idx, row_idx)] = time.time()

#                         if row_idx % 2 == 0:
#                             top_row = row_idx + 1
#                         else:
#                             top_row = row_idx - 1

#                         if (RIGHT, col_idx, top_row) in velocity_timings:
#                             dt = time.time() - velocity_timings[(RIGHT, col_idx, top_row)]
#                             velocity = delta_time_to_velocity(dt)
#                             print(f"Velocity for (col={col_idx}): {velocity}")

#                     else:
#                         # key released
#                         if row_idx % 2 == 0: # only print for non velocity switches
#                             print(f"Key released at col={col_idx}, row={row_idx}")

#                         # clear velocity data
#                         if (RIGHT, col_idx, row_idx) in velocity_timings:
#                             del velocity_timings[(RIGHT, col_idx, row_idx)]

#             old_key_states = new_key_states
#             time.sleep(0.05)

#     except KeyboardInterrupt:
#         pass
#     print("Exiting...")


#
# MAIN PROGRAM
#

if __name__ == "__main__":
    import threading
    
    # Create scanner instances for both sides
    left_scanner = KeyboardScanner(
        "LEFT", LEFT, LEFT_A0_PIN, LEFT_A1_PIN, LEFT_A2_PIN,
        LEFT_PL_PIN, LEFT_CLK_PIN, LEFT_DATA_PIN, LEFT_NUM_COLS, LEFT_NUM_ROWS
    )
    right_scanner = KeyboardScanner(
        "RIGHT", RIGHT, RIGHT_A0_PIN, RIGHT_A1_PIN, RIGHT_A2_PIN,
        RIGHT_PL_PIN, RIGHT_CLK_PIN, RIGHT_DATA_PIN, RIGHT_NUM_COLS, RIGHT_NUM_ROWS
    )
    
    # Create threads for both scanners
    left_thread = threading.Thread(target=left_scanner.scan_keyboard)
    right_thread = threading.Thread(target=right_scanner.scan_keyboard)
    
    # Start both threads
    # left_thread.start()
    right_thread.start()
    
    # Wait for both threads to complete
    try:
        # left_thread.join()
        right_thread.join()
    except KeyboardInterrupt:
        print("Main program exiting...")