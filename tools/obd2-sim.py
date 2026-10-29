import can
import time
from datetime import datetime
import random

# --- Conversion Functions (Mimicking Custom/can_conv_funcs.c) ---
def can_ConvPercent(inval):
    return int((inval * 255) / 100)

def can_ConvAbsoluteMAP(inval):
    return int(inval)

def can_ConvVehicleSpeed(inval):
    return int(inval)

def can_ConvertRPM(inval):
    return int(inval / 4)

def can_ConvTemp1(inval):
    return int(inval + 40)

def can_ConvTemp2(inval):
    return int((inval / 10 - 40))

def can_ConvFuelTrim(inval):
    return int((inval / 1.28 - 100))

def can_ConvO2Group3(inval):
    # return int(((inval/65536) * 2))
    return int(((inval*65536) / 2))

def can_ConvTargetAFR(inval):
    return int((inval * 65536) / 2)

def can_ConvMazdaOilTemp(inval):
    return int((inval + 40) * 100)

def can_ConvMazdaAtfTemp(inval):
    return int((inval * 16))
def can_ConvTimingAdvance(inval):
    return int((inval + 64) * 2.0)


# Mapping of PIDs to conversion functions
PID_CONVERSION_MAP = {
    0x04: can_ConvPercent,
    0x06: can_ConvFuelTrim,
    0x0B: can_ConvAbsoluteMAP,
    0x0C: can_ConvertRPM,
    0x0D: can_ConvVehicleSpeed,
    0x0E: can_ConvTimingAdvance,
    0x11: can_ConvPercent,
    0x34: can_ConvO2Group3,
    0x3C: can_ConvTemp2,
    0x44: can_ConvTargetAFR,
    0x45: can_ConvPercent,
    0x05: can_ConvTemp1,
    0x0F: can_ConvTemp1,
    0x2F: can_ConvPercent,
    0x1310: can_ConvMazdaOilTemp,
    0x1E1C: can_ConvMazdaAtfTemp
}

# --- Simulation Values (Raw Input uint32_t) ---
SIM_VALUES = {
    0x04: 127, 0x06: 1280, 0x0B: 5000, 0x0C: 2500, 0x0D: 50, 0x0E: 80, 0x11: 127, 0x34: 1.0, 0x3C: 500, 0x44: 1.0, 0x45: 127, 0x05: 88, 0x0F: 88, 0x2F: 80
}

# Simulate Mazda specific PIDs (MAZDA is defined in Custom/includes.h)
SIM_VALUES[0x1310] = 80 # Oil Temp (e.g., 90 C)
SIM_VALUES[0x1E1C] = 107 # ATF Temp (e.g., 60 C)

# --- Core Simulation Logic ---
def simulate_can_message(pid):
    # target_obd2_value = SIM_VALUES.get(pid, 0)
    
    conv_func = PID_CONVERSION_MAP.get(pid)
    
    if conv_func is None:
        # If no conversion function is defined, return a default 8-byte packet with zeros
        return [0] * 8 

    # raw_value = SIM_VALUES.get(pid, 0)
    raw_value = SIM_VALUES[pid]
    multiplier = float(random.randrange(80,120)/100)
    # multiplier = random.uniform(0.8, 1.2)
    fluctuated_raw_value = raw_value * multiplier
    
    conv_func = PID_CONVERSION_MAP.get(pid)
    
    if conv_func is None:
        # If no conversion function is defined, return a default 8-byte packet with zeros
        return [0] * 8 

    # Apply conversion
    converted_value = conv_func(fluctuated_raw_value)
    # Convert the 16-bit converted value to 2 bytes
    data_bytes = converted_value.to_bytes(2, byteorder='big')
    # print(data_bytes)
    print(pid,converted_value,list(data_bytes))

    mode = 1
    if pid in [0x1310, 0x1E1C]:
        mode = 0x22 # Mazda mode
    
    # Calculate length based on 2 bytes of data
    if mode == 1:
        length = len(data_bytes) + 2
        if(pid == 0x34): length+=2
        byte0 = length
        byte1 = 0x41 # OBD Mode
        pid_bytes = pid.to_bytes(1, byteorder='big')
        txdat = [byte0, byte1] + list(pid_bytes) + list(data_bytes)
    elif mode == 0x22:
        length = len(data_bytes) + 3
        byte0 = length
        byte1 = 0x62 # OBD Mode
        pid_bytes = pid.to_bytes(2, byteorder='big')
        txdat = [byte0, byte1] + list(pid_bytes) + list(data_bytes)
    else:
        txdat = [0] * 8 
        
    return (txdat + [0] * (8 - len(txdat)))[:8]

# --- Initialization and Main Loop ---
def init():
    """Initializes the CAN bus connection."""
    try:
        # Using slcan on COM4, mirroring typical setup
        return can.interface.Bus(bustype="slcan", channel="COM4", bitrate=500000)
    except Exception as e:
        print(f"Error connecting to CAN bus: {e}")
        return None

def main():
    """The main execution loop for the CAN simulator."""
    print("Starting OBD2 Simulation (Listening for requests)...")
    
    bus = init()
    if bus is None:
        print("Failed to initialize CAN bus. Exiting.")
        return

    print("CAN bus connected successfully. Waiting for CAN requests...")

    fast_pids = [0x04, 0x06, 0x0B, 0x0C, 0x0D, 0x0E, 0x11, 0x34, 0x3C, 0x44, 0x45]
    slow_pids = [0x05, 0x0F, 0x2F]
    slow_pids.extend([0x1310, 0x1E1C])
    all_pids = fast_pids + slow_pids

    try:
        while True:
            rx = bus.recv(0.0005)
            
            if rx:
                # time.sleep(0.002)
                data = list(rx.data)
                pid = 0
                
                # Check OBD Mode 1/Mode 22 structure (data[1] == 0x41)
                # Check OBD Mode 01/Mode 62 structure
                if len(data) >= 3:
                    if data[1] == 0x01:
                        pid = data[2]
                    elif data[1] == 0x22:
                        pid = (data[2] << 8) | data[3]
                    else:
                        print(f"Received message with unknown mode or structure. Skipping. Data: {data}")
                        continue
                else:
                    print(f"Received message with insufficient data length. Skipping. Data: {data}")
                    continue

                if pid in all_pids:
                    txdat = simulate_can_message(pid)
                    tx_data_bytes = bytes(txdat)
                    tx = can.Message(arbitration_id=0x7E8, is_extended_id=False, data=tx_data_bytes, dlc=8)
                    
                    print(f"-> Received request for PID {hex(pid)}. Sending normal response: {txdat}")
                    bus.send(tx)
                else:
                    print(f"Received PID {hex(pid)} not in supported list. Ignoring request.")

    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    except Exception as e:
        print(f"An error occurred during simulation: {e}")
    finally:
        if bus:
            bus.shutdown()
            print("CAN bus shut down.")

if __name__ == "__main__":
    main()