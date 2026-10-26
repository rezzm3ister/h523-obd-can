import can
import random
import time

bus = can.interface.Bus(bustype="slcan", channel="COM4",bitrate=500000)

while(1):
    rxdat = [0,0,0,0,0,0,0,0,0,0]
    try:
        rx = bus.recv(0.5)
        print(rx.dlc,list(rx.data),hex(rx.arbitration_id))
        rxdat = list(rx.data)
    except KeyboardInterrupt:
        exit()
    except Exception as E:
        print(E)
        None
    
    txdat = rxdat
    if rxdat[2] == 52:
        txdat[0] = 6
    else:
        txdat[0] =4
    txdat[1] = 0x41
    # txdat[3] = 0
    txdat[3] = int(0x50 * random.randint(15,25)/20)
    txdat[4] = int(0x50 * random.randint(15,25)/20)
    txdat[5] = int(0x50 * random.randint(15,25)/20)
    txdat[6] = int(0x50 * random.randint(15,25)/20)
    txdat[7] = 0xFF
    print(txdat)
    time.sleep(0.005)
    tx = can.Message(arbitration_id=0x7E8,is_extended_id=False,data=bytes(txdat),dlc=8)
    bus.send(tx)
