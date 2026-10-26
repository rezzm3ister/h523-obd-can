from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QComboBox, QMessageBox, QCheckBox, QGridLayout, QLabel
from PyQt6.QtCore import Qt,QTimer,QTimerEvent,QThread

import sys
import minimalmodbus
import serial
from serial.tools import list_ports
from tqdm import tqdm
import time
from datetime import datetime

app=QApplication(sys.argv)

MODB_SIZE=0x500
MODB_MAX_RD_REGS=0x7D
GRID_MAX_ROWS = 0x18
DIAG_GRID_ROW_START = 5

MAZDA = True

modb_db = [] #raw Modbus data
processed_modb_db = [] #processed Modbus data, to be used in GUI
com_port=None
modb_addr = None
modb = None
supported_registers = []
baudrate = 500000
supported_baudrates = [500000, 250000, 115200, 57600, 38400, 19200, 9600]

log_enabled = False
log_file = None
fast_mode = False

sys_run=False
modb = None

cycle_readtime=0

can_interval = 0

# PID list based on https://en.wikipedia.org/wiki/OBD-II_PIDs
OBD_PIDS = {
    0x00: "PIDs supported [01-20]",
    0x01: "Monitor status this drive",
    0x02: "Freeze DTC",
    0x03: "Fuel system status",
    0x04: "Calculated engine load",
    0x05: "Engine coolant temperature",
    0x06: "Short term fuel trim bank 1",
    0x07: "Long term fuel trim bank 1",
    0x08: "Short term fuel trim bank 2",
    0x09: "Long term fuel trim bank 2",
    0x0A: "Fuel pressure",
    0x0B: "Intake manifold absolute pressure",
    0x0C: "Engine RPM",
    0x0D: "Vehicle speed",
    0x0E: "Timing advance",
    0x0F: "Intake air temperature",
    0x10: "MAF air flow rate",
    0x11: "Throttle position",
    0x12: "Commanded secondary air status",
    0x13: "Oxygen sensors present",
    0x14: "Oxygen sensor 1",
    0x15: "Oxygen sensor 2",
    0x16: "Oxygen sensor 3",
    0x17: "Oxygen sensor 4",
    0x18: "Oxygen sensor 5",
    0x19: "Oxygen sensor 6",
    0x1A: "Oxygen sensor 7",
    0x1B: "Oxygen sensor 8",
    0x1C: "OBD standards this vehicle conforms to",
    0x1D: "Oxygen sensors present",
    0x1E: "Auxiliary input status",
    0x1F: "Run time since engine start",
    0x20: "PIDs supported [21-40]",
    0x21: "Distance traveled with malfunction indicator lamp (MIL) on",
    0x22: "Fuel rail pressure (diesel, or gasoline direct injection)",
    0x23: "Fuel rail gauge pressure (diesel, or gasoline direct injection)",
    0x24: "Oxygen sensor 1",
    0x25: "Oxygen sensor 2",
    0x26: "Oxygen sensor 3",
    0x27: "Oxygen sensor 4",
    0x28: "Oxygen sensor 5",
    0x29: "Oxygen sensor 6",
    0x2A: "Oxygen sensor 7",
    0x2B: "Oxygen sensor 8",
    0x2C: "Commanded EGR",
    0x2D: "EGR error",
    0x2E: "Commanded evaporative purge",
    0x2F: "Fuel tank level input",
    0x30: "Warmups since codes cleared",
    0x31: "Distance traveled since codes cleared",
    0x32: "Evap system vapor pressure",
    0x33: "Barometric pressure",
    0x34: "Oxygen sensor 1",
    0x35: "Oxygen sensor 2",
    0x36: "Oxygen sensor 3",
    0x37: "Oxygen sensor 4",
    0x38: "Oxygen sensor 5",
    0x39: "Oxygen sensor 6",
    0x3A: "Oxygen sensor 7",
    0x3B: "Oxygen sensor 8",
    0x3C: "Catalyst temperature bank 1 sensor 1",
    0x3D: "Catalyst temperature bank 1 sensor 2",
    0x3E: "Catalyst temperature bank 2 sensor 1",
    0x3F: "Catalyst temperature bank 2 sensor 2",
    0x40: "PIDs supported [41-60]",
    0x41: "Monitor status this drive",
    0x42: "Control module voltage",
    0x43: "Absolute load value",
    0x44: "Commanded air-fuel ratio",
    0x45: "Relative throttle position",
    0x46: "Ambient air temperature",
    0x47: "Absolute throttle position B",
    0x48: "Absolute throttle position C",
    0x49: "Accelerator pedal position D",
    0x4A: "Accelerator pedal position E",
    0x4B: "Accelerator pedal position F",
    0x4C: "Commanded throttle actuator",
    0x4D: "Time run with MIL on",
    0x4E: "Time since trouble codes cleared",
    0x4F: "Maximum value for fuel-air equivalence ratio",
    0x50: "Maximum value for air flow rate from mass air flow sensor",
    0x51: "Fuel type",
    0x52: "Ethanol fuel %",
    0x53: "Absolute Evap system vapor pressure",
    0x54: "Evap system vapor pressure",
    0x55: "Short term secondary oxygen sensor trim bank 1",
    0x56: "Long term secondary oxygen sensor trim bank 1",
    0x57: "Short term secondary oxygen sensor trim bank 2",
    0x58: "Long term secondary oxygen sensor trim bank 2",
    0x59: "Fuel rail absolute pressure",
    0x5A: "Relative accelerator pedal position",
    0x5B: "Hybrid battery pack remaining life",
    0x5C: "Engine oil temperature",
    0x5D: "Fuel injection timing",
    0x5E: "Engine fuel rate",
    0x5F: "Emission requirements to which vehicle is designed",
    0x60: "PIDs supported [61-80]",
    0x61: "Driver's demand engine - percent torque",
    0x62: "Actual engine - percent torque",
    0x63: "Engine reference torque",
    0x64: "Engine percent torque data",
    0x65: "Auxiliary input / output supported",
    0x66: "Mass air flow sensor",
    0x67: "Engine coolant temperature",
    0x68: "Intake air temperature",
    0x69: "Commanded EGR and EGR error",
    0x6A: "Commanded diesel intake air flow control",
    0x6B: "EGR temperature",
    0x6C: "Commanded throttle actuator control and relative throttle position",
    0x6D: "Fuel pressure control system",
    0x6E: "Injection pressure control system",
    0x6F: "Turbocharger compressor inlet pressure",
    0x70: "Boost pressure control",
    0x71: "Variable geometry turbo control",
    0x72: "Wastegate control",
    0x73: "Exhaust pressure",
    0x74: "Turbocharger RPM",
    0x75: "Turbocharger temperature",
    0x76: "Turbocharger temperature",
    0x77: "Charge air cooler temperature",
    0x78: "Exhaust gas recirculation temperature Bank 1",
    0x79: "Exhaust gas recirculation temperature Bank 2",
    0x7A: "DPF Differential pressure",
    0x7B: "DPF",
    0x7C: "DPF Temperature",
    0x7D: "NOx NTE control area status",
    0x7E: "NOx NTE Control Area Status",
    0x7F: "Engine run time",

    0x105: "X G-Force",
    0x106: "Y G-Force",
    0x107: "Z G-Force",
    0x108: "Total G-Force",
    0x109: "Total G-Force no Gravity",
    0x110: "MAZDA: OIL TEMP",
    0x111: "MAZDA: ATF TEMP",
    
    #anything after this is useless for now
}

class ModbThread(QThread):
    global modb_db, com_port, modb_addr, modb, baudrate, supported_baudrates, sys_run
    
    def get_supported_registers(self):
        global supported_registers
        
        # temp_supported_regs = modb.read_registers(0, 0x7D)
        # temp_supported_regs2 = modb.read_registers(0x7D, 0x7D)
        # temp_supported_regs += temp_supported_regs2
        # # temp_supported_regs.append(0x401)
        # # temp_supported_regs.append(0x402)
        # supported_registers = []
        # for i in range(len(temp_supported_regs)):
        #     if temp_supported_regs[i] > 0:
        #         supported_registers.append((i))
        # supported_registers.append(0x401)
        # supported_registers.append(0x402)
        supported_registers = [0x04, 0x05, 0x06, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x11, 0x2F, 0x34, 0x3C, 0x44, 0x45, 0x105,0x106,0x107,0x108,0x109]
        if MAZDA: supported_registers.extend([0x110,0x111])
        # print("Supported registers:", supported_registers)

    def process_modb_db(self):
        global modb_db, processed_modb_db
        #groups of registers that use the same formulas

        group1x = [0x0B, 0x0C, 0x0D, 0x3C]
        group10x = []
        group100x = [0x04, 0x05, 0x06, 0x0E, 0x0F, 0x11, 0x2F, 0x44, 0x45]
        group1000x = [0x34]
        group1000div = [0x105,0x106,0x107,0x108,0x109]

        val = 0

        for i in range(len(modb_db)):
            if i in supported_registers:
                val = float(modb_db[i])
                if(val>32767): val = 0-(65535-val)
                # if(val>32767): val = 32767-val

                if i in group10x:
                    val *= 10
                elif i in group100x:
                    val *= 100
                elif i in group1000x:
                    val *= 1000
                elif i in group1000div:
                    val /= 1000

            else:
                val = 0
            processed_modb_db[i] = val

    def run(self):
        global modb, cycle_readtime, can_interval
        global sys_run
        internal_cycle_readtime = time.time()
        print("Modbus thread started")
        while True:
            try:
                if sys_run:
                    # if(time.time() - fastmode_timer > 5):
                    #     if fast_mode:
                    #         modb.write_register(0x300,1,functioncode=6)
                    #     else:
                    #         modb.write_register(0x300,0,functioncode=6)
                    #     fastmode_timer = time.time()
                    # starttime = time.time()
                    # starttime = time.time()
                    internal_cycle_readtime = time.time()
                    rx_modb = modb.read_registers(0x00, MODB_MAX_RD_REGS)
                    for i in range(len(rx_modb)):
                        modb_db[i] = rx_modb[i]
                    rx_modb = modb.read_registers(0x100, MODB_MAX_RD_REGS)
                    for i in range(len(rx_modb)):
                        modb_db[i+0x100] = rx_modb[i]
                    rx_modb = modb.read_registers(0x200, MODB_MAX_RD_REGS)
                    for i in range(len(rx_modb)):
                        modb_db[i+0x200] = rx_modb[i]
                    # rx_modb = modb.read_registers(0x300, 10)
                    # for i in range(len(rx_modb)):
                    #     modb_db[i+0x300] = rx_modb[i]
                    # rx_modb = modb.read_registers(0x400,5)
                    # for i in range(len(rx_modb)):
                    #     modb_db[i+0x400] = rx_modb[i]

                    cycle_readtime = time.time() - internal_cycle_readtime
                    can_interval = modb_db[0x200]/10
                    self.process_modb_db()
                    if(log_enabled and log_file is not None):
                        log_line = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f") + ","
                        for reg in supported_registers:
                            log_line += str(processed_modb_db[reg]) + ","
                        log_file.write(log_line + "\n")
                    # print("Cycle read time:", cycle_readtime)
            except Exception as e:
                print("Error Occurred:",e)


class MainWindow(QMainWindow):
    global modb_db, com_port, modb_addr, modb, baudrate, supported_baudrates, processed_modb_db
    comport_selector = None
    modb_dev_addr_selector = None
    layout = QGridLayout()
    test_label = QLabel("Not Running")
    log_label = QLabel("Logging OFF")
    cycletimelabel = QLabel("Cycle read time:__")
    cantimelabel = QLabel("CAN read time:__")
    baudrate_widget = QComboBox()
    sys_enable = False
    sys_active_label=QLabel("System is inactive")
    # start_btn = 
    start_btn = QPushButton("Start/stop View")
    log_btn = QPushButton("Start/stop Logging")
    mainthread= ModbThread()
    # fastmode_checkbox = QCheckBox("Fast mode")
    # fastmode_checkbox.setChecked(False)
    test_reg=0
    test=0
    pid_widget_list = []
    pid_data_list = []
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Secondwave OBD2 Logger")

        mainlooptimer=QTimer(self)
        mainlooptimer.timeout.connect(self.main_loop)
        mainlooptimer.start(0)

        com_ports= list_ports.comports()
        self.init_modb_db()
        port_sel_btn = QLabel("Set Com Port")

        
        self.layout.addWidget(port_sel_btn,0,0)
        self.comport_selector = QComboBox()
        for port in com_ports:
            self.comport_selector.addItem(port.device)
        # port_sel_btn.clicked.connect(self.port_sel_click)

        self.layout.addWidget(self.comport_selector,0,1)
        central_widget = QWidget()
        central_widget.setLayout(self.layout)
        self.setCentralWidget(central_widget)

        self.modb_dev_addr_selector = QComboBox()
        for i in range(1, 256):
            self.modb_dev_addr_selector.addItem(str(hex(i)), i)
        self.modb_dev_addr_selector.setCurrentIndex(0x69-1)

        self.layout.addWidget(self.modb_dev_addr_selector, 1, 1)

        dev_set_btn = QLabel("Set Modbus Device Address")
        # dev_set_btn.clicked.connect(self.set_modb_dev_addr)
        self.layout.addWidget(dev_set_btn, 1, 0)

        # baudrate_widget = QComboBox()
        # for i in supported_baudrates:
        #     self.baudrate_widget.addItem(str(i), i)
        self.baudrate_widget.addItems(["500000","250000","115200","57600","38400","19200","9600"])
        # baudrate_widget.setCurrentIndex(0)
        self.layout.addWidget(self.baudrate_widget, 2, 1)
        dev_set_btn = QLabel("Set Baudrate")
        # dev_set_btn.clicked.connect(self.set_modb_baudrate)
        self.layout.addWidget(dev_set_btn, 2, 0)

        # self.test_label = QLabel("testreg")
        self.layout.addWidget(self.test_label, 3, 0)

        self.layout.addWidget(QLabel("LOG interval:"), 3, 1)
        self.layout.addWidget(QLabel("CAN interval:"), 4, 1)

        self.layout.addWidget(self.cycletimelabel, 3, 2)
        self.layout.addWidget(self.cantimelabel, 4, 2)

        # start_btn = QPushButton("Start/stop")
        # start_btn.setCheckable(True)
        self.start_btn.clicked.connect(self.start_stop)
        self.layout.addWidget(self.start_btn, 1, 2, 2,2)
        self.log_btn.clicked.connect(self.log_start_stop)
        self.layout.addWidget(self.log_btn, 1, 4, 2,2)
        self.layout.addWidget(self.log_label, 3, 4, 1, 2)


        # self.fastmode_checkbox.stateChanged.connect(self.getFastMode)
        # self.layout.addWidget(self.fastmode_checkbox, 0, 2,1,2)
        self.mainthread.start()

        

    # def port_sel_click(self):
    #     com_port = self.comport_selector.currentText()
    #     print("selected port:", com_port)
    # def set_modb_dev_addr(self):
    #     modb_addr = self.modb_dev_addr_selector.currentData()
    #     print("selected modbus device address:", modb_addr)
    # def set_modb_baudrate(self):
    #     baudrate = self.baudrate_widget.currentText()
    #     print("selected baudrate:", baudrate)

    def getFastMode(self):
        global fast_mode
        if self.fastmode_checkbox.isChecked():
            fast_mode = True
            print("Fast mode enabled")
        else:
            fast_mode = False
            print("Fast mode disabled")

    def add_pids(self):
        for i in supported_registers:
            self.pid_widget_list.append(QLabel(str(hex(i)) + " " + OBD_PIDS.get(i, "Unknown PID")))
            self.pid_data_list.append([QLabel("0"),i])
        # self.pid_widget_list.append(QLabel(str(hex(i)) + " " + OBD_PIDS.get(i, "Unknown PID")))
        # self.pid_data_list.append([QLabel("0"),i])
            # print("added PID:", hex(i))
        wrow=DIAG_GRID_ROW_START
        wcol=0
        print("Adding PIDs to layout")
        for i in tqdm(self.pid_widget_list):
            self.layout.addWidget(i,wrow,wcol)
            wrow += 1
            if wrow >= GRID_MAX_ROWS:
                wrow = DIAG_GRID_ROW_START
                wcol += 2
        wrow=DIAG_GRID_ROW_START
        wcol=1
        print("Adding PID data to layout")
        for i in tqdm(self.pid_data_list):
            i[0].setStyleSheet("background-color: white")
            self.layout.addWidget(i[0], wrow, wcol)
            wrow += 1
            if wrow >= GRID_MAX_ROWS:
                wrow = DIAG_GRID_ROW_START
                wcol += 2

    def log_start_stop(self):
        global log_enabled, log_file
        if not sys_run:
            QMessageBox.critical(self, "Error", f"System needs to be running to log data.\nPlease start the system first.")
            return

        log_enabled = not log_enabled
        if log_enabled:
            print("Logging started")
            self.log_label.setText("Logging ON")
            print(supported_registers)
            csv_header="Timestamp,"
            if log_file is None:
                log_file = open("obd2_log"+datetime.now().strftime("%Y%m%d_%H%M%S")+".csv", "w")
                for reg in supported_registers:
                    csv_header += str(OBD_PIDS[reg]) + ","
                    # csv_header += str(hex(reg)) + ","
                log_file.write(csv_header + "\n")
        else:
            print("Logging stopped")
            if log_file is not None:
                log_file.close()
                log_file = None
            self.log_label.setText("Logging OFF")
        None

    def start_stop(self):
        # print(self.comport_selector.currentText())
        # print(self.modb_dev_addr_selector.currentData())
        # print(self.baudrate_widget.currentText())
        global com_port, modb_addr, baudrate, modb_db, sys_run, modb

        com_port = self.comport_selector.currentText()
        modb_addr = self.modb_dev_addr_selector.currentData()
        baudrate = int(self.baudrate_widget.currentText())

        try:
            if not sys_run:
                modb = minimalmodbus.Instrument(com_port, slaveaddress=modb_addr)
                modb.serial.baudrate=baudrate
                modb.serial.bytesize=8
                modb.serial.parity=serial.PARITY_EVEN
                modb.read_register(0)
                print("getting supported registers")
                self.mainthread.get_supported_registers()
                print("supported registers found:")
                print(supported_registers)
                self.add_pids()
            sys_run = not sys_run
        except Exception as e:

            print("Error initializing Modbus:", e)
            sys_run = False
            QMessageBox.critical(self, "Error", f"Failed to initialize Modbus: {e}\nPlease check your configuration and try again.")
            return
        if sys_run:
            print("System started")
            self.test_label.setText("Running")
            # modb = minimalmodbus.Instrument(com_port, slaveaddress=modb_addr)
            # modb.serial.baudrate=baudrate
            # modb.serial.bytesize=8
            # modb.serial.parity=serial.PARITY_EVEN
        else:
            print("System stopped")
            self.test_label.setText("Not Running")
            for i in self.pid_data_list:
                self.layout.removeWidget(i[0])
            for i in self.pid_widget_list:
                self.layout.removeWidget(i)
            self.pid_widget_list.clear()
            self.pid_data_list.clear()

    def init_modb_db(self):
        for i in range(MODB_SIZE):
            modb_db.append(0)
            processed_modb_db.append(0)


    prevtime=time.time()
    def main_loop(self):
        # if time.time() - self.prevtime > 0.1:
        #     self.prevtime = time.time()
        #     for i in range(0x100):
        #         processed_modb_db[i]+=1
        #         if processed_modb_db[i] > 0xFFFF:
        #             processed_modb_db[i] = 0
        # self.test_label.setText("testreg: " + str(int(modb_db[0])))
        self.cycletimelabel.setText(str(round(cycle_readtime*1000,2)) + " ms")
        self.cantimelabel.setText(str(round(can_interval,2)) + " ms")
        for i in range(len(self.pid_data_list)):
            self.pid_data_list[i][0].setText(str(processed_modb_db[self.pid_data_list[i][1]]))
            if processed_modb_db[self.pid_data_list[i][1]] > 0:
                self.pid_data_list[i][0].setStyleSheet("background-color: lightgreen")
            else:
                self.pid_data_list[i][0].setStyleSheet("background-color: white")     
        # self.test_label = QLabel("testreg: " + str(int(modb_db[0])))
        # global modb_db, com_port, modb_addr, baudrate, sys_run
        # if sys_run  :
        #     try:
        #         modb = minimalmodbus.Instrument(com_port, slaveaddress=modb_addr)
        #         modb.serial.baudrate=baudrate
        #         modb.serial.bytesize=8
        #         modb.serial.parity=serial.PARITY_EVEN
        #         # Read holding registers as an example
        #         # for i in range(0, 0x300):
        #         modb_db[self.test_reg] = modb.read_register(self.test_reg, 0)  # Read register at address i
        #         self.test_reg += 1
        #         if self.test_reg >= 0x300:
        #             self.test_reg = 0
        #     except Exception as e:
        #         print("Error reading from Modbus:", e)
        #         self.start_stop()                                   


        

window=MainWindow()
window.setFixedSize(1024, 600)
window.show()
app.exec()