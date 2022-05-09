#!/usr/bin/env python3
import argparse
import os
import serial
import serial.tools.list_ports
import struct
import sys
import threading
import time
from collections import deque
from datetime import datetime
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

# Victron packages
sys.path.insert(1, os.path.join(os.path.dirname(__file__), '/opt/victronenergy/dbus-systemcalc-py/ext/velib_python'))
import dbus.service
import ve_utils
from vedbus import VeDbusService
from settingsdevice import SettingsDevice

class SmartBMSSerial:
    BMS_COMM_TIMEOUT = 10 # Seconds
    BMS_COMM_BLOCK_SIZE = 58

    BATTERY_CHARGE_STATE_BULKABSORPTION = 1
    BATTERY_CHARGE_STATE_STORAGE = 2
    BATTERY_CHARGE_STATE_ERROR = 3
     
    def __init__(
        self,
        loop,
        dev
    ):
        self.loop = loop
        
        self.last_received = 0
        self.battery_voltage = 0
        self.charge_current = 0
        self.discharge_current = 0
        self.battery_current = 0
        self.soc = 0
        self.lowest_cell_voltage = 0
        self.lowest_cell_voltage_num = 0
        self.highest_cell_voltage = 0
        self.highest_cell_voltage_num = 0
        self.lowest_cell_temperature = 0
        self.lowest_cell_temperature_num = 0
        self.highest_cell_temperature = 0
        self.highest_cell_temperature_num = 0
        self.cell_count = 0
        self.capacity = 0
        self.capacity_ah = 0
        self.energy_stored_wh = 0
        self.ah_stored = 0
        self.cell_voltage_min = 0
        self.cell_voltage_max = 0
        self.cell_voltage_full = 0
        self.time_to_go = 0
        self.alarm_minimum_voltage = 0
        self.alarm_maximum_voltage = 0
        self.alarm_minimum_temperature = 0
        self.alarm_maximum_temperature = 0
        self.alarm_cell_communication = 0
        self.allowed_to_charge = None
        self.allowed_to_discharge = None
        self.nominal_battery_voltage = 0
        self.consumed_ah = 0
        self.battery_charge_state = self.BATTERY_CHARGE_STATE_BULKABSORPTION
        self._battery_full_counter = 0
        self._balanced_timer = 0
        self._unbalance_detection_timer = 0
        
        self._comm_error_shadow = False
        self._current_filter = deque()

        self.lock = threading.Lock()
        self._poller = threading.Thread(target=lambda:self._poll(dev))
        self._poller.daemon = True
        self._poller.start()
    
    @property
    def alarm_serial_communication(self):
        if time.time() > self.last_received + self.BMS_COMM_TIMEOUT:
            return True
        else:
            return False
    
    def determine_nominal_cell_voltage(self):
        if self.cell_voltage_full >= 3.4 and self.cell_voltage_full <= 3.6: return 3.3
        if self.cell_voltage_full >= 3.9 and self.cell_voltage_full <= 4.4: return 3.7
        if self.cell_voltage_full >= 2.5 and self.cell_voltage_full <= 2.7: return 2.3
        return 0
    
    # Must be called every second
    def update(self):
        self._calculate_consumed_ah()
        self._update_time_to_go()
        self._battery_charge_state()
            
        if self.alarm_serial_communication and not self._comm_error_shadow:
            self._comm_error_shadow = True
            print('Serial comm error')
        if not self.alarm_serial_communication and self._comm_error_shadow:
            self._comm_error_shadow = False
            print('Serial comm restored')
    
    def _poll(self, dev, test_packet = ''):
        try:
            # The SmartBMS transmits each 500ms or 1000ms a message containing 58 bytes
            # When the serial does not contain any new bytes and no complete message was received, empty the buffer and wait for a new message
            buffer = bytearray (self.BMS_COMM_BLOCK_SIZE)
            buffer_index = 0
            time.sleep(0.5)
            self._ser = serial.Serial(dev, 9600)

            while(1):
                if len(test_packet) > 0:
                    read_data = test_packet
                else:
                    waiting_bytes = self._ser.in_waiting
                    read_data = self._ser.read(waiting_bytes)

                if len(read_data) > 0:
                    for c in read_data:
                        if buffer_index <= self.BMS_COMM_BLOCK_SIZE-1:
                            buffer[buffer_index] = c
                        buffer_index += 1
        
                if buffer_index == self.BMS_COMM_BLOCK_SIZE:
                        checksum = 0
                        for i in range (self.BMS_COMM_BLOCK_SIZE-1):
                            checksum += buffer[i]
                        received_checksum = buffer[self.BMS_COMM_BLOCK_SIZE-1]
                        if(checksum & 0xff) == received_checksum:
                            self.lock.acquire()
                            self.cell_voltage_full = self._decode_voltage(buffer[55:57]) # Value important for estimating nominal voltage, keep on top
                            self.cell_voltage_min = self._decode_voltage(buffer[51:53])
                            self.cell_voltage_max = self._decode_voltage(buffer[53:55])
                            self.battery_voltage = round(self._decode_voltage(buffer[0:3]), 2)
                            self.charge_current = self._decode_current(buffer[3:6])
                            self.discharge_current = self._decode_current(buffer[6:9])
                            self.battery_current = self._decode_current(buffer[9:12])
                            self.soc = buffer[40]
                            self.lowest_cell_voltage = self._decode_voltage(buffer[12:14])
                            self.lowest_cell_voltage_num = buffer[14]
                            self.highest_cell_voltage = self._decode_voltage(buffer[15:17])
                            self.highest_cell_voltage_num = buffer[17]
                            self.lowest_cell_temperature = self._decode_temperature(buffer[18:20])
                            self.lowest_cell_temperature_num = buffer[20]
                            self.highest_cell_temperature = self._decode_temperature(buffer[21:23])
                            self.highest_cell_temperature_num = buffer[23]
                            self.cell_count = buffer[25]
                            self.energy_stored_wh = self._decode_value(buffer[34:37], 1)
                            self.capacity = round(self._decode_value(buffer[49:51], 0.1), 1) # in kWh
                            self.nominal_battery_voltage = self.determine_nominal_cell_voltage()*self.cell_count
                            if self.nominal_battery_voltage != 0:
                                self.capacity_ah = self.capacity*1000/self.nominal_battery_voltage
                                self.energy_stored_ah = self.energy_stored_wh/self.nominal_battery_voltage
                            else:
                                self.capacity_ah = 0
                                self.energy_stored_ah = 0
                            self.energy_stored_wh = self._decode_value(buffer[34:37], 1)
                            self.alarm_minimum_voltage = True if (buffer[30] & 0b00001000) else False
                            self.alarm_maximum_voltage = True if (buffer[30] & 0b00010000) else False
                            self.alarm_minimum_temperature = True if (buffer[30] & 0b00100000) else False
                            self.alarm_maximum_temperature = True if (buffer[30] & 0b01000000) else False
                            self.alarm_cell_communication = True if (buffer[30] & 0b00000100) else False
                            self.allowed_to_discharge = True if (buffer[30] & 0b00000010) else False
                            self.allowed_to_charge = True if (buffer[30] & 0b00000001) else False
                            self.last_received = time.time()
                            self.lock.release()
                
                        buffer_index = 0
                elif len(read_data) == 0:
                    buffer_index = 0

                time.sleep(0.2)
        except Exception as e:
            print('Fatal exception: ')
            print(e)
            self.loop.quit()

    def _decode_current(self, raw_value):
        if raw_value[0] == ord('X'):
            return 0
        elif raw_value[0] == ord('-'):
            factor = -1
        else:
            factor = 1
        return factor*round(0.125*struct.unpack('>H', raw_value[1:3])[0],1)

    def _decode_value(self, raw_value, multiplier):
        if len(raw_value) == 3:
            value = struct.unpack('>L', bytearray(b'\x00')+raw_value)[0]
        else:
            value = struct.unpack('>H', raw_value[0:2])[0]
        return round(multiplier*value, 3)

    def _decode_voltage(self, raw_value):
        return self._decode_value(raw_value, 0.005)

    def _decode_temperature(self, raw_value):
        return round(struct.unpack('>H', raw_value[0:2])[0]*0.857-232,0)
        
    def _calculate_consumed_ah(self):
        self.consumed_ah = round(-1*(self.capacity_ah-self.energy_stored_ah),1)+0 # Add zero to remove negative sigh from -0.0

    def _update_time_to_go(self):
        # Filter current with a 3 minute moving average filter to stabilize the time-to-go
        if len(self._current_filter) >= 180:
            self._current_filter.popleft()
        self._current_filter.append(self.battery_current)
        
        current_filter_sum = 0
        for value in self._current_filter:
            current_filter_sum += value
        current_filter_average = current_filter_sum/len(self._current_filter)
        battery_power_filtered = (self.nominal_battery_voltage * -1 * current_filter_average)
        normalized_power = self.capacity*1000*0.05 # The battery capacity was rated at a current of <= 0.05C -> calculate this measurement current (in wh)
        if current_filter_average < 0 and battery_power_filtered > 0 and normalized_power > 0 and self.capacity > 0: # > 0 to avoid divide by zero
            # When discharge power is bigger than normalized current, use Peukert-like formula
            if battery_power_filtered > normalized_power:
                time_to_go_from_full =  60 * 60 * (self.capacity*1000)/(pow(battery_power_filtered/normalized_power, 1.02))/normalized_power
                time_to_go = time_to_go_from_full*(self.energy_stored_wh/(self.capacity*1000))
            else:
                time_to_go = self.energy_stored_wh * 60 * 60 / battery_power_filtered
            self.time_to_go = time_to_go
        else:
            self.time_to_go = None
            
    def _battery_charge_state(self):
        battery_current = self.battery_current
        if self.lowest_cell_voltage >= self.cell_voltage_full:
           battery_current -= 1 # When all cells balance, the charge current is 1A lower because of passive balancing
        if self.battery_charge_state == self.BATTERY_CHARGE_STATE_BULKABSORPTION:
            if self.lowest_cell_voltage >= self.cell_voltage_full and battery_current < self.capacity_ah*0.05:
                self._battery_full_counter += 1
            else:
                self._battery_full_counter = 0
            if self._battery_full_counter >= 60 and self.soc == 100: # When BMS also sees the pack as full
                self.battery_charge_state = self.BATTERY_CHARGE_STATE_STORAGE
        elif self.battery_charge_state == self.BATTERY_CHARGE_STATE_STORAGE:
            # Battery idle and unbalance of more than 40mV
            if self.capacity_ah*-0.05 < battery_current < self.capacity_ah*0.05 and self.highest_cell_voltage < self.cell_voltage_full and self.highest_cell_voltage - self.lowest_cell_voltage >= 0.04:
                self._unbalance_detection_timer += 1
            else:
                self._unbalance_detection_timer = 0
            
            # At least balance once a week
            self._balanced_timer += 1

            # At least 60 seconds in a row a voltage difference of at least 40mV? Unbalance detected
            if self._unbalance_detection_timer > 60 or self._balanced_timer >= 7*24*60*60:
                self._unbalance_detection_timer = 0
                self._balanced_timer = 0
                self.battery_charge_state = self.BATTERY_CHARGE_STATE_BULKABSORPTION
        else:
             self.battery_charge_state = self.BATTERY_CHARGE_STATE_BULKABSORPTION

class SmartBMSToDbus(SmartBMSSerial):
    def __init__(self, loop, dev, serial_id):
        super().__init__(loop, dev)
        self._dev = dev
        self._serial_id = serial_id
        
        self._info = {
            'name'      : "123SmartBMS",
            'servicename' : "123SmartBMS",
            'id'          : 0,
            'version'    : 1.3
        }

        device_port = args.device[dev.rfind('/') + 1:]
        device_port_num = device_port[device_port.rfind('USB') + 3:]
        self._device_instance = 288+int(device_port_num)
        self._dbusservice = VeDbusService("com.victronenergy.battery." + device_port)
        
        # Create the management objects, as specified in the ccgx dbus-api document
        self._dbusservice.add_path('/Mgmt/ProcessName', __file__)
        self._dbusservice.add_path('/Mgmt/ProcessVersion', self._info['version'])
        self._dbusservice.add_path('/Mgmt/Connection', ' Serial ' + dev)

        # Create the basic objects
        self._dbusservice.add_path('/DeviceInstance', self._device_instance)
        self._dbusservice.add_path('/ProductId',     self._info['id'])
        self._dbusservice.add_path('/ProductName',     self._info['name'])
        self._dbusservice.add_path('/FirmwareVersion', self._info['version'], gettextcallback=lambda p, v: "v{:.2f}".format(v))
        self._dbusservice.add_path('/HardwareVersion', None)
        self._dbusservice.add_path('/Serial', self._serial_id)
        self._dbusservice.add_path('/Connected', None)

        # Create device list
        self._dbusservice.add_path('/Devices/0/DeviceInstance',  self._device_instance)
        self._dbusservice.add_path('/Devices/0/FirmwareVersion', self._info['version'])
        self._dbusservice.add_path('/Devices/0/ProductId',       self._info['id'])
        self._dbusservice.add_path('/Devices/0/ProductName',   self._info['name'])
        self._dbusservice.add_path('/Devices/0/ServiceName',   self._info['servicename'])
        self._dbusservice.add_path('/Devices/0/VregLink',     "(API)")

        # Create the bms paths
        self._dbusservice.add_path('/TimeToGo',                             None)
        self._dbusservice.add_path('/SystemSwitch',                         None)
        self._dbusservice.add_path('/Soc',                                  None, gettextcallback=lambda p, v: "{:.0f}%%".format(v))
        self._dbusservice.add_path('/Capacity',                             None, gettextcallback=lambda p, v: "{:.1f}kWh".format(v))
        self._dbusservice.add_path('/InstalledCapacity',                    None, gettextcallback=lambda p, v: "{:.1f}kWh".format(v))
        self._dbusservice.add_path('/ConsumedAmphours',                     None, gettextcallback=lambda p, v: "{:.1f}Ah".format(v))
        self._dbusservice.add_path('/UpdateTimestamp',                      None)
        self._dbusservice.add_path('/Dc/0/Voltage',                         None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Dc/0/Current',                         None, gettextcallback=lambda p, v: "{:.1f}A".format(v))
        self._dbusservice.add_path('/Dc/0/Power',                           None, gettextcallback=lambda p, v: "{:.0f}W".format(v))
        self._dbusservice.add_path('/Dc/0/Temperature',                     None)
        self._dbusservice.add_path('/Io/AllowToCharge',                     None)
        self._dbusservice.add_path('/Io/AllowToDischarge',                  None)
        #self._dbusservice.add_path('/Voltages/Cell1',                      None)
        #self._dbusservice.add_path('/Voltages/Cell2',                      None)
        self._dbusservice.add_path('/System/MaxCellVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/System/MinCellVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/System/MinVoltageCellId',              None)
        self._dbusservice.add_path('/System/MaxVoltageCellId',              None)
        self._dbusservice.add_path('/System/MinCellTemperature',            None)
        self._dbusservice.add_path('/System/MinTemperatureCellId',          None)
        self._dbusservice.add_path('/System/MaxCellTemperature',            None)
        self._dbusservice.add_path('/System/MaxTemperatureCellId',          None)
        self._dbusservice.add_path('/System/NrOfModulesOnline',             None)
        self._dbusservice.add_path('/System/NrOfModulesOffline',            None)
        self._dbusservice.add_path('/System/NrOfModulesBlockingCharge',     None)
        self._dbusservice.add_path('/System/NrOfModulesBlockingDischarge',  None)
        self._dbusservice.add_path('/System/BatteryChargeState',            None) 
        self._dbusservice.add_path('/System/LowVoltageThreshold',           None)
        self._dbusservice.add_path('/System/HighVoltageThreshold',          None)
        self._dbusservice.add_path('/System/FullVoltageThreshold',          None) 
        self._dbusservice.add_path('/System/NrOfCells',                     None) 
        self._dbusservice.add_path('/Alarms/LowVoltage',                    None)
        self._dbusservice.add_path('/Alarms/HighVoltage',                   None)
        self._dbusservice.add_path('/Alarms/LowTemperature',                None)
        self._dbusservice.add_path('/Alarms/HighTemperature',               None)
        
        # Register persistent settings
        self._settings_register()
        # Register paths which can be externally changed, for example via the GUI
        self._dbusservice.add_path('/CustomName', value=self._settings['CustomName'], writeable=True, onchangecallback=self._settext)
    
    def update(self):
        super().update()

        if self.alarm_cell_communication or self.alarm_serial_communication:
            self._dbusservice["/Connected"] = 1
            self._dbusservice["/Soc"] = None
            self._dbusservice["/SystemSwitch"] = None
            self._dbusservice["/ConsumedAmphours"] = None
            self._dbusservice["/Capacity"] = None
            self._dbusservice["/InstalledCapacity"] = None
            self._dbusservice['/TimeToGo'] = None
            self._dbusservice["/Dc/0/Voltage"] = None
            self._dbusservice["/Dc/0/Current"] =None
            self._dbusservice["/Dc/0/Power"] = None
            self._dbusservice["/Dc/0/Temperature"] = None
            self._dbusservice["/Io/AllowToCharge"] = None
            self._dbusservice["/Io/AllowToDischarge"] = None
            self._dbusservice["/System/MinCellVoltage"] = None
            self._dbusservice["/System/MinVoltageCellId"] = None
            self._dbusservice["/System/MaxCellVoltage"] = None
            self._dbusservice["/System/MaxVoltageCellId"] = None
            self._dbusservice["/System/MinCellTemperature"] = None
            self._dbusservice["/System/MinTemperatureCellId"] = None
            self._dbusservice["/System/MaxCellTemperature"] = None
            self._dbusservice["/System/MaxTemperatureCellId"] = None
            self._dbusservice["/System/NrOfModulesOnline"] = 0
            self._dbusservice["/System/NrOfModulesOffline"] = 1
            self._dbusservice["/System/NrOfModulesBlockingCharge"] = None
            self._dbusservice["/System/NrOfModulesBlockingDischarge"] = None
            self._dbusservice["/System/BatteryChargeState"] = None
            self._dbusservice["/System/LowVoltageThreshold"] = None
            self._dbusservice["/System/HighVoltageThreshold"] = None
            self._dbusservice["/System/FullVoltageThreshold"] = None
            self._dbusservice["/System/NrOfCells"] = None
            self._dbusservice["/Alarms/LowVoltage"] = None
            self._dbusservice["/Alarms/HighVoltage"] = None
            self._dbusservice["/Alarms/LowTemperature"] = None
            self._dbusservice["/Alarms/HighTemperature"] = None
        else:
            self._dbusservice["/Connected"] = 1
            self._dbusservice["/Soc"] = self.soc
            self._dbusservice["/SystemSwitch"] = 1
            self._dbusservice["/ConsumedAmphours"] = self.consumed_ah
            self._dbusservice["/Capacity"] = round(self.energy_stored_ah, 1)
            self._dbusservice["/InstalledCapacity"] = self.capacity_ah
            self._dbusservice['/TimeToGo'] = self.time_to_go
            self._dbusservice["/Dc/0/Voltage"] = self.battery_voltage
            self._dbusservice["/Dc/0/Current"] = self.battery_current
            self._dbusservice["/Dc/0/Power"] = self.battery_voltage * self.battery_current
            self._dbusservice["/Dc/0/Temperature"] = self.highest_cell_temperature
            self._dbusservice["/Io/AllowToCharge"] = int(self.allowed_to_charge)
            self._dbusservice["/Io/AllowToDischarge"] = int(self.allowed_to_discharge)
            self._dbusservice["/System/MinCellVoltage"] = round(self.lowest_cell_voltage, 2)
            self._dbusservice["/System/MinVoltageCellId"] = self.lowest_cell_voltage_num
            self._dbusservice["/System/MaxCellVoltage"] = round(self.highest_cell_voltage, 2)
            self._dbusservice["/System/MaxVoltageCellId"] = self.highest_cell_voltage_num
            self._dbusservice["/System/MinCellTemperature"] = self.lowest_cell_temperature
            self._dbusservice["/System/MinTemperatureCellId"] = self.lowest_cell_temperature_num
            self._dbusservice["/System/MaxCellTemperature"] = self.highest_cell_temperature
            self._dbusservice["/System/MaxTemperatureCellId"] = self.highest_cell_temperature_num
            self._dbusservice["/System/NrOfModulesOnline"] = 1
            self._dbusservice["/System/NrOfModulesOffline"] = 0
            self._dbusservice["/System/NrOfModulesBlockingCharge"] = int(not self.allowed_to_charge)
            self._dbusservice["/System/NrOfModulesBlockingDischarge"] = int(not self.allowed_to_discharge)
            self._dbusservice["/System/BatteryChargeState"] = self.battery_charge_state
            self._dbusservice["/System/LowVoltageThreshold"] = self.cell_voltage_min
            self._dbusservice["/System/HighVoltageThreshold"] = self.cell_voltage_max
            self._dbusservice["/System/FullVoltageThreshold"] = self.cell_voltage_full
            self._dbusservice["/System/NrOfCells"] = self.cell_count
            self._dbusservice["/Alarms/LowVoltage"] = int(self.alarm_minimum_voltage)
            self._dbusservice["/Alarms/HighVoltage"] = int(self.alarm_maximum_voltage)
            self._dbusservice["/Alarms/LowTemperature"] = int(self.alarm_minimum_temperature)
            self._dbusservice["/Alarms/HighTemperature"] = int(self.alarm_maximum_temperature)
        
        # Beeds to be the last thing so others know we finished updating
        self._dbusservice["/UpdateTimestamp"] = int(time.time())
    
    def _settext(self, path, value): # Currently only used for CustomName
        self._settings['CustomName'] = value
        return True
        
    def _settings_handle_changed(self, setting, oldvalue, newvalue):
        return True

    def _settings_register(self):
        # Load all persistent data
        self._settings = SettingsDevice(
                dbus.SessionBus() if 'DBUS_SESSION_BUS_ADDRESS' in os.environ else dbus.SystemBus(),
                supportedSettings={
                'CustomName': ['/Settings/123electric/Products/'+ self._serial_id + '/CustomName', self._info['name'], 0, 0]
                },
                eventCallback = self._settings_handle_changed)

# Called on a one second timer
def handle_timer_tick():
    # The BMS data readout and variable writing happens on a different thread -> lock before
    bms_dbus.lock.acquire()
    bms_dbus.update()
    bms_dbus.lock.release()
    return True  # keep timer running

if __name__ == "__main__":  
    # Have a mainloop, so we can send/receive asynchronous calls to and from dbus
    print('123\\SmartBMS to dbus started')
    DBusGMainLoop(set_as_default=True)

    parser = argparse.ArgumentParser(description = '123SmartBMS to dbus')
    requiredArguments = parser.add_argument_group('required arguments')
    requiredArguments.add_argument('-d', '--device', help='serial device for data (eg /dev/ttyUSB0)', required=True)
    args = parser.parse_args()
    
    dev_objects = serial.tools.list_ports.comports()
    device_serial_numbers = {}
    for d in dev_objects:
        device_serial_numbers[d.device] = d.serial_number

    mainloop = GLib.MainLoop()
    bms_dbus = SmartBMSToDbus(mainloop, args.device, device_serial_numbers[args.device])

    time.sleep(3) # Wait until we have received some data

    GLib.timeout_add(1000, lambda: ve_utils.exit_on_error(handle_timer_tick))
    mainloop.run()
