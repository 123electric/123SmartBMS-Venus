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

class SmartBMS:
    BMS_COMM_TIMEOUT = 10 # Seconds
    BMS_COMM_BLOCK_SIZE = 58
    def __init__(
        self,
        dev
    ):
        self.last_received = 0
        self.pack_voltage = 0
        self.charge_current = 0
        self.discharge_current = 0
        self.pack_current = 0
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
        self.energy_stored = 0
        self.balance_voltage = 0
        self.alarm_minimum_voltage = 0
        self.alarm_maximum_voltage = 0
        self.alarm_minimum_temperature = 0
        self.alarm_maximum_temperature = 0
        self.alarm_cell_communication = 0
        self.allowed_to_charge = 0
        self.allowed_to_discharge = 0

        self.lock = threading.Lock()
        self._poller = threading.Thread(target=lambda:self._poll(dev))
        self._poller.daemon = True
        self._poller.start()

    @property
    def alarm_serial_communication(self):
        if(time.time() > self.last_received + self.BMS_COMM_TIMEOUT):
            return True
        else:
            return False
            
    def determine_nominal_voltage(self):
        if(self.balance_voltage >= 3.4 and self.balance_voltage <= 3.6): return 3.3
        if(self.balance_voltage >= 3.9 and self.balance_voltage <= 4.4): return 3.65
        if(self.balance_voltage >= 2.5 and self.balance_voltage <= 2.7): return 2.3
        return 0
    
    def _poll(self, dev, test_packet = ''):
        try:
            # The SmartBMS transmit each 500ms or 1000ms a message containing 58 bytes
            # When the serial does not contain any new bytes and no complete message was received, empty the buffer and wait for a new message
            #logging.debug("Parse Packet [" + str(len(packet)) + "] bytes")
            buffer = bytearray (self.BMS_COMM_BLOCK_SIZE)
            buffer_index = 0
            time.sleep(1)
            self._ser = serial.Serial(dev, 9600)

            while(1):
                if (len(test_packet) > 0):
                    read_data = test_packet
                else:
                    waiting_bytes = self._ser.in_waiting
                    read_data = self._ser.read(waiting_bytes)

                if(len(read_data) > 0):
                    for c in read_data:
                        if(buffer_index <= self.BMS_COMM_BLOCK_SIZE-1):
                            buffer[buffer_index] = c
                        buffer_index += 1
        
                if(buffer_index == self.BMS_COMM_BLOCK_SIZE):
                        checksum = 0
                        for i in range (self.BMS_COMM_BLOCK_SIZE-1):
                            checksum += buffer[i]
                        received_checksum = buffer[self.BMS_COMM_BLOCK_SIZE-1]
                        if((checksum & 0xff) == received_checksum):
                            self.lock.acquire()
                            self.pack_voltage = self._decode_voltage(buffer[0:3])
                            self.charge_current = self._decode_current(buffer[3:6])
                            self.discharge_current = self._decode_current(buffer[6:9])
                            self.pack_current = self._decode_current(buffer[9:12])
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
                            self.capacity = self._decode_value(buffer[49:51], 0.1)
                            self.energy_stored = self._decode_value(buffer[34:37], 1)
                            self.balance_voltage = self._decode_voltage(buffer[55:57])
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
                elif(len(read_data) == 0):
                    buffer_index = 0

                time.sleep(0.2)
        except Exception as e:
            print('Fatal exception: ')
            print(e)
            mainloop.quit()

    def _decode_current(self, raw_value):
        if(raw_value[0] == ord('X')):
            return 0
        elif(raw_value[0] == ord('-')):
            factor = -1
        else:
            factor = 1
        return factor*round(0.125*struct.unpack('>H', raw_value[1:3])[0],1)

    def _decode_value(self, raw_value, multiplier):
        if(len(raw_value) == 3):
            value = struct.unpack('>L', bytearray(b'\x00')+raw_value)[0]
        else:
            value = struct.unpack('>H', raw_value[0:2])[0]
        return round(multiplier*value,2)

    def _decode_voltage(self, raw_value):
        return self._decode_value(raw_value, 0.005)

    def _decode_temperature(self, raw_value):
        return round(struct.unpack('>H', raw_value[0:2])[0]*0.857-232,0)

class SmartBMSDbus():
    def __init__(self, dev, serial_id):
        self._BMS = SmartBMS(dev)
        self._dev = dev
        self._serial_id = serial_id
        self.comm_error_shadow = False
        self._current_filter = deque()
        self._nominal_pack_voltage = 0
        
        self._info = {
            'name'      : "123SmartBMS",
            'servicename' : "smartbms",
            'id'          : 0,
            'version'    : 0.6
        }

        self._gettexts = {
                        '/ConsumedAmphours': {'gettext': '%.1FAh'},
                        '/System/MaxCellVoltage': {'gettext': '%.2FV'},
                        '/System/MinCellVoltage': {'gettext': '%.2FV'},
                        '/Dc/0/Voltage': {'gettext': '%.2FV'},
                        '/Dc/0/Current': {'gettext': '%.1FA'},
                        '/Dc/0/Power': {'gettext': '%.0FW'},
                        '/Soc': {'gettext': '%.0F%%'},
                        '/Capacity': {'gettext': '%.1FkWh'},
                        '/InstalledCapacity': {'gettext': '%.1FkWh'}
        }
        
        device_port = args.device[dev.rfind('/') + 1:]
        device_port_num = device_port[device_port.rfind('USB') + 3:]
        self._dbusservice = VeDbusService("com.victronenergy.battery." + device_port)
        
        # Create the management objects, as specified in the ccgx dbus-api document
        self._dbusservice.add_path('/Mgmt/ProcessName', __file__)
        self._dbusservice.add_path('/Mgmt/ProcessVersion', self._info['version'])
        self._dbusservice.add_path('/Mgmt/Connection', ' Serial ' + dev)

        # Create the basic objects
        self._dbusservice.add_path('/DeviceInstance', 288+int(device_port_num))
        self._dbusservice.add_path('/ProductId',     self._info['id'])
        self._dbusservice.add_path('/ProductName',     self._info['name'])
        self._dbusservice.add_path('/FirmwareVersion', self._info['version'])
        self._dbusservice.add_path('/HardwareVersion', None)
        self._dbusservice.add_path('/Serial', self._serial_id)
        self._dbusservice.add_path('/Connected',     1)

        # Create device list
        self._dbusservice.add_path('/Devices/0/DeviceInstance',  0x288+int(device_port_num))
        self._dbusservice.add_path('/Devices/0/FirmwareVersion', self._info['version'])
        self._dbusservice.add_path('/Devices/0/ProductId',       self._info['id'])
        self._dbusservice.add_path('/Devices/0/ProductName',   self._info['name'])
        self._dbusservice.add_path('/Devices/0/ServiceName',   self._info['servicename'])
        self._dbusservice.add_path('/Devices/0/VregLink',     "(API)")

        # Create the bms paths
        self._dbusservice.add_path('/TimeToGo',                             None)
        self._dbusservice.add_path('/SystemSwitch',                         None)
        self._dbusservice.add_path('/Dc/0/Temperature',                     None)
        self._dbusservice.add_path('/Io/AllowToCharge',                     None)
        self._dbusservice.add_path('/Io/AllowToDischarge',                  None)
        self._dbusservice.add_path('/Info/UpdateTimestamp',                 None)
        #self._dbusservice.add_path('/Voltages/Cell1',                      None)
        #self._dbusservice.add_path('/Voltages/Cell2',                      None)
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
        self._dbusservice.add_path('/Alarms/LowVoltage',                    None)
        self._dbusservice.add_path('/Alarms/HighVoltage',                   None)
        self._dbusservice.add_path('/Alarms/LowTemperature',                None)
        self._dbusservice.add_path('/Alarms/HighTemperature',               None)
        
        # Register paths with custom texts
        for path in self._gettexts.keys():
            self._dbusservice.add_path(path, value=None, gettextcallback=self._gettext)
        
        # Register persistent settings
        self._settings_register()
        # Register paths which can be externally changed, for example via the GUI
        self._dbusservice.add_path('/CustomName', value=self._settings['CustomName'], writeable=True, onchangecallback=self._settext)
    
    def update(self):
        # The BMS data readout and variable writing happens on a different thread -> lock before
        self._BMS.lock.acquire()
        if(self._BMS.alarm_cell_communication or self._BMS.alarm_serial_communication):
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
            self._dbusservice["/Alarms/LowVoltage"] = None
            self._dbusservice["/Alarms/HighVoltage"] = None
            self._dbusservice["/Alarms/LowTemperature"] = None
            self._dbusservice["/Alarms/HighTemperature"] = None
        else:
            self._dbusservice["/Soc"] = self._BMS.soc
            self._dbusservice["/SystemSwitch"] = 1
            self._dbusservice["/Capacity"] = self._BMS.capacity
            self._dbusservice["/InstalledCapacity"] = self._BMS.capacity
            self._dbusservice["/Dc/0/Voltage"] = self._BMS.pack_voltage
            self._dbusservice["/Dc/0/Current"] = self._BMS.pack_current
            self._dbusservice["/Dc/0/Power"] = self._BMS.pack_voltage * self._BMS.pack_current
            self._dbusservice["/Dc/0/Temperature"] = self._BMS.highest_cell_temperature
            self._dbusservice["/Io/AllowToCharge"] = int(self._BMS.allowed_to_charge)
            self._dbusservice["/Io/AllowToDischarge"] = int(self._BMS.allowed_to_discharge)
            self._dbusservice["/System/MinCellVoltage"] = self._BMS.lowest_cell_voltage
            self._dbusservice["/System/MinVoltageCellId"] = self._BMS.lowest_cell_voltage_num
            self._dbusservice["/System/MaxCellVoltage"] = self._BMS.highest_cell_voltage
            self._dbusservice["/System/MaxVoltageCellId"] = self._BMS.highest_cell_voltage_num
            self._dbusservice["/System/MinCellTemperature"] = self._BMS.lowest_cell_temperature
            self._dbusservice["/System/MinTemperatureCellId"] = self._BMS.lowest_cell_temperature_num
            self._dbusservice["/System/MaxCellTemperature"] = self._BMS.highest_cell_temperature
            self._dbusservice["/System/MaxTemperatureCellId"] = self._BMS.highest_cell_temperature_num
            self._dbusservice["/System/NrOfModulesOnline"] = 1
            self._dbusservice["/System/NrOfModulesOffline"] = 0
            self._dbusservice["/System/NrOfModulesBlockingCharge"] = int(not self._BMS.allowed_to_charge)
            self._dbusservice["/System/NrOfModulesBlockingDischarge"] = int(not self._BMS.allowed_to_discharge)
            self._dbusservice["/Alarms/LowVoltage"] = int(self._BMS.alarm_minimum_voltage)
            self._dbusservice["/Alarms/HighVoltage"] = int(self._BMS.alarm_maximum_voltage)
            self._dbusservice["/Alarms/LowTemperature"] = int(self._BMS.alarm_minimum_temperature)
            self._dbusservice["/Alarms/HighTemperature"] = int(self._BMS.alarm_maximum_temperature)
            
            nominal_pack_voltage = self._BMS.determine_nominal_voltage()*self._BMS.cell_count
            # If no nominal pack voltage could be determined, just use current pack voltage
            if(nominal_pack_voltage == 0): nominal_pack_voltage = self._BMS.pack_voltage
            consumed_amp_hours = round(-1*(self._BMS.capacity*1000-self._BMS.energy_stored)/nominal_pack_voltage,1)
            if(consumed_amp_hours < 0.1): consumed_amp_hours = 0 # Convert negative zero to zero
            self._dbusservice["/ConsumedAmphours"] = consumed_amp_hours
            
            # Filter current with a 3 minute moving average filter to stabilize the time-to-go
            if(len(self._current_filter) >= 180):
                self._current_filter.popleft()
            self._current_filter.append(self._BMS.pack_current)
            
            current_filter_sum = 0
            for value in self._current_filter:
                current_filter_sum += value
            current_filter_average = current_filter_sum/len(self._current_filter)
            if current_filter_average < 0:
                self._dbusservice['/TimeToGo'] = (self._BMS.soc*self._BMS.capacity * 10) * 60 * 60 / (self._BMS.pack_voltage * -1 * current_filter_average)
            else:
                self._dbusservice['/TimeToGo'] = None
       
        if(self._BMS.alarm_serial_communication and not self._comm_error_shadow):
            self._comm_error_shadow = True
            print('Serial comm error')
        if(self._BMS.alarm_serial_communication and self._comm_error_shadow):
            self._comm_error_shadow = False
            print('Serial comm restored')
        
        self._BMS.lock.release()
    
    def _settext(self, path, value): # Currently only used for CustomName
        self._settings['CustomName'] = value
        return True
        
    def _gettext(self, path, value):
        item = self._gettexts.get(path)
        if item is not None:
            return item['gettext'] % value
        return str(value)

    # Currently nothing has to be done with the saved setting
    def _handle_setting_changed(self, setting, oldvalue, newvalue):
        return True

    def _settings_register(self):
        # Load all persistent data
        self._settings = SettingsDevice(
                dbus.SessionBus() if 'DBUS_SESSION_BUS_ADDRESS' in os.environ else dbus.SystemBus(),
                supportedSettings={
                'CustomName': ['/Settings/123electric/Products/'+ self._serial_id + '/CustomName', self._info['name'], 0, 0]
                },
                eventCallback = self._handle_setting_changed)

# Called on a one second timer
def handle_timer_tick():
    SmartBMSDbus.update()
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

    SmartBMSDbus = SmartBMSDbus(args.device, device_serial_numbers[args.device])

    time.sleep(3) # Wait until we have received some data

    GLib.timeout_add(1000, lambda: ve_utils.exit_on_error(handle_timer_tick))
    mainloop = GLib.MainLoop()
    mainloop.run()