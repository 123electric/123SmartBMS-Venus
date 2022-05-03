#!/usr/bin/env python3
import argparse
import os
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
from dbusmonitor import DbusMonitor
from vedbus import VeDbusService
from settingsdevice import SettingsDevice

class SmartBMSToDbus:
    LAST_SEEN_TIMEOUT = 30
    
    BATTERY_CHARGE_STATE_BULKABSORPTION = 1
    BATTERY_CHARGE_STATE_STORAGE = 2
    BATTERY_CHARGE_STATE_ERROR = 3

    BATTERY_DISCHARGE_MAX_RATING = 1.0
    BATTERY_CHARGE_MAX_RATING = 1.0

    def __init__(self, loop):
        self._info = {
            'name'      : "123SmartBMS master",
            'servicename' : "smartbmsmaster",
            'id'          : 0,
            'version'    : 0.1
        }
        
        dummy = {'code': None, 'whenToLog': 'configChange', 'accessLevel': None}
        dbus_tree = {
            'com.victronenergy.battery': {
                '/Connected': dummy,
                '/ProductName': dummy,
                '/Mgmt/Connection': dummy,
                '/DeviceInstance': dummy,
                '/Dc/0/Voltage': dummy,
                '/Dc/0/Current': dummy,
                '/Dc/0/Power': dummy,
                '/Soc': dummy,
                '/TimeToGo': dummy,
                '/ConsumedAmphours': dummy,
                '/Capacity': dummy,
                '/ProductId': dummy,
                '/System/MinCellVoltage': dummy,
                '/System/MaxCellVoltage': dummy,
                '/System/BatteryChargeState': dummy,
                '/System/LowVoltageThreshold': dummy,
                '/System/HighVoltageThreshold': dummy,
                '/System/FullVoltageThreshold': dummy,
                '/System/NrOfCells': dummy,
                '/Io/AllowToCharge': dummy,
                '/Io/AllowToDischarge': dummy},
                'com.victronenergy.system': {
                '/Connected': dummy,
                '/ProductName': dummy,
                '/Mgmt/Connection': dummy,
                '/DeviceInstance': dummy,
                '/Dc/Battery/Soc': dummy
                }
        }

        self._device_instance = 287
        self._dbusmonitor = DbusMonitor(dbus_tree, valueChangedCallback=self._dbus_value_changed,
            deviceAddedCallback=self._device_added, deviceRemovedCallback=self._device_removed)

        self._dbusservice = VeDbusService("com.victronenergy.battery.smartbmsmaster")
        
        # Create the management objects, as specified in the ccgx dbus-api document
        self._dbusservice.add_path('/Mgmt/ProcessName', __file__)
        self._dbusservice.add_path('/Mgmt/ProcessVersion', self._info['version'])
        self._dbusservice.add_path('/Mgmt/Connection', '')

        # Create the basic objects
        self._dbusservice.add_path('/DeviceInstance', self._device_instance)
        self._dbusservice.add_path('/ProductId',     self._info['id'])
        self._dbusservice.add_path('/ProductName',     self._info['name'])
        self._dbusservice.add_path('/FirmwareVersion', self._info['version'], gettextcallback=lambda p, v: "v{:.2f}".format(v))
        self._dbusservice.add_path('/HardwareVersion', None)
        self._dbusservice.add_path('/Serial', '')
        self._dbusservice.add_path('/Connected',     1)

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
        self._dbusservice.add_path('/Dc/0/Voltage',                         None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Dc/0/Current',                         None, gettextcallback=lambda p, v: "{:.1f}A".format(v))
        self._dbusservice.add_path('/Dc/0/Power',                           None, gettextcallback=lambda p, v: "{:.0f}W".format(v))
        #self._dbusservice.add_path('/Dc/0/Temperature',                     None)
        #self._dbusservice.add_path('/Io/AllowToCharge',                     None)
        #self._dbusservice.add_path('/Io/AllowToDischarge',                  None)
        #self._dbusservice.add_path('/Info/UpdateTimestamp',                 None)
        #self._dbusservice.add_path('/Voltages/Cell1',                      None)
        #self._dbusservice.add_path('/Voltages/Cell2',                      None)
        #self._dbusservice.add_path('/System/MaxCellVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        #self._dbusservice.add_path('/System/MinCellVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        #self._dbusservice.add_path('/System/MinVoltageCellId',              None)
        #self._dbusservice.add_path('/System/MaxVoltageCellId',              None)
        #self._dbusservice.add_path('/System/MinCellTemperature',            None)
        #self._dbusservice.add_path('/System/MinTemperatureCellId',          None)
        #self._dbusservice.add_path('/System/MaxCellTemperature',            None)
        #self._dbusservice.add_path('/System/MaxTemperatureCellId',          None)
        #self._dbusservice.add_path('/System/NrOfModulesOnline',             None)
        #self._dbusservice.add_path('/System/NrOfModulesOffline',            None)
        #self._dbusservice.add_path('/System/NrOfModulesBlockingCharge',     None)
        #self._dbusservice.add_path('/System/NrOfModulesBlockingDischarge',  None)
        #self._dbusservice.add_path('/System/BatteryChargeState',            None) 
        #self._dbusservice.add_path('/Info/BatteryLowVoltage',               None)
        #self._dbusservice.add_path('/Alarms/LowVoltage',                    None)
        #self._dbusservice.add_path('/Alarms/HighVoltage',                   None)
        #self._dbusservice.add_path('/Alarms/LowTemperature',                None)
        #self._dbusservice.add_path('/Alarms/HighTemperature',               None)
        #Only used for master BMS
        self._dbusservice.add_path('/Info/MaxChargeVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Info/MaxChargeCurrent',                None, gettextcallback=lambda p, v: "{:.2f}A".format(v))
        self._dbusservice.add_path('/Info/MaxDischargeCurrent',             None, gettextcallback=lambda p, v: "{:.2f}A".format(v))
        
        # Cache of all connected SmartBMSes
        self._connected_smartbmses = []

        # Only for master
        self._charge_voltage_controller_integral = 0
        self.charge_current_limit = 0
        self.max_charge_voltage = 0
        self.max_discharge_current = 0
        self.max_charge_current = 0
    
    def update(self):
        self._scan_connected_smartbmses()
        self._remove_disconnected_bmses()

        if len(self._connected_smartbmses) == 0:
            self._dbusservice["/Soc"] = None
            #self._dbusservice["/SystemSwitch"] = None
            #self._dbusservice["/ConsumedAmphours"] = None
            #self._dbusservice["/Capacity"] = None
            #self._dbusservice["/InstalledCapacity"] = None
            #self._dbusservice['/TimeToGo'] = None
            self._dbusservice["/Dc/0/Voltage"] = None
            self._dbusservice["/Dc/0/Current"] = None
            self._dbusservice["/Dc/0/Power"] = None
            #self._dbusservice["/Dc/0/Temperature"] = None
            #self._dbusservice["/Io/AllowToCharge"] = None
            #self._dbusservice["/Io/AllowToDischarge"] = None
            #self._dbusservice["/System/MinCellVoltage"] = None
            #self._dbusservice["/System/MinVoltageCellId"] = None
            #self._dbusservice["/System/MaxCellVoltage"] = None
            #self._dbusservice["/System/MaxVoltageCellId"] = None
            #self._dbusservice["/System/MinCellTemperature"] = None
            #self._dbusservice["/System/MinTemperatureCellId"] = None
            #self._dbusservice["/System/MaxCellTemperature"] = None
            #self._dbusservice["/System/MaxTemperatureCellId"] = None
            #self._dbusservice["/System/NrOfModulesOnline"] = 0
            #self._dbusservice["/System/NrOfModulesOffline"] = 1
            #self._dbusservice["/System/NrOfModulesBlockingCharge"] = None
            #self._dbusservice["/System/NrOfModulesBlockingDischarge"] = None
            #self._dbusservice["/System/BatteryChargeState"] = None
            #self._dbusservice["/Alarms/LowVoltage"] = None
            #self._dbusservice["/Alarms/HighVoltage"] = None
            #self._dbusservice["/Alarms/LowTemperature"] = None
            #self._dbusservice["/Alarms/HighTemperature"] = None
            #self._dbusservice["/Info/BatteryLowVoltage"] = None
        else:
            self._dbusservice["/Soc"] = self._get_bmses_average_soc()
            #self._dbusservice["/SystemSwitch"] = 1
            #self._dbusservice["/ConsumedAmphours"] = self.consumed_ah
            #self._dbusservice["/Capacity"] = round(self.energy_stored_ah, 1)
            #self._dbusservice["/InstalledCapacity"] = self.capacity_ah
            #self._dbusservice['/TimeToGo'] = self.time_to_go
            self._dbusservice["/Dc/0/Voltage"] = self._get_bmses_average_voltage()
            self._dbusservice["/Dc/0/Current"] = self._get_bmses_sum_current()
            self._dbusservice["/Dc/0/Power"] = self._get_bmses_sum_power()
            #self._dbusservice["/Dc/0/Temperature"] = self.highest_cell_temperature
            #self._dbusservice["/Io/AllowToCharge"] = int(self.allowed_to_charge)
            #self._dbusservice["/Io/AllowToDischarge"] = int(self.allowed_to_discharge)
            #self._dbusservice["/System/MinCellVoltage"] = round(self._get_lowest_bms_cell_voltage(), 2)
            #self._dbusservice["/System/MinVoltageCellId"] = self.lowest_cell_voltage_num
            #self._dbusservice["/System/MaxCellVoltage"] = round(self.highest_cell_voltage, 2)
            #self._dbusservice["/System/MaxVoltageCellId"] = self.highest_cell_voltage_num
            #self._dbusservice["/System/MinCellTemperature"] = self.lowest_cell_temperature
            #self._dbusservice["/System/MinTemperatureCellId"] = self.lowest_cell_temperature_num
            #self._dbusservice["/System/MaxCellTemperature"] = self.highest_cell_temperature
            #self._dbusservice["/System/MaxTemperatureCellId"] = self.highest_cell_temperature_num
            #self._dbusservice["/System/NrOfModulesOnline"] = 1
            #self._dbusservice["/System/NrOfModulesOffline"] = 0
            #self._dbusservice["/System/NrOfModulesBlockingCharge"] = int(not self.allowed_to_charge)
            #self._dbusservice["/System/NrOfModulesBlockingDischarge"] = int(not self.allowed_to_discharge)
            #self._dbusservice["/System/BatteryChargeState"] = self.battery_charge_state
            #self._dbusservice["/Alarms/LowVoltage"] = int(self.alarm_minimum_voltage)
            #self._dbusservice["/Alarms/HighVoltage"] = int(self.alarm_maximum_voltage)
            #self._dbusservice["/Alarms/LowTemperature"] = int(self.alarm_minimum_temperature)
            #self._dbusservice["/Alarms/HighTemperature"] = int(self.alarm_maximum_temperature)
            #self._dbusservice["/Info/BatteryLowVoltage"] = (self.cell_voltage_min * self.cell_count)
        
        self._calculate_current_limits()
        self._dbusservice["/Info/MaxChargeVoltage"] = self.max_charge_voltage
        self._dbusservice["/Info/MaxChargeCurrent"] = self.max_charge_current
        self._dbusservice["/Info/MaxDischargeCurrent"] = self.max_discharge_current

    def _handleservicechange(self):
        self._changed = True

    def _dbus_value_changed(self, dbusServiceName, dbusPath, dict, changes, deviceInstance):
        self._changed = True

        # Workaround because com.victronenergy.vebus is available even when there is no vebus product
        # connected.
        if (dbusPath in ['/Connected', '/ProductName', '/Mgmt/Connection'] or
            (dbusPath == '/State' and dbusServiceName.split('.')[0:3] == ['com', 'victronenergy', 'vebus'])):
            self._handleservicechange()

    def _device_added(self, service, instance, do_service_change=True):
        if do_service_change:
            self._handleservicechange()

    def _device_removed(self, service, instance):
        self._handleservicechange()
                
    def _get_connected_service_list(self, classfilter=None):
        services = self._dbusmonitor.get_service_list(classfilter=classfilter)
        return services
    
    def _scan_connected_smartbmses(self):
        batteries = self._get_connected_service_list('com.victronenergy.battery').items()
        print('Scanning connected SmartBMS...')
        self._connected_smartbmses.clear()
        for battery in batteries:
            product_name = self._dbusmonitor.get_value(battery[0], '/ProductName')
            connected = self._dbusmonitor.get_value(battery[0], '/Connected')
            print('Found ' + product_name)
            if product_name == '123SmartBMS' and connected == 1:
                print('Is a connected BMS')
                service = battery[0]
                device_instance = battery[1]
                matching_bms = list(filter(lambda b: b.service == service, self._connected_smartbmses))
                in_list = len(matching_bms)
                if in_list == 0:
                    print("Not in list, adding...")
                    # New: create new instance
                    found_bms = SmartBMSDbus(self._dbusmonitor, service, device_instance)
                    self._connected_smartbmses.append(found_bms)
                    self._update_bms_data(found_bms)
                else:
                    print("Already in list, updating...")
                    self._update_bms_data(matching_bms[0])

    def _remove_disconnected_bmses(self):
        for bms in self._connected_smartbmses:
            if time.time() > bms.last_seen + self.LAST_SEEN_TIMEOUT:
                self._connected_smartbmses.remove(bms)

    def _update_bms_data(self, bms):
        bms.last_seen = time.time()
        bms.lowest_voltage = self._dbusmonitor.get_value(bms.service, '/System/MinCellVoltage')
        bms.highest_voltage = self._dbusmonitor.get_value(bms.service, '/System/MaxCellVoltage')
        bms.capacity = self._dbusmonitor.get_value(bms.service, '/Capacity')
        bms.battery_charge_state = self._dbusmonitor.get_value(bms.service, '/System/BatteryChargeState')
        bms.allowed_to_charge = self._dbusmonitor.get_value(bms.service, '/Io/AllowToCharge') == 1
        bms.allowed_to_discharge = self._dbusmonitor.get_value(bms.service, '/Io/AllowToDischarge') == 1
        bms.soc = self._dbusmonitor.get_value(bms.service, '/Soc')
        bms.voltage = self._dbusmonitor.get_value(bms.service, '/Dc/0/Voltage')
        bms.current = self._dbusmonitor.get_value(bms.service, '/Dc/0/Current')
        bms.power = self._dbusmonitor.get_value(bms.service, '/Dc/0/Power')
        bms.cell_voltage_min = self._dbusmonitor.get_value(bms.service, '/System/LowVoltageThreshold')
        bms.cell_voltage_max = self._dbusmonitor.get_value(bms.service, '/System/HighVoltageThreshold')
        bms.cell_voltage_full = self._dbusmonitor.get_value(bms.service, '/System/FullVoltageThreshold')
        bms.cell_count = self._dbusmonitor.get_value(bms.service, '/System/NrOfCells')
        bms.communication_error = True if bms.soc == None else False
        
    def _get_system_soc(self):
        soc = self._dbusmonitor.get_value('com.victronenergy.system', '/Dc/Battery/Soc')
        return soc
        
    def _get_bmses_lowest_cell_voltage(self):
        lowest_voltage = None
        for bms in self._connected_smartbmses:
            bms_lowest_voltage = bms.lowest_cell_voltage
            if bms_lowest_voltage != None and (lowest_voltage == None or bms_lowest_voltage < lowest_voltage):
                lowest_voltage = bms_lowest_voltage
        return lowest_voltage
    
    def _get_bmses_highest_cell_voltage(self):
        highest_voltage = None
        for bms in self._connected_smartbmses:
            bms_highest_voltage = bms.highest_voltage
            if bms_highest_voltage != None and (highest_voltage == None or bms_highest_voltage > highest_voltage):
                highest_voltage = bms_highest_voltage
        return highest_voltage

    def _get_bmses_average_soc(self):
        soc_sum = 0
        bms_count = 0
        for bms in self._connected_smartbmses:
            if bms.soc != None:
                soc_sum += bms.soc
                bms_count += 1
        if bms_count == 0: return 0
        return soc_sum / bms_count

    def _get_bmses_average_voltage(self):
        voltage_sum = 0
        bms_count = 0
        for bms in self._connected_smartbmses:
            if bms.voltage != None:
                voltage_sum += bms.voltage
                bms_count += 1
        if bms_count == 0: return 0
        return voltage_sum / bms_count

    def _get_bmses_sum_power(self):
        sum = 0
        for bms in self._connected_smartbmses:
            if bms.power != None:
                sum += bms.power
        return sum

    def _get_bmses_sum_current(self):
        sum = 0
        for bms in self._connected_smartbmses:
            if bms.current != None:
                sum += bms.current
        return sum

    def _get_bmses_sum_communication_error(self):
        sum = 0
        for bms in self._connected_smartbmses:
            if bms.communication_error == True:
                sum += 1
        return sum

    def _get_bmses_cell_voltage_min(self):
        for bms in self._connected_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_min != None:
                return bms.cell_voltage_min
    
    def _get_bmses_cell_voltage_max(self):
        for bms in self._connected_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_max != None:
                return bms.cell_voltage_max
        return None
    
    def _get_bmses_cell_voltage_full(self):
        for bms in self._connected_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_full != None:
                return bms.cell_voltage_full
        return None

    def _get_bmses_cell_voltage_max(self):
        for bms in self._connected_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_max != None:
                return bms.cell_voltage_max
        return None

    def _get_bmses_cell_count(self):
        for bms in self._connected_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_count != None:
                return bms.cell_count
        return None
        
    def _calculate_current_limits(self):
        system_highest_cell_voltage = self._get_bmses_highest_cell_voltage()
        
        if self._get_bmses_sum_communication_error() > 0:
            max_discharge_current = 0
            max_charge_current = 0
            return
        
        # Calculate total, connected capacity for charging
        charge_capacity_sum = 0
        discharge_capacity_sum = 0
        for bms in self._connected_smartbmses:
            capacity = bms.capacity
            if capacity == None: capacity = 0
            if bms.allowed_to_charge: charge_capacity_sum += capacity
            if bms.allowed_to_discharge: discharge_capacity_sum += capacity
        
        # Discharge - use SoC for discharge current
        discharge_limit = discharge_capacity_sum*self.BATTERY_DISCHARGE_MAX_RATING
        system_soc = self._dbusmonitor.get_value('com.victronenergy.system', '/Dc/Battery/Soc')
        print("System highest cell voltage: ")
        print(system_highest_cell_voltage)
        print("Cell_voltage_min: ")
        print(self._get_bmses_cell_voltage_min())
        print("System SoC:")
        print(system_soc)
        if self._get_bmses_cell_voltage_min() == None or system_soc == None:
            self.max_discharge_current = 0
        elif system_highest_cell_voltage - 0.05 <= self._get_bmses_cell_voltage_min():
            self.max_discharge_current = 0
        elif system_soc <= 10:
            self.max_discharge_current = round(discharge_limit/8, 1)
        elif 10 < system_soc <= 20:
            self.max_discharge_current = round(discharge_limit/4, 1)
        elif 20 < system_soc <= 25:
            self.max_discharge_current = round(discharge_limit/2, 1)
        elif 25 < system_soc <= 30:
            self.max_discharge_current = round(discharge_limit/1.5, 1)
        else:
            self.max_discharge_current = round(discharge_limit, 1)
        
        # Charge
        # Fixed charge current - when pack is full, regulate the voltage
        if self._get_bmses_highest_cell_voltage() == None:
            self.max_charge_current = 0
        elif self._get_bmses_highest_cell_voltage()+0.05 >= self._get_bmses_cell_voltage_max(): # Pre-critical cutoff: should never happen. Avoid BMS triggering power cutoff
            self.max_charge_current = 0
        else:
            self.max_charge_current = charge_capacity_sum*self.BATTERY_CHARGE_MAX_RATING
        
        bmses_in_bulkabsorption = list(filter(lambda b: b.battery_charge_state == self.BATTERY_CHARGE_STATE_BULKABSORPTION, self._connected_smartbmses))
        # When not all BMSes are in storage state (balanced): keep CVL higher so battery can fully charge
        if len(bmses_in_bulkabsorption) > 0:
            highest_cell_voltage_target = round(self._get_bmses_cell_voltage_full() + 0.01, 3)
            voltage_upper_limit_cell_margin = 0.025
            voltage_upper_limit = (highest_cell_voltage_target + voltage_upper_limit_cell_margin) * self._get_bmses_cell_count()
            # If code just started, set value to default
            if(self.max_charge_voltage == 0): self.max_charge_voltage = voltage_upper_limit
            
            #print('Highest cell voltage target:\t{}'.format(highest_cell_voltage_target))
            #print('Highest cell voltage:\t\t{}'.format(self.highest_cell_voltage))
            # When battery is ~unbalanced, charge to a point where all cells are balancing.
            # When the battery is balanced, charge to a voltage a little lower so no balancing energy is wasted
            charge_voltage_controller_kp = 0.1
            charge_voltage_controller_ki = 0.03
            charge_voltage_controller_setpoint = highest_cell_voltage_target
            charge_voltage_controller_input = system_highest_cell_voltage
            charge_voltage_controller_error = charge_voltage_controller_setpoint - charge_voltage_controller_input # Negative value when over full threshold - example: -0.1 (100mV over threshold)
            if charge_voltage_controller_error <= 0:
                self._charge_voltage_controller_integral = self._charge_voltage_controller_integral + charge_voltage_controller_error
            else: # positive error, slowly rise
                self._charge_voltage_controller_integral = self._charge_voltage_controller_integral + (0.0001/charge_voltage_controller_ki)
            # limit integral, only lower voltage when needed
            if self._charge_voltage_controller_integral*charge_voltage_controller_ki > voltage_upper_limit_cell_margin: self._charge_voltage_controller_integral = voltage_upper_limit_cell_margin/charge_voltage_controller_ki
            if self._charge_voltage_controller_integral*charge_voltage_controller_ki < -0.2: self._charge_voltage_controller_integral = -0.2/charge_voltage_controller_ki
            #print('Controller error:\t\t{}'.format(charge_voltage_controller_error))
            #print('Controller integral:\t\t{}'.format(self._charge_voltage_controller_integral))
            charge_voltage_controller_output = round(charge_voltage_controller_kp * charge_voltage_controller_error + charge_voltage_controller_ki * self._charge_voltage_controller_integral, 3)
            #print('Controller ouput:\t\t{}'.format(charge_voltage_controller_output))
            charge_voltage = round((highest_cell_voltage_target + charge_voltage_controller_output) * self._get_bmses_cell_count(), 2)
            self.max_charge_voltage = charge_voltage if charge_voltage < voltage_upper_limit else voltage_upper_limit
        else:
            if self._get_bmses_cell_voltage_full() != None and self._get_bmses_cell_count() != None:
                self.max_charge_voltage = ((self._get_bmses_cell_voltage_full() - 0.04) * self._get_bmses_cell_count()) # A little under the full voltage to prevent the BMS from balancing all the time
            else:
                self.max_charge_voltage = 0
            charge_voltage_controller_ki = 0 # Reset controller
        
        #print('Charge voltage:\t{}'.format(self.max_charge_voltage))
        
        
class SmartBMSDbus(object):
    def __init__(self, monitor, service, device_instance):
        self.monitor = monitor
        self.service = service
        self.device_instance = device_instance
        self.last_seen = time.time()
        self.lowest_voltage = None
        self.highest_voltage = None
        self.capacity = None
        self.soc = None
        self.voltage = None
        self.current = None
        self.power = None
        self.cell_voltage_min = None
        self.cell_voltage_max = None
        self.cell_voltage_full = None
        self.cell_count = None
        self.battery_charge_state = None
        self.allowed_to_charge = False
        self.allowed_to_discharge = False
        self.communication_error = False

# Called on a one second timer
def handle_timer_tick():
    # The BMS data readout and variable writing happens on a different thread -> lock before
    bms_dbus.update()
    return True  # keep timer running

if __name__ == "__main__":  
    # Have a mainloop, so we can send/receive asynchronous calls to and from dbus
    print('123\\SmartBMS master to dbus started')
    DBusGMainLoop(set_as_default=True)

    mainloop = GLib.MainLoop()
    bms_dbus = SmartBMSToDbus(mainloop)

    time.sleep(3) # Wait until we have received some data

    GLib.timeout_add(1000, lambda: ve_utils.exit_on_error(handle_timer_tick))
    mainloop.run()
