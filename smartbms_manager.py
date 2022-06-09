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
            'name'      : "123SmartBMS Manager",
            'servicename' : "123SmartBMSManager",
            'id'          : 0,
            'version'    : 1.03
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
                '/CustomName': dummy,
                '/InstalledCapacity': dummy,
                '/ProductId': dummy,
                '/UpdateTimestamp': dummy,
                '/System/MinCellVoltage': dummy,
                '/System/MinVoltageCellId': dummy,
                '/System/MaxCellVoltage': dummy,
                '/System/MaxVoltageCellId': dummy,
                '/System/MinCellTemperature': dummy,
                '/System/MinTemperatureCellId': dummy,
                '/System/MaxCellTemperature': dummy,
                '/System/MaxTemperatureCellId': dummy,
                '/System/NrOfModulesBlockingCharge': dummy,
                '/System/NrOfModulesBlockingDischarge': dummy,
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

        self._dbusservice = VeDbusService("com.victronenergy.battery.smartBMSManager")
        
        # Create the management objects, as specified in the ccgx dbus-api document
        self._dbusservice.add_path('/Mgmt/ProcessName', __file__)
        self._dbusservice.add_path('/Mgmt/ProcessVersion', self._info['version'])
        self._dbusservice.add_path('/Mgmt/Connection', 'Internal')

        # Create the basic objects
        self._dbusservice.add_path('/DeviceInstance', self._device_instance)
        self._dbusservice.add_path('/ProductId',     self._info['id'])
        self._dbusservice.add_path('/ProductName',     self._info['name'])
        self._dbusservice.add_path('/FirmwareVersion', self._info['version'], gettextcallback=lambda p, v: "v{:.2f}".format(v))
        self._dbusservice.add_path('/HardwareVersion', None)
        self._dbusservice.add_path('/Serial', '')
        self._dbusservice.add_path('/Connected',     1)
        self._dbusservice.add_path('/CustomName', self._info['name'])

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
        self._dbusservice.add_path('/Capacity',                             None, gettextcallback=lambda p, v: "{:.1f}Ah".format(v))
        self._dbusservice.add_path('/InstalledCapacity',                    None, gettextcallback=lambda p, v: "{:.1f}Ah".format(v))
        self._dbusservice.add_path('/ConsumedAmphours',                     None, gettextcallback=lambda p, v: "{:.1f}Ah".format(v))
        self._dbusservice.add_path('/Dc/0/Voltage',                         None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Dc/0/Current',                         None, gettextcallback=lambda p, v: "{:.1f}A".format(v))
        self._dbusservice.add_path('/Dc/0/Power',                           None, gettextcallback=lambda p, v: "{:.0f}W".format(v))
        self._dbusservice.add_path('/Dc/0/Temperature',                     None)
        #self._dbusservice.add_path('/Io/AllowToCharge',                     None)
        #self._dbusservice.add_path('/Io/AllowToDischarge',                  None)
        #self._dbusservice.add_path('/Info/UpdateTimestamp',                 None)
        #self._dbusservice.add_path('/Voltages/Cell1',                      None)
        #self._dbusservice.add_path('/Voltages/Cell2',                      None)
        self._dbusservice.add_path('/System/MinCellVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/System/MinVoltageCellId',              None)
        self._dbusservice.add_path('/System/MaxCellVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
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
        self._dbusservice.add_path('/Info/BatteryLowVoltage',               None)
        self._dbusservice.add_path('/Info/MaxChargeVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Info/MaxChargeCurrent',                None, gettextcallback=lambda p, v: "{:.2f}A".format(v))
        self._dbusservice.add_path('/Info/MaxDischargeCurrent',             None, gettextcallback=lambda p, v: "{:.2f}A".format(v))
        
        # Cache of all connected SmartBMSes
        self._connected_smartbmses = []
        self._managed_smartbmses = []

        self._charge_voltage_controller_integral = 0
        self.charge_current_limit = 0
        self.max_charge_voltage = 0
        self.max_discharge_current = 0
        self.max_charge_current = 0

        # Setup timers
        self.timer_1000ms = 0
    
    def update(self):
        self._scan_connected_smartbmses()
        self._remove_disconnected_bmses()
        self._determine_managed_smartbmses()
        bms_having_lowest_voltage = self._get_bmses_having_lowest_voltage()
        bms_having_highest_voltage = self._get_bmses_having_highest_voltage()
        bms_having_lowest_temperature = self._get_bmses_having_lowest_temperature()
        bms_having_highest_temperature = self._get_bmses_having_highest_temperature()

        if len(self._managed_smartbmses) == 0 \
        or bms_having_lowest_voltage == None \
        or bms_having_highest_voltage == None \
        or bms_having_lowest_temperature == None \
        or bms_having_highest_temperature == None:
            self._dbusservice["/Soc"] = None
            #self._dbusservice["/SystemSwitch"] = None
            self._dbusservice["/ConsumedAmphours"] = None
            self._dbusservice["/Capacity"] = None
            self._dbusservice["/InstalledCapacity"] = None
            self._dbusservice["/TimeToGo"] = None
            self._dbusservice["/Dc/0/Voltage"] = None
            self._dbusservice["/Dc/0/Current"] = None
            self._dbusservice["/Dc/0/Power"] = None
            self._dbusservice["/Dc/0/Temperature"] = None
            #self._dbusservice["/Io/AllowToCharge"] = None
            #self._dbusservice["/Io/AllowToDischarge"] = None
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
            #self._dbusservice["/Alarms/LowVoltage"] = None
            #self._dbusservice["/Alarms/HighVoltage"] = None
            #self._dbusservice["/Alarms/LowTemperature"] = None
            #self._dbusservice["/Alarms/HighTemperature"] = None
        else:
            stored_ah = self._get_bmses_stored_ah()
            installed_capacity = self._get_bmses_installed_capacity()
            self._dbusservice["/Soc"] = self._get_bmses_soc()
            #self._dbusservice["/SystemSwitch"] = 1
            self._dbusservice["/ConsumedAmphours"] = round(-1*(installed_capacity-stored_ah),1)+0 # Add zero to remove negative sigh from -0.0
            self._dbusservice["/Capacity"] = self._get_bmses_stored_ah()
            self._dbusservice["/InstalledCapacity"] = self._get_bmses_installed_capacity()
            self._dbusservice["/TimeToGo"] = self._get_bmses_average_time_to_go()
            self._dbusservice["/Dc/0/Voltage"] = self._get_bmses_average_voltage()
            self._dbusservice["/Dc/0/Current"] = self._get_bmses_sum_current()
            self._dbusservice["/Dc/0/Power"] = self._get_bmses_sum_power()
            self._dbusservice["/Dc/0/Temperature"] = bms_having_highest_temperature.highest_temperature
            #self._dbusservice["/Io/AllowToCharge"] = int(self.allowed_to_charge)
            #self._dbusservice["/Io/AllowToDischarge"] = int(self.allowed_to_discharge)
            self._dbusservice["/System/MinCellVoltage"] = bms_having_lowest_voltage.lowest_voltage
            self._dbusservice["/System/MinVoltageCellId"] = '[' + bms_having_lowest_voltage.custom_name[:12] + '] ' + str(bms_having_lowest_voltage.lowest_voltage_num)
            self._dbusservice["/System/MaxCellVoltage"] = bms_having_highest_voltage.highest_voltage
            self._dbusservice["/System/MaxVoltageCellId"] =  '[' + bms_having_highest_voltage.custom_name[:12] + '] ' + str(bms_having_highest_voltage.highest_voltage_num)
            self._dbusservice["/System/MinCellTemperature"] = bms_having_lowest_temperature.lowest_temperature
            self._dbusservice["/System/MinTemperatureCellId"] = '[' + bms_having_lowest_temperature.custom_name[:12] + '] ' + str(bms_having_lowest_temperature.lowest_temperature_num)
            self._dbusservice["/System/MaxCellTemperature"] = bms_having_highest_temperature.highest_temperature
            self._dbusservice["/System/MaxTemperatureCellId"] = '[' + bms_having_highest_temperature.custom_name[:12] + '] ' + str(bms_having_lowest_temperature.highest_temperature_num)
            self._dbusservice["/System/NrOfModulesOnline"] = len(self._managed_smartbmses)
            self._dbusservice["/System/NrOfModulesOffline"] = 0
            self._dbusservice["/System/NrOfModulesBlockingCharge"] = self._get_bmses_nr_of_banks_blocking_charge()
            self._dbusservice["/System/NrOfModulesBlockingDischarge"] = self._get_bmses_nr_of_banks_blocking_discharge()
            #self._dbusservice["/Alarms/LowVoltage"] = 0
            #self._dbusservice["/Alarms/HighVoltage"] = 0
            #self._dbusservice["/Alarms/LowTemperature"] = int(self.alarm_minimum_temperature)
            #self._dbusservice["/Alarms/HighTemperature"] = int(self.alarm_maximum_temperature)

        cell_voltage_min = self._get_bmses_cell_voltage_min()
        cell_count = self._get_bmses_cell_count()
        if cell_voltage_min != None and cell_count != None:
            self._dbusservice["/Info/BatteryLowVoltage"] = (cell_voltage_min+0.05) * cell_count
        else:
            self._dbusservice["/Info/BatteryLowVoltage"] = None

        self.timer_1000ms = 0 if self.timer_1000ms == 4 else self.timer_1000ms+1
        # Make sure updating only happens once a second
        if self.timer_1000ms == 0:
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
        for battery in batteries:
            product_name = self._dbusmonitor.get_value(battery[0], '/ProductName')
            connected = self._dbusmonitor.get_value(battery[0], '/Connected')
            updated_timestamp = self._dbusmonitor.get_value(battery[0], '/UpdateTimestamp')
            if product_name == '123SmartBMS' and connected == 1 and updated_timestamp != None:
                service = battery[0]
                device_instance = battery[1]
                matching_bms = list(filter(lambda b: b.service == service, self._connected_smartbmses))
                in_list = len(matching_bms)
                if in_list == 0:
                    # New: create new instance
                    found_bms = SmartBMSDbus(self._dbusmonitor, service, device_instance)
                    self._connected_smartbmses.append(found_bms)
                    self._update_bms_data(found_bms)
                else:
                    self._update_bms_data(matching_bms[0])

    def _remove_disconnected_bmses(self):
        for bms in self._connected_smartbmses:
            if time.time() > bms.last_seen + self.LAST_SEEN_TIMEOUT:
                self._connected_smartbmses.remove(bms)

    def _determine_managed_smartbmses(self):
        # Sort all connected SmartBMS'es based on cell count. All managed BMSes need to have the same cell count. Smaller batteries will not be managed
        # Match is based on biggest cell count
        bmses = self._connected_smartbmses
        # First filter out all BMSes which start with * symbol. This is our magical symbol to indicate we do not want this BMS to be managed
        bmses_filter1 = list(filter(lambda b: b.custom_name[:1] != '*', bmses))
        bmses_filter1_sorted = sorted(bmses_filter1, key=lambda b: b.cell_count, reverse=True)
        self._managed_smartbmses = list(filter(lambda b: b.cell_count == bmses_filter1_sorted[0].cell_count, bmses_filter1_sorted))

    def _update_bms_data(self, bms):
        bms.last_seen = time.time()
        bms.lowest_voltage = self._dbusmonitor.get_value(bms.service, '/System/MinCellVoltage')
        bms.lowest_voltage_num = self._dbusmonitor.get_value(bms.service, '/System/MinVoltageCellId')
        bms.highest_voltage = self._dbusmonitor.get_value(bms.service, '/System/MaxCellVoltage')
        bms.highest_voltage_num = self._dbusmonitor.get_value(bms.service, '/System/MaxVoltageCellId')
        bms.lowest_temperature = self._dbusmonitor.get_value(bms.service, '/System/MinCellTemperature')
        bms.lowest_temperature_num = self._dbusmonitor.get_value(bms.service, '/System/MinTemperatureCellId')
        bms.highest_temperature = self._dbusmonitor.get_value(bms.service, '/System/MaxCellTemperature')
        bms.highest_temperature_num = self._dbusmonitor.get_value(bms.service, '/System/MaxTemperatureCellId')
        bms.stored_ah = self._dbusmonitor.get_value(bms.service, '/Capacity')
        bms.installed_capacity = self._dbusmonitor.get_value(bms.service, '/InstalledCapacity')
        bms.battery_charge_state = self._dbusmonitor.get_value(bms.service, '/System/BatteryChargeState')
        bms.allowed_to_charge = self._dbusmonitor.get_value(bms.service, '/System/NrOfModulesBlockingCharge') == 0
        bms.allowed_to_discharge = self._dbusmonitor.get_value(bms.service, '/System/NrOfModulesBlockingDischarge') == 0
        bms.soc = self._dbusmonitor.get_value(bms.service, '/Soc')
        bms.voltage = self._dbusmonitor.get_value(bms.service, '/Dc/0/Voltage')
        bms.current = self._dbusmonitor.get_value(bms.service, '/Dc/0/Current')
        bms.power = self._dbusmonitor.get_value(bms.service, '/Dc/0/Power')
        bms.cell_voltage_min = self._dbusmonitor.get_value(bms.service, '/System/LowVoltageThreshold')
        bms.cell_voltage_max = self._dbusmonitor.get_value(bms.service, '/System/HighVoltageThreshold')
        bms.cell_voltage_full = self._dbusmonitor.get_value(bms.service, '/System/FullVoltageThreshold')
        bms.cell_count = self._dbusmonitor.get_value(bms.service, '/System/NrOfCells')
        bms.time_to_go = self._dbusmonitor.get_value(bms.service, '/TimeToGo')
        bms.custom_name = self._dbusmonitor.get_value(bms.service, '/CustomName')
        bms.communication_error = True if bms.soc == None else False
        
        
    def _get_system_soc(self):
        soc = self._dbusmonitor.get_value('com.victronenergy.system', '/Dc/Battery/Soc')
        return soc
        
    def _get_bmses_having_lowest_voltage(self):
        bms_found = None
        lowest_voltage = None
        for bms in self._managed_smartbmses:
            bms_lowest_voltage = bms.lowest_voltage
            if bms_lowest_voltage != None and (lowest_voltage == None or bms_lowest_voltage < lowest_voltage):
                bms_found = bms
                lowest_voltage = bms_lowest_voltage
        return bms_found
    
    def _get_bmses_having_highest_voltage(self):
        bms_found = None
        highest_voltage = None
        for bms in self._managed_smartbmses:
            bms_highest_voltage = bms.highest_voltage
            if bms_highest_voltage != None and (highest_voltage == None or bms_highest_voltage > highest_voltage):
                bms_found = bms
                highest_voltage = bms_highest_voltage
        return bms_found

    def _get_bmses_having_lowest_temperature(self):
        bms_found = None
        lowest_temperature = None
        for bms in self._managed_smartbmses:
            if bms.lowest_temperature != None and (lowest_temperature == None or bms.lowest_temperature < lowest_temperature):
                bms_found = bms
                lowest_temperature = bms.lowest_temperature
        return bms_found

    def _get_bmses_having_highest_temperature(self):
        bms_found = None
        highest_temperature = None
        for bms in self._managed_smartbmses:
            if bms.highest_temperature != None and (highest_temperature == None or bms.highest_temperature > highest_temperature):
               bms_found = bms
               highest_temperature = bms.highest_temperature
        return bms_found

    def _get_bmses_installed_capacity(self):
        installed_capacity = 0
        for bms in self._managed_smartbmses:
            if bms.installed_capacity != None:
                installed_capacity += bms.installed_capacity
    
        return installed_capacity

    def _get_bmses_stored_ah(self):
        stored_ah = 0
        for bms in self._managed_smartbmses:
            if bms.stored_ah != None:
                stored_ah += bms.stored_ah
    
        return stored_ah

    def _get_bmses_soc(self):
        soc_sum = 0
        bms_stored_ah_sum = 0
        bms_installed_capacity_sum = 0
        all_bmses_full = True
        if len(self._managed_smartbmses) == 1: return self._managed_smartbmses[0].soc
        for bms in self._managed_smartbmses:
            if bms.installed_capacity != None:
                bms_stored_ah_sum += bms.stored_ah
                bms_installed_capacity_sum += bms.installed_capacity
                if bms.soc < 100: all_bmses_full = False
        if bms_installed_capacity_sum == 0: return 0
        calculated_soc = round(bms_stored_ah_sum * 100 / bms_installed_capacity_sum, 0)
        # If calculated SoC is 100 and one or more BMS still indicate 99 because not fully balanced: keep it at 99%
        if calculated_soc == 100 and (not all_bmses_full): calculated_soc = 99
        return calculated_soc

    def _get_bmses_average_voltage(self):
        voltage_sum = 0
        bms_count = 0
        for bms in self._managed_smartbmses:
            if bms.voltage != None:
                voltage_sum += bms.voltage
                bms_count += 1
        if bms_count == 0: return 0
        return voltage_sum / bms_count

    def _get_bmses_average_time_to_go(self):
        ttg_sum = 0
        bms_count = 0
        for bms in self._managed_smartbmses:
            if bms.time_to_go != None:
                ttg_sum += bms.time_to_go
                bms_count += 1
        if bms_count == 0: return None
        return ttg_sum / bms_count

    def _get_bmses_sum_power(self):
        sum = 0
        for bms in self._managed_smartbmses:
            if bms.power != None:
                sum += bms.power
        return sum

    def _get_bmses_sum_current(self):
        sum = 0
        for bms in self._managed_smartbmses:
            if bms.current != None:
                sum += bms.current
        return sum

    def _get_bmses_sum_communication_error(self):
        sum = 0
        for bms in self._managed_smartbmses:
            if bms.communication_error == True:
                sum += 1
        return sum

    def _get_bmses_cell_voltage_min(self):
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_min != None:
                return bms.cell_voltage_min
    
    def _get_bmses_cell_voltage_max(self):
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_max != None:
                return bms.cell_voltage_max
        return None
    
    def _get_bmses_cell_voltage_full(self):
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_full != None:
                return bms.cell_voltage_full
        return None

    def _get_bmses_cell_voltage_max(self):
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_voltage_max != None:
                return bms.cell_voltage_max
        return None

    def _get_bmses_cell_count(self):
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.cell_count != None:
                return bms.cell_count
        return None

    def _get_bmses_nr_of_banks_blocking_charge(self):
        blocking = 0
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.allowed_to_charge != None and (not bms.allowed_to_charge):
                blocking += 1
        return blocking

    def _get_bmses_nr_of_banks_blocking_discharge(self):
        blocking = 0
        for bms in self._managed_smartbmses:
            # Take value from first BMS with valid value
            if bms.allowed_to_discharge != None and (not bms.allowed_to_discharge):
                blocking += 1
        return blocking
        
    def _calculate_current_limits(self):
        lowest_cell_voltage_bms = self._get_bmses_having_lowest_voltage()
        highest_cell_voltage_bms = self._get_bmses_having_highest_voltage()
        cell_voltage_min_bms = self._get_bmses_cell_voltage_min()
        cell_voltage_max_bms = self._get_bmses_cell_voltage_max()
        cell_voltage_full_bms = self._get_bmses_cell_voltage_full()
        cell_count = self._get_bmses_cell_count()
        system_soc = self._dbusmonitor.get_value('com.victronenergy.system', '/Dc/Battery/Soc')
        soc = system_soc if system_soc != None else self._get_bmses_soc()
        system_lowest_cell_voltage = lowest_cell_voltage_bms.lowest_voltage
        system_highest_cell_voltage = highest_cell_voltage_bms.highest_voltage

        # One or more BMS have an error, or no BMS found? Set limits to zero
        if self._get_bmses_sum_communication_error() > 0 or lowest_cell_voltage_bms == None or highest_cell_voltage_bms == None \
            or system_highest_cell_voltage == None or cell_voltage_min_bms == None or cell_voltage_max_bms == None \
            or cell_voltage_full_bms == None or cell_count == None or soc == None:
            self.max_discharge_current = 0
            self.max_charge_current = 0
            self.max_charge_voltage = None
            return
        
        # Calculate total, connected capacity for charging
        charge_capacity_sum = 0
        discharge_capacity_sum = 0
        for bms in self._managed_smartbmses:
            capacity = bms.installed_capacity
            if capacity == None: capacity = 0
            if bms.allowed_to_charge: charge_capacity_sum += capacity
            if bms.allowed_to_discharge: discharge_capacity_sum += capacity
        
        # Discharge - use SoC for discharge current
        discharge_limit = discharge_capacity_sum*self.BATTERY_DISCHARGE_MAX_RATING
        
        if system_lowest_cell_voltage - 0.05 <= cell_voltage_min_bms:
            self.max_discharge_current = 0
        elif soc <= 10:
            self.max_discharge_current = round(discharge_limit/8, 1)
        elif 10 < soc <= 20:
            self.max_discharge_current = round(discharge_limit/4, 1)
        elif 20 < soc <= 25:
            self.max_discharge_current = round(discharge_limit/2, 1)
        elif 25 < soc <= 30:
            self.max_discharge_current = round(discharge_limit/1.5, 1)
        else:
            self.max_discharge_current = round(discharge_limit, 1)
        
        # Charge
        # Fixed charge current - when pack is full, regulate the voltage
        if system_highest_cell_voltage+0.05 >= cell_voltage_max_bms: # Pre-critical cutoff: should never happen. Avoid BMS triggering power cutoff
            self.max_charge_current = 0
        else:
            self.max_charge_current = charge_capacity_sum*self.BATTERY_CHARGE_MAX_RATING
        
        bmses_in_bulkabsorption = list(filter(lambda b: b.battery_charge_state == self.BATTERY_CHARGE_STATE_BULKABSORPTION, self._managed_smartbmses))
        # When not all BMSes are in storage state (balanced): keep CVL higher so battery can fully charge
        if len(bmses_in_bulkabsorption) > 0:
             # Really need 25mV per cell headroom because otherwise we won't get all cells at Vbalance for at least 30 seconds
             # as the Victron can swing a little with the charge current, so sometimes a cell gets a little lower - in this case for example to 3.50V
            highest_cell_voltage_target = round(cell_voltage_full_bms + 0.025, 3)
            voltage_upper_limit = (cell_voltage_full_bms + 0.02) * cell_count
            # If code just started, set value to default
            if(self.max_charge_voltage == 0): self.max_charge_voltage = voltage_upper_limit
            
            #print('Highest cell voltage target:\t{}'.format(highest_cell_voltage_target))
            #print('Highest cell voltage:\t\t{}'.format(self.highest_cell_voltage))
            # When battery is ~unbalanced, charge to a point where all cells are balancing.
            # When the battery is balanced, charge to a voltage a little lower so no balancing energy is wasted
            charge_voltage_controller_kp = 0.1
            charge_voltage_controller_ki = 0.01
            charge_voltage_controller_setpoint = highest_cell_voltage_target
            charge_voltage_controller_input = system_highest_cell_voltage
            charge_voltage_controller_error = charge_voltage_controller_setpoint - charge_voltage_controller_input # Value when over full threshold - example: -0.1 (100mV over threshold)
            # Add some blindspot for error so we do not keep changing when within certain bounds
            if charge_voltage_controller_error <= -0.01:
                self._charge_voltage_controller_integral += charge_voltage_controller_error
            elif charge_voltage_controller_error > 0: # positive error, slowly rise
                self._charge_voltage_controller_integral += (0.0004/charge_voltage_controller_ki)*charge_voltage_controller_error
            # limit integral, only lower voltage when needed
            if self._charge_voltage_controller_integral*charge_voltage_controller_ki > 0.1: self._charge_voltage_controller_integral = 0.1/charge_voltage_controller_ki
            if self._charge_voltage_controller_integral*charge_voltage_controller_ki < -0.1: self._charge_voltage_controller_integral = -0.1/charge_voltage_controller_ki
            #print('Controller error:\t\t{}'.format(charge_voltage_controller_error))
            #print('Controller integral:\t\t{}'.format(self._charge_voltage_controller_integral))
            charge_voltage_controller_output = round(charge_voltage_controller_kp * charge_voltage_controller_error + charge_voltage_controller_ki * self._charge_voltage_controller_integral, 3)
            #print('Controller ouput:\t\t{}'.format(charge_voltage_controller_output))
            charge_voltage = round((highest_cell_voltage_target + charge_voltage_controller_output) * cell_count, 2)
            self.max_charge_voltage = min(charge_voltage, voltage_upper_limit)
        else:
            self.max_charge_voltage = ((cell_voltage_full_bms - 0.04) * cell_count) # A little under the full voltage to prevent the BMS from balancing all the time
            charge_voltage_controller_ki = 0 # Reset controller
        
        #print('Charge voltage:\t{}'.format(self.max_charge_voltage))
        
        
class SmartBMSDbus(object):
    def __init__(self, monitor, service, device_instance):
        self.monitor = monitor
        self.service = service
        self.device_instance = device_instance
        self.last_seen = time.time()
        self.lowest_voltage = None
        self.lowest_voltage_num = None
        self.highest_voltage = None
        self.highest_voltage_num = None
        self.lowest_temperature = None
        self.lowest_temperature_num = None
        self.highest_temperature = None
        self.highest_temperature_num = None
        self.stored_ah = None
        self.installed_capacity = None
        self.soc = None
        self.voltage = None
        self.current = None
        self.power = None
        self.cell_voltage_min = None
        self.cell_voltage_max = None
        self.cell_voltage_full = None
        self.cell_count = None
        self.time_to_go = None
        self.custom_name = None
        self.battery_charge_state = None
        self.allowed_to_charge = False
        self.allowed_to_discharge = False
        self.communication_error = False

# Called on a one second timer
def handle_timer_tick():
    bms_dbus.update()
    return True  # keep timer running

if __name__ == "__main__":  
    # Have a mainloop, so we can send/receive asynchronous calls to and from dbus
    print('123\\SmartBMS Manager to dbus started')
    DBusGMainLoop(set_as_default=True)

    mainloop = GLib.MainLoop()
    bms_dbus = SmartBMSToDbus(mainloop)

    time.sleep(3) # Wait until we have received some data

    GLib.timeout_add(200, lambda: ve_utils.exit_on_error(handle_timer_tick))
    mainloop.run()
