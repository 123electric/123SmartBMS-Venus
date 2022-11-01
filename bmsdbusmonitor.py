#!/usr/bin/env python3
import argparse
import os
from os import _exit as os_exit
from traceback import print_exc
import struct
import sys
import threading
import time
import logging
import copy
from datetime import datetime
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib

# Victron packages
sys.path.insert(1, os.path.join(os.path.dirname(__file__), '/opt/victronenergy/dbus-systemcalc-py/ext/velib_python'))
import dbus.service
import ve_utils
from dbusmonitor import DbusMonitor

class SmartBMSDbusMonitor:
    def __init__(self):
        # Shared data
        self._shared_dbus_smartbmses = []
        self._shared_system_soc = None

        # Lock for shared data
        self._data_lock = threading.Lock()
        self._monitor_thread = threading.Thread(target=lambda:self._monitor())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()

    def get_system_soc(self):
        with self._data_lock:
            system_soc_copy = self._shared_system_soc
        
        return system_soc_copy

    def get_smartbmses(self):
        with self._data_lock:
            # Get reference to last list of BMSes
            smartbmses_copy = copy.deepcopy(self._shared_dbus_smartbmses)
        
        # Give back copy of dbus values
        return smartbmses_copy
    
    def _monitor(self):
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

        self._dbusmonitor = DbusMonitor(dbus_tree, valueChangedCallback=self._dbus_value_changed,
            deviceAddedCallback=self._device_added, deviceRemovedCallback=self._device_removed)

        # It is important to not reda the values from the dbus when data_lock is active.
        # Good practise: read data from dbus async, then lock and 
        while(1):
            system_soc = self._dbusmonitor.get_value('com.victronenergy.system', '/Dc/Battery/Soc')
            with self._data_lock:
                self._shared_system_soc = system_soc

            
            dbus_smartbmses = []
            # Lock data before reading/writing because this is a separate thread
            batteries = self._get_connected_service_list('com.victronenergy.battery').items()
            for battery in batteries:
                product_name = self._dbusmonitor.get_value(battery[0], '/ProductName')
                connected = self._dbusmonitor.get_value(battery[0], '/Connected')
                updated_timestamp = self._dbusmonitor.get_value(battery[0], '/UpdateTimestamp')
                if product_name == '123SmartBMS' and connected == 1 and updated_timestamp != None:
                    service = battery[0]
                    device_instance = battery[1]
                    bms = SmartBMSDbus(service, device_instance)
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
                    # Add BMS data to SmartBMS Dbus list
                    dbus_smartbmses.append(bms)
            
            # Update dbus_smartbmses list with latest found
            with self._data_lock:
                self._shared_dbus_smartbmses = dbus_smartbmses
            time.sleep(0.2)

    def _get_connected_service_list(self, classfilter=None):
        services = self._dbusmonitor.get_service_list(classfilter=classfilter)
        return services

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

def exit_on_error(func, *args, **kwargs):
	try:
		return func(*args, **kwargs)
	except:
		try:
			logging.critical('exit_on_error: there was an exception. Printing stacktrace will be tried and then exit', exc_info=True)
		except:
			pass

		# sys.exit() is not used, since that throws an exception, which does not lead to a program
		# halt when used in a dbus callback, see connection.py in the Python/Dbus libraries, line 230.
		os_exit(1)

class SmartBMSDbus(object):
    def __init__(self, service, device_instance):
        self.service = service
        self.device_instance = device_instance
        self.last_seen = 0
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