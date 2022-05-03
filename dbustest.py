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
from dbusmonitor import DbusMonitor
from vedbus import VeDbusService

class SmartBMS:  
    def __init__(
        self,
        loop
    ):
        self.loop = loop

class SmartBMSDbus(SmartBMS):
    def __init__(self, loop):
        super().__init__(loop)
        
        
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
                '/ProductId': dummy,
                '/CustomName': dummy,
                '/System/MinCellVoltage': dummy,
                '/System/MaxCellVoltage': dummy},
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
    
    def update(self):
        print('Scanning...')
        batteries = self._get_connected_batteries()
        for bms in batteries:
            print('Found ' + bms)
        
    @property    
    def is_master(self):
        battery_list = self._get_connected_batteries()
        s = sorted((value, key) for (key, value) in battery_list.items())
        # The device with lowest instance is the master
        master_id = s[0][0]
        return self._device_instance == master_id
        
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
        print('Device Added') 
        if do_service_change:
            self._handleservicechange()

    def _device_removed(self, service, instance):
        print('Device removed')
        self._handleservicechange()
                
    def _get_connected_service_list(self, classfilter=None):
        services = self._dbusmonitor.get_service_list(classfilter=classfilter)
        return services
    
    def _get_connected_batteries(self):
        batteries = self._get_connected_service_list('com.victronenergy.battery').items()
        bms_list = []
        for battery in batteries:
            product_name = self._dbusmonitor.get_value(battery[0], '/ProductName')
            connected = self._dbusmonitor.get_value(battery[0], '/Connected')
            if product_name == '123SmartBMS' and connected == 1:
                bms_list.append(battery[0])
        
        return bms_list
        
    def _get_system_soc(self):
        soc = self._dbusmonitor.get_value('com.victronenergy.system', '/Dc/Battery/Soc')
        print(soc)
        return soc
        
    def _get_lowest_battery_cell_voltage(self):
        battery_list = self._get_connected_batteries()
        lowest_voltage = None
        for battery in battery_list:
            battery_lowest_voltage = self._dbusmonitor.get_value(battery, '/System/MinCellVoltage')
            if battery_lowest_voltage != None and (lowest_voltage == None or battery_lowest_voltage < lowest_voltage):
                lowest_voltage = battery_lowest_voltage
        return lowest_voltage
    
    def _get_highest_battery_cell_voltage(self):
        battery_list = self._get_connected_batteries()
        highest_voltage = None
        for battery in battery_list:
            battery_highest_voltage = self._dbusmonitor.get_value(battery, '/System/MaxCellVoltage')
            if battery_highest_voltage != None and (highest_voltage == None or battery_highest_voltage > highest_voltage):
                highest_voltage = battery_highest_voltage
        return highest_voltage

# Called on a one second timer
def handle_timer_tick():
    # The BMS data readout and variable writing happens on a different thread -> lock before
    bms_dbus.update()
    return True  # keep timer running

if __name__ == "__main__":  
    # Have a mainloop, so we can send/receive asynchronous calls to and from dbus
    print('123\\SmartBMS to dbus started')
    DBusGMainLoop(set_as_default=True)

    mainloop = GLib.MainLoop()
    bms_dbus = SmartBMSDbus(mainloop)
    GLib.timeout_add(1000, lambda: ve_utils.exit_on_error(handle_timer_tick))
    mainloop.run()
