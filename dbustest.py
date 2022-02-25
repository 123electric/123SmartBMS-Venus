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
    def __init__(self, loop, serial_id):
        super().__init__(loop)
        
        self._serial_id = serial_id
        
        self._info = {
            'name'      : "123SmartBMS",
            'servicename' : "smartbms",
            'id'          : 0,
            'version'    : 1.3
        }
        
        dummy = {'code': None, 'whenToLog': 'configChange', 'accessLevel': None}
		dbus_tree = {
			'com.victronenergy.vebus' : {
				'/Ac/ActiveIn/ActiveInput': dummy,
				'/Ac/ActiveIn/L1/P': dummy,
				'/Ac/ActiveIn/L2/P': dummy,
				'/Ac/ActiveIn/L3/P': dummy,
				'/Ac/Out/L1/P': dummy,
				'/Ac/Out/L2/P': dummy,
				'/Ac/Out/L3/P': dummy,
				'/Connected': dummy,
				'/ProductId': dummy,
				'/ProductName': dummy,
				'/Mgmt/Connection': dummy,
				'/Mode': dummy,
				'/State': dummy,
				'/Dc/0/Voltage': dummy,
				'/Dc/0/Current': dummy,
				'/Dc/0/Power': dummy,
				'/Soc': dummy}
		}


        device_port = 'ttyUSB4'
        device_port_num = 4
        self._dbusservice = VeDbusService("com.victronenergy.battery." + device_port)
        self._dbusmonitor = DbusMonitor(dbus_tree, valueChangedCallback=self._dbus_value_changed,
			deviceAddedCallback=self._device_added, deviceRemovedCallback=self._device_removed)
        
        # Create the management objects, as specified in the ccgx dbus-api document
        self._dbusservice.add_path('/Mgmt/ProcessName', __file__)
        self._dbusservice.add_path('/Mgmt/ProcessVersion', self._info['version'])
        self._dbusservice.add_path('/Mgmt/Connection', ' Serial ' + dev)

        # Create the basic objects
        self._dbusservice.add_path('/DeviceInstance', 288+int(device_port_num))
        self._dbusservice.add_path('/ProductId',     self._info['id'])
        self._dbusservice.add_path('/ProductName',     self._info['name'])
        self._dbusservice.add_path('/FirmwareVersion', self._info['version'], gettextcallback=lambda p, v: "v{:.2f}".format(v))
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
        self._dbusservice.add_path('/Soc',                                  None, gettextcallback=lambda p, v: "{:.0f}%%".format(v))
        self._dbusservice.add_path('/Capacity',                             None, gettextcallback=lambda p, v: "{:.1f}kWh".format(v))
        self._dbusservice.add_path('/InstalledCapacity',                    None, gettextcallback=lambda p, v: "{:.1f}kWh".format(v))
        self._dbusservice.add_path('/ConsumedAmphours',                     None, gettextcallback=lambda p, v: "{:.1f}Ah".format(v))
        self._dbusservice.add_path('/Dc/0/Voltage',                         None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Dc/0/Current',                         None, gettextcallback=lambda p, v: "{:.1f}A".format(v))
        self._dbusservice.add_path('/Dc/0/Power',                           None, gettextcallback=lambda p, v: "{:.0f}W".format(v))
        self._dbusservice.add_path('/Dc/0/Temperature',                     None)
        self._dbusservice.add_path('/Io/AllowToCharge',                     None)
        self._dbusservice.add_path('/Io/AllowToDischarge',                  None)
        self._dbusservice.add_path('/Info/UpdateTimestamp',                 None)
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
        self._dbusservice.add_path('/Info/BatteryLowVoltage',               None)
        self._dbusservice.add_path('/Info/MaxChargeVoltage',                None, gettextcallback=lambda p, v: "{:.2f}V".format(v))
        self._dbusservice.add_path('/Info/MaxChargeCurrent',                None, gettextcallback=lambda p, v: "{:.2f}A".format(v))
        self._dbusservice.add_path('/Info/MaxDischargeCurrent',             None, gettextcallback=lambda p, v: "{:.2f}A".format(v))
        self._dbusservice.add_path('/Alarms/LowVoltage',                    None)
        self._dbusservice.add_path('/Alarms/HighVoltage',                   None)
        self._dbusservice.add_path('/Alarms/LowTemperature',                None)
        self._dbusservice.add_path('/Alarms/HighTemperature',               None)
        
        self.charge_current_limit = 0
        self.max_charge_voltage = 0
        self.max_discharge_current = 0
        self.max_charge_current = 0
        
        self._balance_state = self.BALANCE_STATE_UNBALANCED
        self._battery_full_counter = 0
        self._balanced_timer = 0
        self._unbalance_detection_timer = 0
        self._charge_voltage_controller_integral = 0
    
    def update(self):
        super().update()
    
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
    
    def _get_active_bms(self):
        batteries = self._get_connected_service_list('com.victronenergy.battery')
        
        #print('Charge voltage:\t{}'.format(self.max_charge_voltage))

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
    bms_dbus = SmartBMSDbus(mainloop, '123')

    time.sleep(3) # Wait until we have received some data

    GLib.timeout_add(1000, lambda: ve_utils.exit_on_error(handle_timer_tick))
    mainloop.run()
