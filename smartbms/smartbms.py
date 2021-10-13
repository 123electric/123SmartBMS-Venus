#!/usr/bin/env python3
import time
import struct
import serial
from dbus.mainloop.glib import DBusGMainLoop
from gi.repository import GLib
from datetime import datetime
import sys
import os
import threading
import argparse
# Victron packages
sys.path.insert(1, os.path.join(os.path.dirname(__file__), '/opt/victronenergy/dbus-systemcalc-py/ext/velib_python'))
from vedbus import VeDbusService

BMS_COMM_TIMEOUT = 10 # Seconds
BMS_COMM_BLOCK_SIZE = 58

class SmartBMS:
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
		if(time.time() > self.last_received + BMS_COMM_TIMEOUT):
			return True
		else:
			return False

	def _poll(self, dev, test_packet = ''):
	# The SmartBMS transmit each 500ms or 1000ms a message containing 58 bytes
	# When the serial does not contain any new bytes and no complete message was received, empty the buffer and wait for a new message
	#logging.debug("Parse Packet [" + str(len(packet)) + "] bytes")
		buffer = bytearray (BMS_COMM_BLOCK_SIZE)
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
					if(buffer_index <= BMS_COMM_BLOCK_SIZE-1):
						buffer[buffer_index] = c
					buffer_index += 1
	
			if(buffer_index == BMS_COMM_BLOCK_SIZE):
					checksum = 0
					for i in range (BMS_COMM_BLOCK_SIZE-1):
						checksum += buffer[i]
					received_checksum = buffer[BMS_COMM_BLOCK_SIZE-1]
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
						self.alarm_minimum_voltage = True if (buffer[30] & 0b00001000) else False
						self.alarm_maximum_voltage = True if (buffer[30] & 0b00010000) else False
						self.alarm_minimum_temperature = True if (buffer[30] & 0b00100000) else False
						self.alarm_maximum_temperature = True if (buffer[30] & 0b01000000) else False
						self.alarm_cell_communication = True if (buffer[30] & 0b00000100) else False
						self.allowed_to_discharge = True if (buffer[30] & 0b00000010) else False
						self.allowed_to_charge = True if (buffer[30] & 0b00000001) else False
						self.last_received = time.time()
						print('Message received')
						self.lock.release()
			
					buffer_index = 0
			elif(len(read_data) == 0):
				buffer_index = 0

			time.sleep(0.2)

	def _decode_current(self, raw_value):
		if(raw_value[0] == 'X'):
			return 0
		elif(raw_value[0] == '-'):
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

# connect and register to dbus
driver = {
	'name'		: "123SmartBMS",
	'servicename' : "smartbms",
	'instance'	: 1,
	'id'		  : 0x01,
	'version'	 : 0.2
}

parser = argparse.ArgumentParser(description = '123SmartBMS driver')
requiredArguments = parser.add_argument_group('required arguments')
requiredArguments.add_argument('-d', '--device', help='serial device for data (eg /dev/ttyUSB0)', required=True)
args = parser.parse_args()

# Have a mainloop, so we can send/receive asynchronous calls to and from dbus
DBusGMainLoop(set_as_default=True)

device_port = args.device[args.device.rfind('/') + 1:]
dbusservice = VeDbusService("com.victronenergy.battery." + device_port)

# Create the management objects, as specified in the ccgx dbus-api document
dbusservice.add_path('/Mgmt/ProcessName', __file__)
dbusservice.add_path('/Mgmt/ProcessVersion', driver['version'])
dbusservice.add_path('/Mgmt/Connection', ' Serial ' + args.device)

# Create the mandatory objects
dbusservice.add_path('/DeviceInstance',  driver['instance'])
dbusservice.add_path('/ProductId',	   driver['id'])
dbusservice.add_path('/ProductName',	 driver['name'])
dbusservice.add_path('/FirmwareVersion', driver['version'])
dbusservice.add_path('/HardwareVersion', driver['version'])
dbusservice.add_path('/Serial',		  'SmartBMS' + device_port)
dbusservice.add_path('/Connected',	   1)

# Create device list
dbusservice.add_path('/Devices/0/DeviceInstance',  driver['instance'])
dbusservice.add_path('/Devices/0/FirmwareVersion', driver['version'])
dbusservice.add_path('/Devices/0/ProductId',	   driver['id'])
dbusservice.add_path('/Devices/0/ProductName',	 driver['name'])
dbusservice.add_path('/Devices/0/ServiceName',	 driver['servicename'])
dbusservice.add_path('/Devices/0/VregLink',		"(API)")

# Create the bms paths
dbusservice.add_path('/TimeToGo',						-1)
dbusservice.add_path('/SystemSwitch',			0)
dbusservice.add_path('/Dc/0/Temperature',				-1)
dbusservice.add_path('/Io/AllowToCharge',				-1)
dbusservice.add_path('/Io/AllowToDischarge',			-1)
dbusservice.add_path('/Info/UpdateTimestamp',			-1)
#dbusservice.add_path('/Voltages/Cell1',				-1)
#dbusservice.add_path('/Voltages/Cell2',				-1)
dbusservice.add_path('/System/MinVoltageCellId',		-1)
dbusservice.add_path('/System/MaxVoltageCellId',		-1)
dbusservice.add_path('/System/MinCellTemperature',		-1)
dbusservice.add_path('/System/MinTemperatureCellId',	-1)
dbusservice.add_path('/System/MaxCellTemperature',		-1)
dbusservice.add_path('/System/MaxTemperatureCellId',	-1)
dbusservice.add_path('/System/NrOfModulesOnline',		0)
dbusservice.add_path('/System/NrOfModulesOffline',		0)
dbusservice.add_path('/System/NrOfModulesBlockingCharge',		0)
dbusservice.add_path('/System/NrOfModulesBlockingDischarge',	0)
dbusservice.add_path('/Alarms/LowVoltage',				0)
dbusservice.add_path('/Alarms/HighVoltage',				0)
dbusservice.add_path('/Alarms/LowTemperature',			0)
dbusservice.add_path('/Alarms/HighTemperature',			0)

def gettext(path, value):
	item = summeditems.get(path)
	if item is not None:
		return item['gettext'] % value
	return str(value)

summeditems = {
						'/System/MaxCellVoltage': {'gettext': '%.2FV'},
						'/System/MinCellVoltage': {'gettext': '%.2FV'},
						'/Dc/0/Voltage': {'gettext': '%.2FV'},
						'/Dc/0/Current': {'gettext': '%.1FA'},
						'/Dc/0/Power': {'gettext': '%.0FW'},
						'/Soc': {'gettext': '%.0F%%'}
		}
for path in summeditems.keys():
				dbusservice.add_path(path, value=None, gettextcallback=gettext)

def update_values():
	# The BMS data readout and variable writing happens on a different thread -> lock before
	BMS1.lock.acquire()
	print('Pack voltage: {}'.format(BMS1.pack_voltage))
	if(BMS1.alarm_cell_communication or BMS1.alarm_serial_communication):
		dbusservice["/Soc"] = None
		dbusservice["/SystemSwitch"] = None
		dbusservice["/Dc/0/Voltage"] = None
		dbusservice["/Dc/0/Current"] =None
		dbusservice["/Dc/0/Power"] = None
		dbusservice["/Dc/0/Temperature"] = None
		dbusservice["/Io/AllowToCharge"] = None
		dbusservice["/Io/AllowToDischarge"] = None
		dbusservice["/System/MinCellVoltage"] = None
		dbusservice["/System/MinVoltageCellId"] = None
		dbusservice["/System/MaxCellVoltage"] = None
		dbusservice["/System/MaxVoltageCellId"] = None
		dbusservice["/System/MinCellTemperature"] = None
		dbusservice["/System/MinTemperatureCellId"] = None
		dbusservice["/System/MaxCellTemperature"] = None
		dbusservice["/System/MaxTemperatureCellId"] = None
		dbusservice["/System/NrOfModulesOnline"] = 0
		dbusservice["/System/NrOfModulesOffline"] = 1
		dbusservice["/System/NrOfModulesBlockingCharge"] = None
		dbusservice["/System/NrOfModulesBlockingDischarge"] = None
		dbusservice["/Alarms/LowVoltage"] = None
		dbusservice["/Alarms/HighVoltage"] = None
		dbusservice["/Alarms/LowTemperature"] = None
		dbusservice["/Alarms/HighTemperature"] = None
	else:
		dbusservice["/Soc"] = BMS1.soc
		dbusservice["/SystemSwitch"] = 1
		dbusservice["/Dc/0/Voltage"] = BMS1.pack_voltage
		dbusservice["/Dc/0/Current"] = BMS1.pack_current
		dbusservice["/Dc/0/Power"] = BMS1.pack_voltage * BMS1.pack_current
		dbusservice["/Dc/0/Temperature"] = BMS1.highest_cell_temperature
		dbusservice["/Io/AllowToCharge"] = int(BMS1.allowed_to_charge)
		dbusservice["/Io/AllowToDischarge"] = int(BMS1.allowed_to_discharge)
		dbusservice["/System/MinCellVoltage"] = BMS1.lowest_cell_voltage
		dbusservice["/System/MinVoltageCellId"] = BMS1.lowest_cell_voltage_num
		dbusservice["/System/MaxCellVoltage"] = BMS1.highest_cell_voltage
		dbusservice["/System/MaxVoltageCellId"] = BMS1.highest_cell_voltage_num
		dbusservice["/System/MinCellTemperature"] = BMS1.lowest_cell_temperature
		dbusservice["/System/MinTemperatureCellId"] = BMS1.lowest_cell_temperature_num
		dbusservice["/System/MaxCellTemperature"] = BMS1.highest_cell_temperature
		dbusservice["/System/MaxTemperatureCellId"] = BMS1.highest_cell_temperature_num
		dbusservice["/System/NrOfModulesOnline"] = 1
		dbusservice["/System/NrOfModulesOffline"] = 0
		dbusservice["/System/NrOfModulesBlockingCharge"] = int(not BMS1.allowed_to_charge)
		dbusservice["/System/NrOfModulesBlockingDischarge"] = int(not BMS1.allowed_to_discharge)
		dbusservice["/Alarms/LowVoltage"] = int(BMS1.alarm_minimum_voltage)
		dbusservice["/Alarms/HighVoltage"] = int(BMS1.alarm_maximum_voltage)
		dbusservice["/Alarms/LowTemperature"] = int(BMS1.alarm_minimum_temperature)
		dbusservice["/Alarms/HighTemperature"] = int(BMS1.alarm_maximum_temperature)

		minUpdateDone = 0
		#now = datetime.now().time()
		#if now.minute != minUpdateDone: 
		#	minUpdateDone = now.minute	

		current_copy = BMS1.pack_current
		if current_copy < 0:
			dbusservice['/TimeToGo'] = (BMS1.soc*BMS1.capacity * 10) * 60 * 60 / (BMS1.pack_voltage * -1 * current_copy)
		else:
			dbusservice['/TimeToGo'] = None

	BMS1.lock.release()

# Called on a one second timer
def handletimertick():
	update_values()

	return True  # keep timer running

BMS1 = SmartBMS(args.device)
instances = [BMS1]
time.sleep(3) # Wait until we have received some data

GLib.timeout_add(1000, handletimertick)
mainloop = GLib.MainLoop()
mainloop.run()