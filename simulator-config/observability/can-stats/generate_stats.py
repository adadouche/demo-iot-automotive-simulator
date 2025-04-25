import pyudev
import usb.core
import usb.util
import can
from can.bus import BusState
from collections import deque
from statistics import mean
import serial
from datetime import datetime, timezone
import serial.tools.list_ports
from time import sleep
from json import dumps


def getSerPort():
	goldbox={"PID": 0x6001, "VID": 0x0403}
	target= goldbox
	
	while True:
		serPorts= serial.tools.list_ports.comports()
		for port in serPorts:
			if (port.pid ==target["PID"] and port.vid ==target["VID"]):
				return port.device
	

def getCAN():
	canPort= 'can0'
	context= pyudev.Context()
	monitor= pyudev.Monitor.from_netlink(context)
	monitor.filter_by(subsystem='usb')
	mc={"PID": 0x606f, "VID": 0x1d50}
	
	dev = usb.core.find(idVendor=mc['VID'], idProduct=mc['PID'])
	if dev != None:
		return canPort
	
	for device in iter(monitor.poll, None):
		dev = usb.core.find(idVendor=mc['VID'], idProduct=mc['PID'])
		if dev != None:
			return canPort



def receive_all(Serialport, canX):
	thpHistory= deque([])
	dlcHistory= deque([])
	
	last= count= txCount= rxCount= 0
	sendMsg= {"name": "CAN_Stats", "id": 0, "CANBaselineStats": { "CANDataThroughput": 0, "CANDataLoad": 0, "MessagesTransmittedCount": txCount, "MessagesReceivedCount": rxCount}}
	with can.Bus(channel=canX, interface= 'socketcan') as bus: 
		ser= serial.Serial(Serialport, 115200)
		try:
			bus.state= BusState.PASSIVE
		except NotImplementedError:
			pass
			
		print("Entering main loop...")
		try: 
			while True: 
				msg = bus.recv(1)
				if msg is not None:
					if not msg.is_rx:
						txCount+=1
					else:
						rxCount+=1
					
					if count > 59:
						throughPut= 1/mean(thpHistory)
						sendMsg['CANBaselineStats']['CANDataThroughput']= throughPut
						#avg packet size*pack/sec/ bitrate = load
						sendMsg['CANBaselineStats']['CANDataLoad']= mean(dlcHistory) * throughPut / 100000
						sendMsg['CANBaselineStats']['MessagesTransmittedCount']= txCount
						sendMsg['CANBaselineStats']['MessagesReceivedCount']= rxCount
						sendMsg['ts_device']= str(datetime.now(timezone.utc))
						try:
							newMsg= dumps(sendMsg)
							newMsg+= "\n"
							print(newMsg)
							ser.write(newMsg.encode())
						except:
							port= getSerPort()
							canChannel= getCAN()
							receive_all(port, canChannel)
						thpHistory.clear()
						dlcHistory.clear()
						count= 0
					elif count==0:
						last= msg.timestamp
					else:
						thpHistory.append(msg.timestamp-last)
						last= msg.timestamp
						dlcHistory.append(msg.dlc * 8) #convert bytes to bits
						
					count+=1
		except KeyboardInterrupt:
			ser.close()
			




def generateStats():
	print("looking")
	port= getSerPort()
	print(port)
	runStats= True
	canChannel= getCAN()
	while runStats:
		try:
			receive_all(port, canChannel)
			runStats=False
		except OSError:
			runStats=True
			sleep(1)

if __name__ == "__main__":
	generateStats()

###systemd code for launch on startup. 
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.bind(("localhost", 9988))
#s.listen()

#while True:
#	conn, addr = s.accept()
#	data = conn.recv(1024)
#	conn.close()
#	generateStats()
	



