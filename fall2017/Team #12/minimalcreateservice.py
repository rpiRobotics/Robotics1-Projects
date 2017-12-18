import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import threading
import serial
import struct

minimal_create_interface="""
service experimental.minimal_create

object create_obj
    function void Drive(int16 velocity, int16 radius)
end object
"""

class create_impl(object):
    def __init__(self, port):
        self._lock=threading.Lock()
        self._serial=serial.Serial(port=port,baudrate=57600)
        dat=struct.pack(">4B",128,132,150, 0)
        self._serial.write(dat)

    def Drive(self, velocity, radius):
        with self._lock:
            dat=struct.pack(">B2h",137,velocity,radius)
            self._serial.write(dat)

#Create and register a transport
t=RR.TcpTransport()
t.StartServer(52222)
RRN.RegisterTransport(t)

#Register the service type
RRN.RegisterServiceType(minimal_create_interface)

create_inst=create_impl("/dev/ttyUSB0")

#Register the service
RRN.RegisterService("Create","experimental.minimal_create.create_obj",create_inst)

#Wait for program exit to quit
raw_input("Press enter to quit")