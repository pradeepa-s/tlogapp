import datetime
import time
import threading
import serial

'''
How to use the module?

import tloglib
tlog = tloglib.App('COM6')
tlog.start() starts the server that would read and write data to the device
tlog.stop() stops the server

tlog.get_status() reads status of device
'''

class DebugPrintf:
    def __init__(self) -> None:
        self._msg = []

    def add(self, msg):
        print(msg)
        if msg[0] == b'\x00':
            line = b''.join(msg[3:]).decode('utf-8')
            dt = datetime.datetime.now().isoformat()
            self._msg.append(f"{dt}:\t{line}")
        else:
            print(msg)

    def print(self):
        msg = '\n'.join(self._msg)
        print(msg)
        self._msg = []

class TransportLayer:
    WAIT_FOR_SOH = 0
    WAIT_FOR_DATA_LENGTH_LSB = 1
    WAIT_FOR_DATA_LENGTH_MSB = 2
    WAIT_FOR_DATA = 3

    def __init__(self, dbg_print):
        self._state = TransportLayer.WAIT_FOR_SOH
        self._length = 0
        self._data = []
        self._debug_print = dbg_print

    def submit(self, data):
        if self._state == TransportLayer.WAIT_FOR_SOH and data == b'\x01':
            self._state = TransportLayer.WAIT_FOR_DATA_LENGTH_LSB
        elif self._state == TransportLayer.WAIT_FOR_DATA_LENGTH_LSB:
            self._length = int.from_bytes(data)
            self._state = TransportLayer.WAIT_FOR_DATA_LENGTH_MSB
        elif self._state == TransportLayer.WAIT_FOR_DATA_LENGTH_MSB:
            self._length |= (int.from_bytes(data) << 8)
            self._state = TransportLayer.WAIT_FOR_DATA
        elif self._state == TransportLayer.WAIT_FOR_DATA and self._length > 0:
            self._data.append(data)
            self._length -= 1
            if self._length == 0:                
                self._debug_print.add(self._data)
                self._data = []
                self._length = 0
                self._state = TransportLayer.WAIT_FOR_SOH                    

class Command:
    def __init__(self):
        self.tx_data = None

class CommandRegistry:
    def __init__(self):
        get_status_cmd = Command()
        get_status_cmd.tx_data = b'\x00\x00'
        self._commands = {
            "GetStatus": get_status_cmd
        }

    def get(self, command:str):
        return self._commands[command]

class ThreadControl:
    def __init__(self, dev):
        self._is_enabled = True
        self._write_event:threading.Event = threading.Event()
        self._device_lock = threading.Lock()
        self._read_thread = None
        self._write_thread = None
        self._debug_print = DebugPrintf()
        self._transport_layer = TransportLayer(self._debug_print)
        self._command:Command = None
        self._device:serial.Serial = dev

    def set_threads(self, read_thread, write_thread):
        self._read_thread = read_thread
        self._write_thread = write_thread

    def start(self):
        self._read_thread.start()
        self._write_thread.start()

    def is_enabled(self):
        return self._is_enabled

    def exit(self):
        self._is_enabled = False
        self._write_event.set()
        self._device.cancel_read()
        self._read_thread.join()
        self._write_thread.join()

    def wait_for_write(self):
        self._write_event.wait()
        self._write_event.clear()
        return True

    def acquire_device_for_write(self):
        self._device_lock.acquire()
        return self._device_lock

    def submit_rx_data(self, val):
        self._transport_layer.submit(val)

    def write_pending(self):
        return self._device_lock.locked()

    def wait_until_write_complete(self):
        while self.write_pending():
            pass
    
    def get_write_command(self):
        return self._command.tx_data
    
    def trigger_command(self, cmd: Command):
        if not self._command:
            self._command = cmd
            self._write_event.set()
        else:
            print("Previous command pending, please wait...")


def read_data(dev:serial.Serial, control:ThreadControl):
    while control.is_enabled():
        val = dev.read()
        if val:
            control.submit_rx_data(val)
        if control.write_pending():
            control.wait_until_write_complete()
        

def write_data(dev:serial.Serial, control:ThreadControl):
    while control.wait_for_write():
        if control.is_enabled():        
            lock = control.acquire_device_for_write()
            dev.cancel_read()            
            command_data = control.get_write_command()
            command_length = len(command_data)
            command = b'\x01' + command_length.to_bytes(2, 'little') + command_data
            print(command)
            dev.write(command)
            control._command = None
            lock.release()
        else:
            break

class App:
    def __init__(self, port):
        self._thread_control = None
        self._command_regisrty = CommandRegistry()
        self._device = serial.Serial(port, 112500)

    def start(self):
        self._thread_control = ThreadControl(self._device)
        read_thread = threading.Thread(target=read_data, args=(self._device, self._thread_control))
        write_thread = threading.Thread(target=write_data, args=(self._device, self._thread_control))

        self._thread_control.set_threads(read_thread, write_thread)
        self._thread_control.start()

    def stop(self):
        self._thread_control.exit()

    def get_status(self):
        self._thread_control.trigger_command(self._command_regisrty.get("GetStatus"))

    def print(self):
        self._thread_control._debug_print.print()

# Only for testing
if __name__ == "__main__":
    app = App('COM6')
    app.start()
    app.get_status()
    app.stop()