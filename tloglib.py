import time
import threading
import serial

'''
How to use the module?

- import the tloglib module
- tloglib.start() starts the server that would read and write data to the device
- tloglib.stop() stops the server
'''

class DebugPrintf:
    @staticmethod
    def print(msg):
        if msg[0] == b'\x00':
            line = b''.join(msg[3:]).decode('utf-8')
            print(f'[DEBUG_TX]\n>>>>>\n{line}\n<<<<<')
        else:
            print(msg)

class TransportLayer:
    WAIT_FOR_SOH = 0
    WAIT_FOR_DATA_LENGTH_LSB = 1
    WAIT_FOR_DATA_LENGTH_MSB = 2
    WAIT_FOR_DATA = 3

    def __init__(self):
        self._state = TransportLayer.WAIT_FOR_SOH
        self._length = 0
        self._data = []

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
                DebugPrintf.print(self._data)
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
    def __init__(self):
        self._is_enabled = True
        self._write_event = threading.Event()
        self._device_lock = threading.Lock()
        self._read_thread = None
        self._write_thread = None
        self._transport_layer = TransportLayer()
        self._command:Command = None

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

        self._read_thread.join()
        self._write_thread.join()

    def wait_for_write(self):
        self._write_event.wait()

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
            command = control.get_write_command()
            dev.write(command)
            lock.release()
        else:
            break

class App:
    def __init__(self):
        self._thread_control = None
        self._command_regisrty = CommandRegistry()

    def start(self):
        device = serial.Serial("COM6", 112500)

        thread_control = ThreadControl()
        read_thread = threading.Thread(target=read_data, args=(device, thread_control))
        write_thread = threading.Thread(target=write_data, args=(device, thread_control))

        thread_control.set_threads(read_thread, write_thread)
        thread_control.start()

        self._thread_control = thread_control

    def stop(self):
        self._thread_control.exit()

    def get_status(self):
        self._thread_control.trigger_command(self._command_regisrty.get("GetStatus"))


# Only for testing
if __name__ == "__main__":
    app = App()
    app.start()
    time.sleep(10)
    app.stop()
    print("Done")