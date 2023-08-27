import datetime
import time
import threading
import serial
import struct

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

    def add(self, msg:bytearray):
        line = b''.join(msg[2:]).decode('utf-8')
        dt = datetime.datetime.now().isoformat()
        self._msg.append(f"{dt}:\t{line}")

    def print(self):
        msg = '\n'.join(self._msg)
        print(msg)
        self._msg = []


class CmdResponseReceiver:
    def __init__(self) -> None:
        self._response_decoders = {}

    def register_decoder(self, cmd, cb):
        self._response_decoders[cmd] = cb

    def add(self, msg:bytearray):
        cmd = int.from_bytes(msg[0:1], "little")
        error = int.from_bytes(msg[2:2], "little")
        length = int.from_bytes(msg[2:3], "little")
        rsp_decoder = self._response_decoders.get(cmd, None)
        if error:
            print(f"Error: {error}")
        elif rsp_decoder:
            rsp_decoder(length, msg[4:])


class GetStatusResponse:
    def __init__(self) -> None:
        pass

    def decode(self, length, data):
        dt = datetime.datetime(2000 + data[0], data[1], data[2], data[3], data[4], data[5])
        temp = int.from_bytes(data[6:9], "little")
        t_c = float(temp/10000)
        print(f"GetStatus response: {dt}: {t_c} C")

class SetDatetimeResponse:
    def __init__(self) -> None:
        pass

    def decode(self, length, data):
        year=2000 + data[0]
        month=data[1]
        day=data[2]
        hour=data[3]
        minute=data[4]
        second=data[5]
        dt = datetime.datetime(year=2000 + data[0], month=data[1], day=data[2], hour=data[3], minute=data[4], second=data[5])
        print(f"SetDatetime response: {dt}")

class ResponseRouter:
    def __init__(self) -> None:
        self._receivers = {}

    def register_receiver(self, code:int, cb):
        self._receivers[code] = cb

    def submit_data(self, data:bytearray):
        code = int.from_bytes(data[0:1], "little")
        rx = self._receivers.get(code, None)
        if rx:
            rx(data[1:])
        else:
            print(f"Unhandled rx data {data}")
        

class TransportLayer:
    WAIT_FOR_SOH = 0
    WAIT_FOR_DATA_LENGTH_LSB = 1
    WAIT_FOR_DATA_LENGTH_MSB = 2
    WAIT_FOR_DATA = 3

    def __init__(self, router:ResponseRouter):
        # TODO: Get presentation layer
        self._response_router = router
        self._reset()

    def _reset(self):
        self._state = TransportLayer.WAIT_FOR_SOH
        self._length = 0
        self._data:bytearray = bytearray()

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
            self._data += bytearray(data)
            self._length -= 1
            if self._length == 0:                
                self._response_router.submit_data(self._data)
                self._reset()                  

class Command:
    def __init__(self, code):
        self._command = code
        self._params:bytes = b''

    def add_params(self, param):
        raise NotImplementedError
    
    def serialize(self) -> bytes:
        tx_data = b'\x01'
        tx_data += self._command
        tx_data += len(self._params).to_bytes(2, 'little')
        tx_data += self._params
        return tx_data

class SetDatetimeCommand(Command):
    def add_params(self, dt:datetime.datetime):
        year = dt.year - 2000
        self._params = struct.pack("BBBBBB", year, dt.month, dt.day,
                                   dt.hour, dt.minute, dt.second)        

class GetStatusCommand(Command):
    def __init__(self, code):
        super().__init__(code)

class CommandBuilder:
    def __init__(self):
        self._commands = {
            "GetStatus": b'\x00',
            "SetDatetime": b'\x01'
        }

        self._command_builders = {
            "GetStatus": GetStatusCommand,
            "SetDatetime": SetDatetimeCommand
        }

    def get(self, command:str) -> Command:
        cmd_code = self._commands[command]
        return self._command_builders[command](cmd_code)

class ThreadControl:
    def __init__(self, dev):
        self._is_enabled = True
        self._write_event:threading.Event = threading.Event()
        self._device_lock = threading.Lock()
        self._read_thread = None
        self._write_thread = None
        self._debug_print = DebugPrintf()
        self._cmd_response_rx = CmdResponseReceiver()

        self._rx_router = ResponseRouter()
        self._rx_router.register_receiver(0, self._debug_print.add)
        self._rx_router.register_receiver(2, self._cmd_response_rx.add)

        self._get_status_response = GetStatusResponse()
        self._set_datetime_response = SetDatetimeResponse()
        self._cmd_response_rx.register_decoder(0, self._get_status_response.decode)
        self._cmd_response_rx.register_decoder(1, self._set_datetime_response.decode)

        self._transport_layer = TransportLayer(self._rx_router)
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
        return self._command.serialize()
    
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
        self._command_builder = CommandBuilder()
        self._port = port
        self._device = None

    def start(self):
        self._device = serial.Serial(self._port, 112500)
        self._thread_control = ThreadControl(self._device)
        read_thread = threading.Thread(target=read_data, args=(self._device, self._thread_control))
        write_thread = threading.Thread(target=write_data, args=(self._device, self._thread_control))

        self._thread_control.set_threads(read_thread, write_thread)
        self._thread_control.start()

    def stop(self):
        self._thread_control.exit()
        self._device.close()
        self._device = None

    def get_status(self):
        self._trigger_command("GetStatus")

    def set_datetime_now(self):
        dt = datetime.datetime.now()
        self.set_datetime(dt)

    def set_datetime(self, dt):
        self._trigger_command("SetDatetime", params=dt)

    def print(self):
        self._thread_control._debug_print.print()

    def _trigger_command(self, command, params=None):
        cmd = self._command_builder.get(command)
        if params:
            cmd.add_params(params)
        self._thread_control.trigger_command(cmd)


if __name__ == "__main__":
    app = App('COM6')
    app.start()

    import code
    vars = globals().copy()
    vars.update(locals())
    shell = code.InteractiveConsole(vars)
    shell.interact()