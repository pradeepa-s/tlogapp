import time
import threading
import serial

'''
How to use the module?

- import the tloglib module
- tloglib.start() starts the server that would read and write data to the device
- tloglib.stop() stops the server
'''

class ThreadControl:
    def __init__(self):
        self._is_enabled = True
        self._write_event = threading.Event()
        self._device_lock = threading.Lock()
        self._read_thread = None
        self._write_thread = None
        self._rx = []

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
        if val == b'\n':
            line = b''.join(self._rx)
            print(line.decode('utf-8'))
            self._rx = []
        elif val != b'\r':
            self._rx.append(val)

    def write_pending(self):
        return self._device_lock.locked()

    def wait_until_write_complete(self):
        while self.write_pending():
            pass
    
    def get_write_command(self):
        return self._command.tx_data


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


if __name__ == "__main__":
    app = App()
    app.start()
    time.sleep(10)
    app.stop()
    print("Done")