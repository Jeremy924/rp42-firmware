import json
import urllib.request
import urllib.error
import serial.tools.list_ports
import time

import serial.tools.list_ports
import ctypes

from serial.serialutil import SerialException


def _print_status(current, total, ser):
    ser.write(f'BM {int(current / total * 256)}\n'.encode())

def compute_checksum(data: bytes) -> int:
    checksum_obj = ctypes.c_int32(0)

    for b_val in data:
        if b_val != 255:
            checksum_obj.value += b_val

    return int(checksum_obj.value)

def update_firmware(data: bytes, port: str):
    install_app(data, port, 0x7C0000, 'ef')

def install_app(data: bytes, port: str, offset: int=0, erase_command: str='ec')-> bool:
    #W 468992 1024
    ser = serial.Serial(port, 9600, timeout=1)

    ser.write('echo off\nmscn\nfrw\n'.encode())
    ser.read(ser.in_waiting)

    ser.write(f'txt\nERASING\n{erase_command}\n'.encode())
    while ser.in_waiting == 0:
        time.sleep(0.1)
    ser.read(ser.in_waiting)

    size = 1024

    ser.write('txt\nDOWNLOADING\n'.encode())
    sb = True
    count = 0
    for i in range(0, len(data) + 1023, size):
        if count == 5:
            _print_status(i, len(data), ser)
            count = 0
        d = data[i:i+size]
        if len(d) < size:
            d += b'\xFF' * (size - len(d))
        ser.write(f'W {i + offset} {size} \n'.encode())
        ser.write(d)
        serial_failed = 0
        while True:
            time.sleep(0.01)
            if ser.in_waiting != 0:
                read = ser.read(ser.in_waiting)
                if 'done'.encode() in read:
                    break

        count += 1

    ser.read(ser.in_waiting)
    ser.write('C\n'.encode())
    while ser.in_waiting == 0:
        time.sleep(0.1)

    if erase_command == 'ef':
        ser.write("BM 0\ntxt\nHold down button 1\n".encode())
        input("Please hold down top left button on your calculator until the calculator resets. Press enter on PC once button is held down")
        ser.write('BM 0 \ntxt\nINSTALLING\npfirm\n'.encode())
        time.sleep(5)
        return True

    calculated_checksum = 0#int(ser.read(ser.in_waiting).decode().strip())
    actual_sum = compute_checksum(data)
    if calculated_checksum != actual_sum:
        print(f'Expected {actual_sum}, got {calculated_checksum}')
        ser.write('txt\nChecksum does not match!\nBM 0 \nexit\necho on\n'.encode())
    else:
        ser.write('BM 0 \ntxt\nDone! Reset calculator\nexit\necho on\n'.encode())
    time.sleep(2)

    ser.write('bootmode 0\nreset\n'.encode())
    ser.close()

    return True

def setup_fs(port):
    ser = serial.Serial(port, 9600, timeout=1)
    ser.write('echo off\nmscn\n'.encode())
    ser.write('frw\ntxt\nCreating Filesystem\nexit\nmkfs\n'.encode())
    print("Creating filesystem")
    time.sleep(45)

    ser.write('frw\ntxt\nSetting up Folders\nexit\n'.encode())
    ser.write('mkdir System\nmkdir Apps\nmkdir AppData\nmkdir AppData/Free42\n'.encode())
    ser.write('frw\ntxt\ndone\nexit\n'.encode())
    print('Finished')
    time.sleep(2)
    ser.write('bootmode 1\nhostnamectl RP-42\nreset\n'.encode())


def list_serial_ports():
    """ Lists serial port names and descriptions """

    # comports() returns a list of ListPortInfo objects
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("No serial ports found.")
        return

    port_nums = []
    print("Available serial ports:")
    for port in sorted(ports):
        # The 'port' object has attributes like 'device', 'description', 'hwid'
        print(f"  - Device: {port.device}")
        print(f"    Description: {port.description}")
        print(f"    Hardware ID: {port.hwid}")
        print("-" * 20)
        port_nums.append(port.device)

    return port_nums

def download_latest_bin(app_list_url: str)->(str, str):
    """Gets the latest Free42 for Rp-42 binary and returns the file path with the binary"""
    # download release list
    try:
        with urllib.request.urlopen(app_list_url) as response:
            data_bytes = response.read()
            encoding = response.info().get_content_charset()
            data_string = data_bytes.decode(encoding)

            decoded_data = json.loads(data_string)
    except urllib.error.URLError as e:
        # Handles errors like network issues, invalid URL, etc.
        print(f"URL Error: {e.reason}")
        return '', ''
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        # You might want to inspect data_string here if decoding fails
        # print(f"Response content: {data_string}")
        return '', ''
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return '', ''
    # search release list for latest version
    if len(decoded_data) == 0:
        # no releases?
        return '', ''

    latest_release = decoded_data[0]
    url = latest_release['url']
    version = latest_release['version']


    # download latest version
    try:
        with urllib.request.urlopen(url) as response:
            if response.status != 200:
                return '', ''
            data = response.read()
    except urllib.error.URLError as e:
        # Handles errors like network issues, invalid URL, etc.
        print(f"URL Error: {e.reason}")
        return '', ''
    except json.JSONDecodeError as e:
        print(f"Error decoding JSON: {e}")
        # You might want to inspect data_string here if decoding fails
        # print(f"Response content: {data_string}")
        return '', ''
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return '', ''

    # return file path of latest version
    return data, version

def download_latest_free42bin()->(str, str):
    """Gets the latest Free42 for Rp-42 binary and returns the file path with the binary"""
    return download_latest_bin('https://jeremy924.github.io/rp42/downloads/free42.json')

def download_latest_firmware():
    # download latest version
    return download_latest_bin('https://jeremy924.github.io/rp42/downloads/firmware.json')

if __name__ == "__main__":
    print("This installer is intended to update from firmware >=0.0.2 to the latest firmware and also update Free42.")
    print("If no port is listed below, then reset the calculator while holding the top left button. Then restart the installer.")
    print("If there are still no ports listed, then you may be using an even older version of firmware than what is supported.\n\n")

    ports = list_serial_ports()
    if len(ports) == 1:
        port = ports[0]
    else: port = input("Enter port: ")
    firmware, version = download_latest_firmware()
    print("Latest firmware version:", version)
    print("Installing firmware")
    update_firmware(firmware, port)

    print("\n\nSetting up FS. \033[1mIMPORTANT: Do not interact with the calculator filesystem while installing. This may mess up the installation\033[0m")
    ports = list_serial_ports()
    if len(ports) == 1:
        port = ports[0]
    else: port = input("Enter port: ")
    setup_fs(port)

    print("\n\nInstalling Free42")
    ports = list_serial_ports()
    if len(ports) == 1:
        port = ports[0]
    else: port = input("Enter port: ")

    free42, version = download_latest_free42bin()

    print(f"Latest version of Free42 for RP-42 is \x1b[42m{version}\x1b[0m")

    time.sleep(10)
    install_app(free42, port)
