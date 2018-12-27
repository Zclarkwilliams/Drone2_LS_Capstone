#!/usr/bin/python

from __future__ import print_function


import sys
import string
import time
import datetime
import pexpect
from bitstring import BitArray, BitStream
import os
#For serial stuff
from pexpect import fdpexpect
import serial

SSH='/usr/bin/ssh'
TELNET='/usr/bin/telnet'
SSH_OPTIONS='-xo UserKnownHostsFile="/dev/null"'
repeatCount = 80 #Number of times to print a '=' to seperate fields

IMU_PIT_ROT_ANG_INDEX = 0
IMU_ROL_ROT_ANG_INDEX = 1
IMU_YAW_ROT_ANG_INDEX = 2
IMU_PIT_ROT_RAT_INDEX = 3
IMU_ROL_ROT_RAT_INDEX = 4
IMU_YAW_ROT_RAT_INDEX = 5
IMU_CALIBR_STAT_INDEX = 6
RECV_THR_VAL_INDEX    = 7
RECV_YAW_VAL_INDEX    = 8
RECV_ROL_VAL_INDEX    = 9
RECV_PIT_VAL_INDEX    = 10
RECV_AUX1_VAL_INDEX   = 11
RECV_AUX2_VAL_INDEX   = 12
RECV_SWAB_VAL_INDEX   = 13
YAAC_ANG_ERR_INDEX    = 14
YAAC_ANG_TGT_INDEX    = 15
YAAC_DBG_OUT_INDEX    = 16
YAAC_STK_NEU_INDEX    = 17

name_indices = {
    0 : "IMU X rotation angle   ",
    1 : "IMU Y rotation angle   ",
    2 : "IMU Z rotation angle   ",
    3 : "IMU X rotation rate    ",
    4 : "IMU Y rotation rate    ",
    5 : "IMU Z rotation rate    ",
    6 : "IMU calibration status ",
    7 : "Recv throttle value    ",
    8 : "Recv yaw value         ",
    9 : "Recv roll value        ",
    10: "Recv pitch value       ",
    11: "Recv aux1 value        ",
    12: "Recv aux2 value        ",
    13: "Recv SWA/SWB value     ",
    14: "YAAc Yaw angle error   ",
    15: "YAAc Yaw angle target  ",
    16: "YAAc debug output      ",
    17: "YAAc stick not neutral ",
}

disp_type = {
    0 : 'signed16',
    1 : 'signed16',
    2 : 'unsigned16',
    3 : 'signed16',
    4 : 'signed16',
    5 : 'signed16',
    6 : 'binary8',
    7 : 'unsigned8',
    8 : 'unsigned8',
    9 : 'unsigned8',
    10: 'unsigned8',
    11: 'unsigned8',
    12: 'unsigned8',
    13: 'unsigned8',
    14: 'signed16',
    15: 'signed16',
    16: 'signed16',
    17: 'binary8',
}


def main():
    #conn_type  ="serial"
    conn_type  ="SSH"
    #conn_type  ="telnet"
    while True:
        result, console = connect_console(conn_type)
        if not result:
            continue
        if not parse_output(console, conn_type):
            return

def connect_console(conn_type):
    if conn_type == "serial": #Handle serial connection
        return connect_local()
    else: #Handle SSH/Telnet connection
        return connect_remote(conn_type)


def connect_local():
    print("Connecting to console via serial port")
    try:
        serial_port = serial.Serial()
        #print("Default serial configration: {0}".format(serial_port))
        serial_port.baudrate = 115200
        serial_port.port="/dev/ttyUSB0"
        serial_port.bytesize=8
        serial_port.parity='N'
        serial_port.stopbits=1
        serial_port.timeout=None
        serial_port.xonxoff=False
        serial_port.rtscts=False
        serial_port.dsrdtr=False
    except:
        sys.exit ("Error creating serial port object")
    if not serial_port.is_open:
        try:
            serial_port.open()
            print("Serial port opened")
        except:
            sys.exit ("Error opening serial port")
    else:
        sys.exit ("Error: serial port already open")
    print("Serial port configuration : ")
    for item in repr(serial_port).split(','):
        print(item)
    return True, serial_port



def connect_remote(conn_type):
    ip         = "192.168.10.1"
    sshport    = "4001"
    telnetport = "2167"
    username   = "admin"
    password   = "admin"
    timeout    = 20
    port       = 0
    tried_password = False
    connect_cmd = None
    if conn_type == "SSH":
        print("Connecting to console via SSH")
        timeout = 20
        connect_cmd = SSH + ' ' + SSH_OPTIONS + ' ' + username + '@' + ip + ' -p ' + sshport
    else:
        print("Connecting to console via Telnet")
        timeout = 2
        connect_cmd = TELNET + ' ' + ip + ' ' + telnetport
    try:
        console = pexpect.spawn(connect_cmd)
    except KeyboardInterrupt:
        sys.exit ("\nExiting Pexpect spawn by keyboard interrupt")
    except Exception as e:
        sys.exit ("Error: {0}".format(e))
    while True:
        #console.logfile = sys.stdout
        try:
            result = console.expect(['continue connecting',
                                    'Password|password',
                                    'Username|username|User|user',
                                    'Connection refused',
                                    'Connection to .* closed', 
                                    'denied',
                                    '\n\r',
                                    'Connected.*\'\^\]\'\.',
                                    pexpect.EOF],
                                    timeout = timeout)
            if timeout > 2:
                timeout = 2
            if result == 0: #Confirm host key
                print("Accepting host key")
                console.sendline('yes')
            elif result == 1: #Enter password
                print("Entering password")
                if not tried_password: #Only enter password once
                    if console.logfile: #Hide password in debugs
                        console.logfile = None
                        console.sendline(password)
                        console.logfile = sys.stdout
                    else:
                        console.sendline(password)
                    tried_password = True
                else: #Password error, already tried this password
                    print("Error: Invalid username/password combo")
                    return False, console
            elif result == 2: #Enter user name
                print("Entering username: \'{0}\'".format(username))
                console.sendline(username)
            elif result == 4: #Connection refused
                print("Connection refused")
                print("Error: {0}".format(console.before))
                return False, console
            elif result == 4: #Session closed
                print("Session closed")
                print("Error: {0}".format(console.before))
                return False, console
            elif result == 5: #Password rejected
                print("Error: Password rejected, invalid username/password combo?")
                return False, console
            elif result == 6: # Receiving valid data
                return True, console
            elif result == 7: # Telnet connected
                print("Connected via Telnet")
                return True, console
            elif result == 8: # EOF, return error
                print("EOF Returned")
                return False, console
            else:
                print("Error: {0}".format(console.before))
                return False, console
        except KeyboardInterrupt:
            sys.exit ("\nExiting remote console connect by keyboard interrupt")
        except pexpect.TIMEOUT:
            if tried_password or conn_type != "telnet":
                print("Prompt wait timeout. Probably connected with no data stream output, check drone power?")
                return True, console
            else:
                sys.exit ("Failed to connect to remote console, timed out")
        except Exception as e:
            print(e)
            sys.exit ("Exiting remote connect due to unknown cause")


def parse_output(console, conn_type= "ssh"):
    prompt = '\n\r'
    output = ''
    longer_timeout = True
    while True:
        index = 0
        value = 0
        value_string = ''
        if longer_timeout:
            timeout = 10
        elif conn_type == "SSH" or conn_type == "telnet":
            timeout = 1
        else:
            timeout = 0.1
        try:
            line = ''
            if conn_type == "serial":
                line = console.readline()
            else:
                console.expect([prompt, pexpect.EOF], timeout=timeout)
                line = console.before
            longer_timeout = False
            line = line.lstrip().rstrip()# remove any control or whitespace characters (' ', '\n', '\r', and so on)
            if line.startswith("\x00"):
                line = line.lstrip("\x00")
            #print("This line = {0}".format(repr(line)))
            if len(line) != 6: #skip partial lines, read started in the middle of a hex string
                continue
            index = int(line[0:2], 16)
            value = int(line[3:], 16)
            if disp_type[index] == 'signed8':
                bit_array = BitArray(hex=line[-2:])
                value_string = bit_array.int
            if disp_type[index] == 'signed16':
                bit_array = BitArray(hex=line[-4:])
                value_string = bit_array.int
            elif disp_type[index] == 'signed32':
                bit_array = BitArray(hex=line[-8:])
                value_string = bit_array.int
            elif disp_type[index] == 'unsigned8':
                bit_array = BitArray(hex=line[-2:])
                value_string = bit_array.uint
            elif disp_type[index] == 'unsigned16':
                bit_array = BitArray(hex=line[-4:])
                value_string = bit_array.uint
            elif disp_type[index] == 'unsigned32':
                bit_array = BitArray(hex=line[-8:])
                value_string = bit_array.uint
            elif disp_type[index] == 'binary8':
                bit_array = BitArray(hex=line[-2:])
                value_string = "0b" + bit_array.bin
            elif disp_type[index] == 'binary16':
                bit_array = BitArray(hex=line[-4:])
                value_string = "0b" + bit_array.bin
            elif disp_type[index] == 'binary32':
                bit_array = "0b" + BitArray(hex=line[-8:])
                value_string = bit_array.bin
            stime = datetime.datetime.now().strftime('%H:%M:%S.%f')
            output = output + "{0} : {1} : {2}\n".format(stime, name_indices[index], value_string)
            if index == YAAC_STK_NEU_INDEX:
                print(output)
                output = ''
        except KeyboardInterrupt:
            sys.exit ("\nExiting while waiting for new line by keyboard interrupt")
        except pexpect.TIMEOUT:
            print("Receive data timeout!, Reconnecting")
            longer_timeout = True
            return True
        except Exception as e:
            sys.exit("Exiting waiting for new line due to error: {0}".format(e))
    return False
if __name__=="__main__":
    main()
