# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.

import sys
import os
from os.path import getsize
import threading
import time
import serial
import serial.tools.list_ports
from xmodem import XMODEM
import struct

seri = serial.Serial()
event = threading.Event()

def getc(size, timeout=1):
    return seri.read(size) or None

def putc(data, timeout=1):
    seri.write(data)
    event.set()

xmod_success = False
xmod = XMODEM(getc, putc, 'xmodem1k')
class ThreadSend(threading.Thread):
    def  __init__(self, fd):
        threading.Thread.__init__(self)
        self.fd = fd

    def run(self):
        global xmod_success
        xmod.send(self.fd)
        print("send finish")
        self.fd.close()
        print("fd close")
        xmod_success = True

class XmodemDownload:
    # Supported binary opcodes
    # set
    OPCODE_H2D_EXIT         = 0xFC # exit binary mode
    OPCODE_H2D_WRWD         = 0xF4 # write a word
    OPCODE_H2D_WRWDMSK      = 0xEC # write a word with mask
    OPCODE_H2D_WRMEMBLK     = 0xE4 # write memory block
    OPCODE_H2D_LDXMDM       = 0xDC # xmodem load
    OPCODE_H2D_LDYMDM       = 0xD4 # ymodem load
    OPCODE_H2D_WREFUSE      = 0xCC # write efuse
    OPCODE_H2D_EXEFUN_NR    = 0xC4 # execute function without return value
    # get
    OPCODE_H2D_RDVER        = 0xF8 # read version
    OPCODE_H2D_RDWD         = 0xF0 # read a word
    OPCODE_H2D_RDMEMBLK     = 0xE8 # read memory block
    OPCODE_H2D_RDEFUSE      = 0xE0 # read efuse
    OPCODE_H2D_EXEFUN_R     = 0xD8 # execute function with return value
    # ack
    OPCODE_D2H_WELCOME      = 0x80 # welcome at first
    OPCODE_D2H_BYEBYE       = 0x88 # bye-bye and exit
    OPCODE_D2H_ACK          = 0x90 # normal ack
    OPCODE_D2H_ACK_RDVER    = 0x98 # ack for "read version"
    OPCODE_D2H_ACK_RDWD     = 0xA0 # ack for "read a word"
    OPCODE_D2H_ACK_RDMEMBLK = 0xA8 # ack for "read memory block"
    OPCODE_D2H_ACK_RDEFUSE  = 0xB0 # ack for "read efuse"
    OPCODE_D2H_ACK_EXEFUN_R = 0xB8 # ack for "execute function with return value"
    # err
    OPCODE_D2H_ERR_CHECKSUM = 0x84 # checksum err
    OPCODE_D2H_ERR_UNKNOWN  = 0x8C # unknown opcode
    OPCODE_D2H_ERR_LENGTH   = 0x94 # length err
    OPCODE_D2H_ERR_ARGUMENT = 0x9C # argument err
    OPCODE_D2H_ERR_TIMEOUT  = 0xA4 # timeout err

    # flash base
    LOAD_BASE_ADDR = 0x100000

    isWaitingForXModemSignal = False
    startXModemSignal = threading.Event()
    tx_cnt = 0
    file_size = 0
    fd = ''

    def __init__(self, port, filename):
        self.port = port
        self.filename = filename
        # 定时器接收数据
        self.timer = threading.Timer(1, self.data_receive)

    # 打开串口
    def port_open(self):
        seri.port = self.port
        seri.baudrate = 921600
        seri.bytesize = serial.EIGHTBITS
        seri.stopbits = serial.STOPBITS_ONE
        seri.parity = serial.PARITY_NONE
        #print(seri.port)
        #print(seri.baudrate)
        #print(seri.bytesize)
        #print(seri.stopbits)
        #print(seri.parity)

        self.isWaitingForXModemSignal = False
        self.binaryModeEntered = False

        try:
            seri.open()
        except:
            print("此串口不能被打开！")
            return False

        if seri.isOpen():
            print("串口已打开！")
        else:
            return False

        # 打开串口接收定时器，周期为1秒
        self.timer.start()
        return True

    # 关闭串口
    def port_close(self):
        #self.timer.stop()
        #self.timer_send.stop()
        try:
            seri.close()
        except:
            pass
        # 接收数据和发送数据数目置零
        print("串口状态（已关闭）")

    def binModeEnterExit(self):
        if seri.is_open:
            if self.binaryModeEntered:
                self.binaryModeEntered = False
                # cmd_str = struct.pack(b'<BB', self.OPCODE_H2D_EXIT, 0x02)
                # cmd_str += struct.pack(b'B', sum(c for c in cmd_str) & 0xFF) # calc checksum
                # seri.write(cmd_str)
                # seri.write(b'con.e 1\r')
            else:
                self.binaryModeEntered = True
                #self.ser.write(b'con.e 0\r')
                #self.ser.write(b'bin\r')
                #seri.write(b'f 1 3 1 2 1\r')
                #seri.write(b'f 3\r')
                #self.ser.write(b'x 8000000\r')
                seri.write(b'x 100000\r')
                seri.write(b'g 160000\r')
                cmd_str = struct.pack(b'<BB', self.OPCODE_H2D_RDVER, 0x02)
                cmd_str += struct.pack(b'B', sum(c for c in cmd_str) & 0xFF) # calc checksum
                print(cmd_str)
                seri.write(cmd_str)
        else:
            print("Open Serial First !")

    # 接收数据
    def data_receive(self):
        try:
            num = seri.inWaiting()
            # print("***recv num***")
            # print(num)
        except:
            self.port_close()
            return None
        if num > 0:
            data = seri.read(num)
            num = len(data)
            if self.binaryModeEntered:
                if self.isWaitingForXModemSignal and (data[-1] == b'C'[0] or data[-1] == b'\x15'[0]):
                    self.isWaitingForXModemSignal = False
                    self.startXModemSignal.set()
                    time.sleep(0.05)
             # hex显示
            if False:
                out_s = ''
                for i in range(0, len(data)):
                    out_s = out_s + '{:02X}'.format(data[i]) + ' '
                print(out_s)
            else:
                print(data)
        else:
            pass

    def UpProgress(self):
        while True:
            event.wait()
            self.tx_cnt = self.tx_cnt + 1
            #print(self.tx_cnt)
            event.clear()

    def sendFile(self):
        if not os.path.exists(self.filename):
            print("File path error! path:%s" % (self.filename))
            return
        try:
            self.fd = open(self.filename, "rb")
        except Exception as e:
            print("打开文件失败！")
            return
        self.file_size = getsize(self.filename)
        print(self.file_size)
        #print(self.binaryModeEntered)
        if self.binaryModeEntered:
            loadAddrInt = self.LOAD_BASE_ADDR
            self.isWaitingForXModemSignal = True
            if (loadAddrInt & 0xFF000000) in [0x04000000, 0x08000000]:
                cmd_str = struct.pack(b'<BBII', self.OPCODE_H2D_LDXMDM, 0x0A, loadAddrInt, self.file_size)
            else:
                cmd_str = struct.pack(b'<BBI', self.OPCODE_H2D_LDXMDM, 0x06, 0x00100000)
            cmd_str += struct.pack(b'B', sum(c for c in cmd_str) & 0xFF) # calc checksum
            print(cmd_str)
            seri.write(cmd_str)
            self.startXModemSignal.wait()
            self.startXModemSignal.clear()
            self.tx_cnt=0
            if not self.isWaitingForXModemSignal:
                print("position 1")
                self.threadsend = ThreadSend(self.fd)
                self.threadsend.daemon = True
                self.threadsend.start()
                self.thprogress = threading.Thread(target=self.UpProgress)
                self.thprogress.daemon = True
                self.thprogress.start()
                print("position 2")
        else:
            seri.write(self.fd.read()) #TODO: optimize send in new thread
        #self.fd.close()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    plist = list(serial.tools.list_ports.comports())

    if len(plist) <= 0:
        print("The Serial port can't find!")
    else:
        plist_0 = list(plist[0])
        serialName = plist_0[0]
        serialFd = serial.Serial(serialName, 9600, timeout=60)
        print("check which port was really used >", serialFd.name)

    xd = XmodemDownload("COM4", ".\\test_wifi_bt.bin")

    if not xd.port_open():
        exit(-1)

    xd.binModeEnterExit()
    xd.sendFile()
    xd.binModeEnterExit()

    while (not xmod_success):
        time.sleep(1)

    print(xd.tx_cnt)
    xd.port_close()

    if (xmod_success):
        exit(0)

    exit(-1)

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
