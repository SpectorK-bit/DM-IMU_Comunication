import serial
import numpy
from struct import unpack
import threading

class DM_IMU_CRC16:
    crc16_table=numpy.array([0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
                            0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
                            0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
                            0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
                            0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
                            0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
                            0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
                            0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
                            0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
                            0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
                            0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
                            0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
                            0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
                            0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
                            0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
                            0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0],dtype=numpy.uint16)
    def crc16_check(data:str):
        crc=numpy.uint16(0xffff)
        for i in range(16):
            index=numpy.uint8((crc >> 8 ^ data[i]))
            crc=numpy.uint16((crc << 1) ^ DM_IMU_CRC16.crc16_table[index])
        return crc

class DM_IMU_usbcmd:
    # IMU_USB_CMD
    IMU_REBOOT=0
    IMU_OFF_ACC=1
    IMU_OFF_AGV=2
    IMU_OFF_ELA=3
    IMU_OFF_QTN=4
    IMU_ON_ACC=5
    IMU_ON_AGV=6
    IMU_ON_ELA=7
    IMU_ON_QTN=8
    IMU_SET_FREQ_100HZ=9
    IMU_SET_FREQ_125HZ=10
    IMU_SET_FREQ_200HZ=11
    IMU_SET_FREQ_250HZ=12
    IMU_SET_FREQ_500HZ=13
    IMU_SET_FREQ_1000HZ=14
    IMU_OFF_CONSTTEMP=15
    IMU_ON_CONSTTEMP=16
    IMU_SET_CONSTTEMP=17
    IMU_OFF_SETMODE=18
    IMU_ON_SETMODE=19
    IMU_SET_SLAVEID=20
    IMU_SET_MASTERID=21
    IMU_SET_COMMTYPE=22
    IMU_SAVE_PARAMS=23
    # IMU_COMMTYPE
    IMU_COMMTYPE_USB=0x00
    IMU_COMMTYPE_RS485=0x01
    IMU_COMMTYPE_CAN=0x01#存疑
    # IMU_CONSTTEMP_SAFTY
    def IMU_CONSTTEMP_SAFTY(val:int):
        if val<0:
            return 0
        elif val>60:
            return 60
        else:
            return val
    # IMU_CONSTTEMP_SAFTY
    def IMU_ID_SAFTY(val:int):
        if val>2047:
            return 2047
        elif val<0:
            return 0
        else:
            return val
    # CMD_List
    _cmdlist=[  b'\xaa\x00\x00\x0d',
                b'\xaa\x01\x04\x0d',
                b'\xaa\x01\x05\x0d',
                b'\xaa\x01\x06\x0d',
                b'\xaa\x01\x07\x0d',
                b'\xaa\x01\x14\x0d',
                b'\xaa\x01\x15\x0d',
                b'\xaa\x01\x16\x0d',
                b'\xaa\x01\x17\x0d',
                b'\xaa\x02\x01\x0d',
                b'\xaa\x02\x02\x0d',
                b'\xaa\x02\x03\x0d',
                b'\xaa\x02\x04\x0d',
                b'\xaa\x02\x05\x0d',
                b'\xaa\x02\x06\x0d',
                b'\xaa\x04\x00\x0d',
                b'\xaa\x04\x01\x0d',
                b'\xaa\x05\x00\x0d',
                b'\xaa\x06\x00\x0d',
                b'\xaa\x06\x01\x0d',
                b'\xaa\x08\x00\x0d',
                b'\xaa\x08\x00\x0d',
                b'\xaa\x0a\x00\x0d',
                b'\xaa\x03\x01\x0d']

class DM_IMU_USB:
    def __init__(self):
        self.__run_flag=False
        self.__run_thread=threading.Thread(target=self.__run,name='dm_imu_run')
        self.__imudata=[0,0,0,0,0,0,0,0,0,0]
        #[acc_x:0, acc_y:1, acc_z:2, agv_x:3, agv_y:4, agv_z:5, ela_x:6, ela_y:7, ela_z:8]
    
    def start(self,COM:str):
        self.__run_flag=True
        self.__serial=serial.Serial(COM,921600,timeout=0.01)
        self.__run_thread.start()
        return True
    
    def stop(self):
        self.__run_flag=False
        while self.__run_thread.is_alive():
            pass
        self.__serial.close()
    
    def get_imudata(self):
        if self.__run_flag==False:
            return []
        return self.__imudata
    
    def get_imudata_acc(self):
        if self.__run_flag==False:
            return []
        return self.__imudata[0:3]
    
    def get_imudata_agv(self):
        if self.__run_flag==False:
            return []
        return self.__imudata[3:6]
    
    def get_imudata_ela(self):
        if self.__run_flag==False:
            return []
        return self.__imudata[6:]
    
    def config(self,usbcmd:int,val:int=0):
        if self.__run_flag==False:
            return False
        self.__serial.reset_output_buffer()
        if usbcmd==DM_IMU_usbcmd.IMU_SET_CONSTTEMP or usbcmd==DM_IMU_usbcmd.IMU_SET_SLAVEID or usbcmd==DM_IMU_usbcmd.IMU_SET_MASTERID or usbcmd==DM_IMU_usbcmd.IMU_SET_COMMTYPE:
            rawcmd=DM_IMU_usbcmd._cmdlist[usbcmd]
            rawcmd=rawcmd[0:2]+bytes([val])+bytes([rawcmd[3]])
            self.__serial.write(rawcmd)
        else:
            self.__serial.write(DM_IMU_usbcmd._cmdlist[usbcmd])
        self.__serial.flush()
        return True
    
    def __run(self):
        self.__serial.reset_input_buffer()
        while self.__run_flag:
            data=self.__serial.read(19)
            if data[0]==numpy.uint8(0x55) and data[1]==numpy.uint8(0xAA) and data[18]==numpy.uint8(0x0A):
                crc16_in_rawdata=numpy.uint16(unpack("@H",data[16:18])[0])
                if crc16_in_rawdata == DM_IMU_CRC16.crc16_check(data):
                    data_type=int(numpy.uint8(data[3]))
                    x_data=float(unpack("@f",data[4:8])[0])
                    y_data=float(unpack("@f",data[8:12])[0])
                    z_data=float(unpack("@f",data[12:16])[0])
                    if data_type==0x01:
                        self.__imudata[0]=x_data
                        self.__imudata[1]=y_data
                        self.__imudata[2]=z_data
                    elif data_type==0x02:
                        self.__imudata[3]=x_data
                        self.__imudata[4]=y_data
                        self.__imudata[5]=z_data
                    else:
                        self.__imudata[6]=x_data
                        self.__imudata[7]=y_data
                        self.__imudata[8]=z_data
                else:
                    continue
            else:
                continue