import time
import sys, getopt
import yaml
import re
import binascii
import RPi.GPIO as rGPIO
from ecomet_i2c_sensors import i2c_command
from ecomet_i2c_sensors.eeprom import chip_list,eeprom_write,eeprom_read
from smbus2 import SMBus
from eeprom_math import hex_to_bytes,hex_to_2bytes,hex_to_3bytes,calculate_byte_crc,calculate_2byte_crc

#from i2c import i2c_io

class StartEnd(object):
    def __init__ (self, start=None, end=None) :
       self.start = start
       self.end = end
    def out(self) :
       return ([self.start,self.end])

class EEPROM_FS(object):

    # Error message
    ERR_PASS = 0x00
    ERR_CRC16_OK = 0x01
    ERR_TOC_INCOSISTENT = 0x10
    ERR_MEMORY_IS_FULL  = 0x11
    ERR_CRC16_WRONG = 0x12

    def __init__(self, chip_ic=None, chip_address=None, busnum=None, writestrobe=None, TOC_start_address=None, i2c=None, **kwargs) :

        self.chip_ic = chip_ic    
        self.chip_address = chip_address
        self.busnum = busnum
        self.writestrobe = writestrobe
        self.block_size=None
        self.TOC_start_address = TOC_start_address
        self.toc_version = None
        self.toc_FreeMemorySize = None
        self.toc_NumberOfFiles = None
        self.toc_NumberOfOrphanBlocks = None
        self.toc_NumberOfDeletedBlocks = None
        self.toc_NumberOfWriteBlocks = None
        self.toc_WearLevelThreshold = None
        self.toc_WearLevelCount = None
        self.toc_FileList = None
        self.toc_OwnerHashList = None
        self.toc_crc = None
        self.toc_DataBlock = None

        self.fh_filename = None
        self.fh_filetype = None
        self.fh_FilsSize = None
        self.fh_NumberOfUsedBlocks = None
        self.fh_Attributes = None
        self.fh_InUse = None
        self.fh_ModificationDate = None
        self.fh_CreationDate = None
        self.fh_crc = None

        self.fh_filename_data = 0x00
        self.fh_build_data = None

        self.toc_data = []

        self.toc_version_data = None
        self.toc_FreeMemorySize_data = None
        self.toc_FileList_data = []
        self.toc_NumberOfFiles = None
        self.toc_NumberOfOrphanBlocks = None
        self.toc_NumberOfDeletedBlocks = None
        self.toc_NumberOfWriteBlocks = None
        self.toc_crc_data = None

        self.file_db_address = 0x00
        self.file_block = []
        self.file_gap = []

        self.error_code = {}

        self.init_config()
        if self.chip_address is None:
           self.chip_address = 0x50
        if self.TOC_start_address is None:
           self.TOC_start_address = 0
        if self.busnum is None:
           self.busnum = 'i2c-0'

        self.wear_level_threshold = 100 # number of writes before triggering wear leveling
        self.wear_level_count = 0

    def init_config(self):
        with open("/home/comet/.comet/config.yaml") as c:
           try:
              config = yaml.safe_load(c)
           except yaml.YAMLError as exc:
              print(exc)

        if self.chip_ic is None:
           self.chip_ic= config['i2c']['eeprom']['ic']
        if self.chip_address is None:
           self.chip_address= config['i2c']['eeprom']['slaveaddr'] # for eeprom (main i2c address)
        if self.busnum is None:
           self.busnum = SMBus(int(re.search("^i2c-(\d+)$",config['i2c']['smb']).group(1))) # set bus i2c-1
        if self.writestrobe is None:
           self.writestrobe = config['i2c']['eeprom']['writestrobe'] # hold pin low to write to eeprom

        c.close()

        with open("config/eepromfs.yaml") as d:
           try:
              toc_config = yaml.safe_load(d)
           except yaml.YAMLError as exc:
              print(exc)
              
        if self.TOC_start_address is None:
           self.TOC_start_address = toc_config['eeprom_fs']['TOC_start_address']
        self.TOC_version = toc_config['eeprom_fs']['TOC_version']  
        self.block_size = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['block_size'] 
        if self.chip_ic in ['24c04','24c08','24c16','24c32','24c64','24c128','24c256','24c512','24c1024','24Cc048']:
           self.toc_version = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_version']
        self.toc_FreeMemorySize = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_FreeMemorySize']
        self.toc_NumberOfFiles = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfFiles']
        if self.chip_ic in ['24c04','24c08','24c16','24c32','24c64','24c128','24c256','24c512','24c1024','24Cc048']:
           self.toc_NumberOfOrphanBlocks = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfOrphanBlocks']
           self.toc_NumberOfDeletedBlocks = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfDeletedBlocks']
           self.toc_NumberOfWriteBlocks = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfWriteBlocks']
        if self.chip_ic in ['24c64','24c128','24c256','24c512','24c1024','24c2048']:
           self.toc_WearLevelThreshold = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_WearLevelThreshold']
           self.toc_WearLevelCount = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_WearLevelCount']
        self.toc_FileList = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_FileList']
        if self.chip_ic in ['24c256','24c512','24c1024','24c2048']:
           self.toc_OwnerHashList = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_OwnerHashList']
        self.toc_crc = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_crc']
        self.toc_DataBlock = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_DataBlock']

        self.fh_filename = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_filename']
        self.fh_filetype = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_filetype']
        self.fh_FilsSize = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_FilsSize']
        self.fh_Attributes = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_Attributes']
        if self.chip_ic in ['24c16','24c32','24c64','24c128','24c256','24c512','24c1024','24c2048']:
          self.fh_NumberOfUsedBlocks = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_NumberOfUsedBlocks']
          self.fh_ModificationDate = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_ModificationDate']
          self.fh_CreationDate = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_CreationDate']

        d.close()

        rGPIO.setmode(rGPIO.BOARD)
        rGPIO.setwarnings(False)
        rGPIO.setup(self.writestrobe, rGPIO.OUT)
        self.mode = rGPIO.getmode()
        self.version = rGPIO.RPI_INFO

        pass

    def build_TOC(self):
        # Read TOC from EEPROM
        # If TOC does not exist, create new TOC
        # TOC should contain information about the location of the files, their sizes, and the address where they start
        self.toc_version # Version of TOC [2-3 byte], 24C04 and higher
        self.toc_FreeMemorySize # Free Memory Size, 24C01-24C02 [1 byte], 24C04 -24C128 [2 byte], higher [3 byte]
        self.toc_NumberOfFiles # Number of Active Files, 24C01 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_NumberOfOrphanBlocks # Number of Orphan Blocks, 24C04 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_NumberOfDeletedBlocks # Number of Deleted Blocks, 24C04 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_NumberOfWriteBlocks # Number of Writed Blocks, 24C04 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_WearLevelThreshold # TOC WL Threshold, only for 24C64 and higher, [ 2 byte ]
        self.toc_WearLevelCount # TOC WL Count, only for 24C64 and higher, [ 2 byte ]
        self.toc_FileList # List of files location address, relative from TOC end address, 24C01-24C02 [ 1 x 4 bytes], 24C04 [ 2 x 6 bytes]
        self.toc_OwnerHashList # List of approved Owner, default ['Admin','Admin'] -> [<user max 10 ASCII char>,<passwd hash max 20 ASCII char >] only for 24C254 and higher
        self.toc_crc # TOC CRC 24C01 - 24C16 [ 2 byte ], higher [ 4 byte ]
        self.toc_DataBlock # start of Data Block

        # TOC size
        #       FreeMemSize, NumofFiles, FileList,   CRC,           TOC_SUM      , Default Block Size
        # 24C01 [ 1 byte ], [ 1 byte ], [ 2(2) bytes], [ 2 byte ] => [ 6 bytes ], [ 2 bytes ]
        # 24C02 [ 1 byte ], [ 1 byte ], [ 3(3) bytes], [ 2 byte ] => [ 7 bytes ], [ 2 bytes ]
        #       Version,   FreeMemSize, NumOfFiles, NumOfOrp,   NumOfDel,  NumOfWrite, FileList,        CRC,          TOC_SUM      , Default Block Size
        # 24C04 [ 2 byte ], [ 2 byte ], [ 1 byte ], [ 1 byte ], [ 1 byte], [ 1 byte], [ 12(6) byte],  [ 2 byte] =>  [ 22 bytes ], [ 4 bytes ]
        # 24C08 [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte], [ 2 byte], [ 20(10) byte], [ 2 byte] =>  [ 34 bytes ], [ 4 bytes ]
        # 24C16 [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte], [ 2 byte], [ 36(18) byte], [ 2 byte] =>  [ 50 bytes ], [ 8 bytes ]
        # 24C32 [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte], [ 2 byte], [ 52(26) byte], [ 2 byte] =>  [ 66 bytes ], [ 8 bytes ]
        #          Version,   FreeMemSize, NumOfFiles, NumOfOrp,   NumOfDel,  NumOfWrite, WLThr,      WLCount,   FileList,         CRC,           TOS_SUM   , Default Block Size 
        # 24C64   [ 3 byte ], [ 3 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 1 byte], [ 1 byte], [ 108(36) byte], [ 2 byte] =>  [ 126 bytes ], [ 8 bytes ]
        # 24C128  [ 3 byte ], [ 3 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 2 byte ], [ 1 byte], [ 1 byte], [ 138(46) byte], [ 2 byte] =>  [ 156 bytes ], [ 16 bytes ]
        #          Version,   FreeMemSize, NumOfFiles, NumOfOrp,   NumOfDel,  NumOfWrite, WLThr,      WLCount,   FileList,       OwnHash,          CRC,         TOC_SUM,    Default Block Size   
        # 24C256  [ 2 byte ], [ 3 byte ], [ 2 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 2 byte], [ 1 byte], [ 174(58) byte], [ 120(4) byte],  [ 2 byte] =>  [ 316 bytes ], [ 16 bytes ]
        # 24C512  [ 3 byte ], [ 3 byte ], [ 2 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 2 byte], [ 1 byte], [ 210(70) byte], [ 180(6) byte],  [ 2 byte] =>  [ 412 bytes ], [ 16 bytes ]
        # 24C1024 [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 1 byte], [ 1 byte], [ 270(90) byte], [ 240(8) byte],  [ 2 byte] =>  [ 532 bytes ], [ 32 bytes ]
        # 24C2048 [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 1 byte], [ 1 byte], [ 360(120) byte], [ 300(10) byte],  [ 2 byte] =>  [ 682 bytes ], [ 40 bytes ]

        chip_list.init()

        self.toc_NumberOfFiles = 0
        self.toc_NumberOfOrphanBlocks = 0
        self.toc_NumberOfDeletedBlocks = 0
        self.toc_NumberOfWriteBlocks = 0

        if self.toc_version == 3 :
           self.toc_version_data = self.TOC_version
        elif self.toc_version == 2 :
           self.toc_version_data = self.TOC_version[1:]

        print (self.toc_version_data)

        if self.chip_ic == '24c01' :
           self.toc_FreeMemorySize_data = 0x7F - self.toc_DataBlock + 1
           data_content = hex_to_bytes(self.toc_FreeMemorySize_data) + hex_to_bytes(self.toc_NumberOfFiles) + hex_to_2bytes(0x0000)
           self.toc_crc_data = calculate_2byte_crc(data_content)
           data_crc = data_content + hex_to_2bytes(self.toc_crc_data)
           data_wipe = []
           for i in range(self.toc_FreeMemorySize_data) :
              data_wipe = data_wipe + [0x11]
           cmp = eeprom_write.writeNBytes(self.TOC_start_address, data_crc, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           cmp = eeprom_write.writeNBytes(self.toc_DataBlock, data_wipe, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

        if self.chip_ic == '24c256' :
           self.toc_FreeMemorySize_data = 0x7FFF - self.toc_DataBlock
           toc_FreeMemorySize_data = self.hex_to_2bytes(self.toc_FreeMemorySize_data)
           data = self.TOC_version_data + toc_FreeMemorySize_data + [0x01,0x02,0x03]
           cmp = eeprom_write.writeNBytes(self.TOC_start_address, data, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

        pass

    def check_TOC(self):
        # check consistency of the file system
        chip_list.init()

        self.toc_NumberOfFiles = 0
        self.toc_NumberOfOrphanBlocks = 0
        self.toc_NumberOfDeletedBlocks = 0
        self.toc_NumberOfWriteBlocks = 0

        if self.toc_version == 3 :
           self.toc_version_data = self.TOC_version
        elif self.toc_version == 2 :
           self.toc_version_data = self.TOC_version[1:]

        #print (self.toc_version_data)

        if self.chip_ic == '24c01' :
           self.toc_data = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.toc_FreeMemorySize_data = self.toc_data [0]
           self.toc_NumberOfFiles_data = self.toc_data [1]
           self.toc_FileList_data = self.toc_data [2:4]
           stored_crc = str(hex(self.toc_data[4])) + str(hex(self.toc_data[5]))[2:]
           calc_crc = str(hex(calculate_2byte_crc(self.toc_data [:4])))
           
           print("TOC_Data: ",self.toc_data)
           print("File List: ",self.toc_FileList_data)

           if (stored_crc == calc_crc) :
              #print ('TOC CRC16 is ok')
              self.error_code['CRC16'] = self.ERR_CRC16_OK
              return (list(self.error_code.values())[-1])

           #print ("Stored CRC: {:02x}{:02x}".format(self.toc_data[4],self.toc_data[5]))
           #print ("Calculated CRC: {:04x}".format(calculate_2byte_crc(self.toc_data [:4])))

        self.error_code['CRC16'] = self.ERR_CRC16_WRONG
        return (list(self.error_code.values())[-1])

    def read_TOC(self):
        if self.check_TOC() == self.ERR_CRC16_WRONG :
           print ("TOC is not consistent")
           self.error_code['read_TOC'] = self.ERR_TOC_INCOSISTENT
           return (list(self.error_code.values())[-1])

        print ("TOC is consistent")

        self.error_code['read_TOC'] = self.ERR_PASS
        return(list(self.error_code.values())[-1])

    def build_file_header(self, filename, filetype, FileSize, Option_RO = None) :
        self.fh_filename # Start Address of DataLocation
        self.fh_filetype # Type of filename
        self.fh_FilsSize # Free Memory Size
        self.fh_NumberOfUsedBlocks # NumberOfUsedBlocks
        self.fh_Attributes # File Attribute [Read Only],[Owner], shared with InUse [ 1 byte ]
        self.fh_InUse # True/False with Attributes shared
        self.fh_ModificationDate # Modification Data as TIMESTAMP from year 2023 (10 years) (FFF)-(1F)-(3F) (DAYS)-(HH)-(MM) => (DDDD DDDD DDDD DDDD DDDD DDDD)-(HHHH HHHH HXMM MMMM MMMM)b => 3 byte
        self.fh_CreationDate # Creation Data as TIMESTAMP from year 2023 (10 years) (FFF)-(1F)-(3F) (DAYS)-(HH)-(MM) => (DDDD DDDD DDDD DDDD DDDD DDDD)-(HHHH HHHH HXMM MMMM MMMM)b => 3 byte
        self.fh_crc # CRC, could be shared with Attributes and InUse

        self.read_TOC()
        if list(self.error_code.values())[-1] != self.ERR_PASS :
           self.error_code['build_file_header'] = self.ERR_PASS
           return (list(self.error_code.values())[-1])

        if self.chip_ic == '24c01' :
                
           if FileSize % self.block_size != 0 :
              FileSize = FileSize + 1

           if self.toc_FreeMemorySize_data < FileSize:
              self.error_code['build_file_header'] = self.ERR_MEMORY_IS_FULL
              return self.error_code

           self.fh_FilsSize = FileSize
           self.file_matrix_24c01()

           element = bytearray()
           for x in filename :
              element = element + binascii.hexlify(bytes(x.encode()))
              tmp = element.decode('ascii')
           self.fh_filename_data = int(tmp,16)
           envelope = hex_to_2bytes(self.fh_filename_data) + hex_to_bytes(filetype) + hex_to_bytes(FileSize) + hex_to_bytes(0x40) # set Attribut InUse = 1, ReadOnly = 0

           self.fh_crc_data = calculate_byte_crc(envelope)
           #print (self.fh_crc_data)

           self.fh_build_data = envelope + hex_to_bytes(self.fh_crc_data)

           print ("File Header: {}".format(self.fh_build_data))

           #cmp = eeprom_write.writeNBytes(self.TOC_start_address, data_crc, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           #cmp = eeprom_write.writeNBytes(self.toc_DataBlock, data_wipe, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

           self.error_code['build_file_header'] = self.ERR_PASS

        print("Error: {}".format(self.error_code))

        return (list(self.error_code.values())[-1], self.fh_build_data)

        # File header size
        #        Filename,  FileType,    FileSize,   Attribute    CRC           TOC_SUM
        # 24C01 [ 2 byte ], [ 1 byte ], [ 1 byte], [ 1 byte],   [1 byte]     => [ 6 bytes ]      [InUse,Read,X,X,X,X,X,X]
        # 24C02 [ 4 byte ], [ 1 byte ], [ 1 byte], [ 1 byte],               => [ 7 bytes ]      [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0]
        #        Filename,  FileType,    FileSize,   Attribute &   CRC           TOC_SUM
        # 24C04 [ 4 byte ], [ 1 byte ], [ 2 byte ], [ 1 byte],             => [ 8 bytes ]      [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0]
        # 24C08 [ 6 byte ], [ 1 byte ], [ 2 byte ], [ 1 byte],             => [ 10 bytes ]      [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0]
        #        Filename,  FileType,    FileSize,   NumOfUB, Attribute &   CRC, ModDate,    CreaDate           TOC_SUM
        # 24C16 [ 6 byte ], [ 1 byte ], [ 2 byte ], [2 byte], [ 1 byte],        [ 3 byte ], [ 3 byte ],     => [ 18 bytes ]      [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0]
        # 24C32 [ 8 byte ], [ 1 byte ], [ 2 byte ], [2 byte], [ 1 byte],        [ 3 byte ], [ 3 byte ],     => [ 20 bytes ]      [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0]
        #          Filename,    FileType,    FileSize,    NumOfUB,   Attribute &   CRC, ModDate,    CreaDate           TOC_SUM 
        # 24C64   [ 12 byte ], [ 1 byte ],   [ 3 byte ], [ 2 byte ], [ 2 byte ],       [ 3 byte ], [ 3 byte],  =>     [ 26 bytes], [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0] + CRC_H
        # 24C128  [ 16 byte ], [ 1 byte ],   [ 3 byte ], [ 3 byte ], [ 2 byte ],       [ 3 byte ], [ 3 byte],  =>     [ 30 bytes], [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0] + CRC_H
        #          Filename,    FileType,    FileSize,    NumOfUB,   Attribute &   CRC, ModDate,    CreaDate           TOC_SUM   
        # 24C256  [ 16 byte ], [ 1 byte ],   [ 3 byte ], [ 3 byte ], [ 2 byte ],       [ 3 byte ], [ 3 byte],  =>     [ 31 bytes], [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0] + CRC_H
        # 24C512  [ 16 byte ], [ 1 byte ],   [ 3 byte ], [ 3 byte ], [ 2 byte ],       [ 3 byte ], [ 3 byte],  =>     [ 31 bytes], [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0] + CRC_H
        # 24C1024 [ 20 byte ], [ 1 byte ],   [ 3 byte ], [ 3 byte ], [ 2 byte ],       [ 3 byte ], [ 3 byte],  =>     [ 35 bytes], [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0] + CRC_H
        # 24C2048 [ 20 byte ], [ 1 byte ],   [ 3 byte ], [ 3 byte ], [ 2 byte ],       [ 3 byte ], [ 3 byte],  =>     [ 35 bytes], [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0] + CRC_H

    def code_filetype(self, filetype):
        # add byte code to filetype, see into filetype.yaml
        with open("config/filetype.yaml") as c:
           try:
              ft_config = yaml.safe_load(c)
           except yaml.YAMLError as exc:
              print(exc)

        self.fh_filetype = ft_config['ft'][filetype]

        if self.fh_filetype is None:
           self.fh_filetype = 0xFF

        c.close()

        return(self.fh_filetype)

    def file_matrix_24c01(self):
        print("matrix: ",self.toc_FileList_data)
        
        for x in self.toc_FileList_data :
           if x != 0 :
              self.file_block.append(StartEnd(start = x, end = 0x30 ))
           else :
              self.file_db_address = self.toc_DataBlock + 0
              pass

        #print("matrix_data: {}". format(self.file_block[0].out()))
        #print("matrix_data: {}". format(self.file_block[1].out()))
        pass

    def add_file_to_TOC(self, file_name, file_size, file_type, start_address):
        # Add file information to TOC
        pass

    def delete_file_from_TOC(self, file_name):
        # delete file information from TOC
        pass

    def get_file_from_TOC(self, file_name):
        # get file information from TOC
        pass

    def create_file(self, file_name, file_size, file_type):
        # check if there is enough space on the chip
        # create entry in TOC for new file
        # write file data to chip
        pass

    def read_file(self, file_name):
        # locate file in TOC
        # read file data from chip
        pass

    def write_file(self, data):
        # locate file in TOC
        # write data to chip
        data_content = []
        for x in data :
           data_content.append(ord(x))

        cmp = eeprom_write.writeNBytes(self.file_db_address, data_content, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

        pass

    def delete_file(self, file_name):
        # locate file in TOC
        # mark file as deleted in TOC
        # free up space on chip
        pass

    def wear_leveling(self):
        # balance the number of writes across the chip
        # Move files around in the EEPROM to distribute writes evenly
        pass

    def update_file(self, file_name, data):
        # locate file in TOC
        # write data to chip
        self.wear_level_count += 1
        if self.wear_level_count > self.wear_level_threshold:
            self.wear_leveling()
            self.wear_level_count = 0

