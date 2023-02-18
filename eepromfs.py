import time
import sys, getopt
import yaml
import re
import binascii
import RPi.GPIO as rGPIO
from ecomet_i2c_sensors import i2c_command
from ecomet_i2c_sensors.eeprom import chip_list,eeprom_write,eeprom_read
from smbus2 import SMBus
from eeprom_math import hex_to_bytes,dec_to_list,hex_to_2bytes,hex_to_3bytes,hex_to_4bytes,calculate_byte_crc,calculate_2byte_crc,zero_to_bytes

#from i2c import i2c_io
class rawline(object) :
    def __init__ (self, line = "") :
        self.line = line
    def out(self) :
        return (self.line)

class StartEnd(object):
    def __init__ (self, start=None, end=None) :
       self.start = start
       self.end = end
       self._class_size = 1
       self._current_index = 0
    def __iter__(self):
       return self
    def __next__(self):
       if self._current_index <= self._class_size:
          if self._current_index == 0 :
             member = self.start
          elif self._current_index == 1 :
             member = self.end
          self._current_index += 1
          return member
       raise StopIteration
    def out(self) :
       return ([self.start,self.end])

class StartEndSize(object):
    def __init__ (self, start=None, end=None, size = 0) :
       self.start = start
       self.end = end
       self.size = end - start
       self._class_size = 1
       self._current_index = 0
    def __iter__(self):
       return self
    def __next__(self):
       if self._current_index <= self._class_size:
          if self._current_index == 0 :
             member = self.start
          elif self._current_index == 1 :
             member = self.end
          self._current_index += 1
          return member
       raise StopIteration
    def out(self) :
       return ([self.start,self.end,self.size])

class EEPROM_FS(object):

    # Error message
    PASS = 0x00
    CRC16_OK = 0x01
    FILE_WRITTEN = 0x02
    FILE_LOADED = 0x03
    FILE_FOUND = 0x04
    FILE_REMOVED = 0x05
    FILETYPE_FIND = 0x06
    DEFRAGMENT_OK = 0x07
    DEFRAGMENT_INGAP = 0x08
    DEFRAGMENT_MOVE = 0x09

    ERR_TOC_INCOSISTENT = 0xa0
    ERR_MEMORY_IS_FULL  = 0xa1
    ERR_CRC16_WRONG = 0xa2
    ERR_FILE_NOT_FOUND = 0xa3
    ERR_WRITE_FILE = 0xa4
    ERR_BUILD_FILE_HEADER = 0xa5
    ERR_WRONG_FILETYPE = 0xa6
    DEFRAGMENT_NOK = 0xa7

    def __init__(self, chip_ic=None, chip_address=None, busnum=None, writestrobe=None, TOC_start_address=None, i2c=None, **kwargs) :

        self.chip_ic = chip_ic    
        self.chip_address = chip_address
        self.busnum = busnum
        self.writestrobe = writestrobe
        self.block_size=None
        self.TOC_start_address = TOC_start_address
        self.toc_version = None # Version of TOC [2-3 byte], 24C04 and higher
        self.toc_FreeMemorySize = None # Free Memory Size, 24C01-24C02 [1 byte], 24C04 -24C128 [2 byte], higher [3 byte]
        self.toc_NumberOfFiles = None # Number of Active Files, 24C01 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_NumberOfOrphanBlocks = None # Number of Orphan Blocks, 24C04 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_NumberOfDeletedBlocks = None # Number of Deleted Blocks, 24C04 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_NumberOfWriteBlocks = None # Number of Writed Blocks, 24C04 - 24C16 [1 byte ], 24C32-24C512 [2 byte], higher [3 byte]
        self.toc_WearLevelThreshold = None # TOC WL Threshold, only for 24C64 and higher, [ 2 byte ]
        self.toc_WearLevelCount = None # TOC WL Count, only for 24C64 and higher, [ 2 byte ]
        self.toc_FileList = None # List of files location address, relative from TOC end address, 24C01-24C02 [ 1 x 4 bytes], 24C04 [ 2 x 6 bytes]
        self.toc_OwnerHashList = None # List of approved Owner, default ['Admin','Admin'] -> [<user max 10 ASCII char>,<passwd hash max 20 ASCII char >] only for 24C254 and higher
        self.toc_crc = None # TOC CRC 24C01 - 24C16 [ 2 byte ], higher [ 4 byte ]
        self.toc_DataBlock = None # start of Data Block

        self.fh_filename = None # Start Address of DataLocation
        self.fh_filetype = None # Type of filename
        self.fh_filesize = None # Free Memory Size
        self.fh_NumberOfUsedBlocks = None # NumberOfUsedBlocks
        self.fh_Attributes = None # File Attribute [Read Only],[Owner], shared with InUse [ 1 byte ]
        self.fh_InUse = None # True/False with Attributes shared
        self.fh_ModificationDate = None # Modification Data as TIMESTAMP from year 2023 (10 years) (FFF)-(1F)-(3F) (DAYS)-(HH)-(MM) => (DDDD DDDD DDDD DDDD DDDD DDDD)-(HHHH HHHH HXMM MMMM MMMM)b => 3 byte
        self.fh_CreationDate = None # Creation Data as TIMESTAMP from year 2023 (10 years) (FFF)-(1F)-(3F) (DAYS)-(HH)-(MM) => (DDDD DDDD DDDD DDDD DDDD DDDD)-(HHHH HHHH HXMM MMMM MMMM)b => 3 byte
        self.fh_CRC = None # CRC, could be shared with Attributes and InUse
        self.fh_Size = 0
        
        self.mem_size = None
        self.size_end_block = 0
        self.size_spread = 0

        self.fh_filename_data = 0x00
        self.fh_build_data = []

        self.toc_data_content = []
        self.fh_data_content = []
        self.data_content = []

        self.toc_version_data = None
        self.toc_FreeMemorySize_data = 0
        self.toc_FileList_data = []
        self.toc_NumberOfFiles = None
        self.toc_NumberOfOrphanBlocks = None
        self.toc_NumberOfDeletedBlocks = None
        self.toc_NumberOfWriteBlocks = None
        self.toc_crc_data = None
        
        self.fh_filename_data = ""
        self.fh_filetype_data = ""
        self.fh_filesize_data = 0
        self.fh_attribute_data = None
        self.fh_crc_data = 0

        self.file_db_address = 0x00
        self.file_db_address_list = []
        self.file_block = []
        self.file_gap = []
        self.file_data = ""

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

        chip_list.init()

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

        self.mem_size = toc_config['eeprom_fs']['Other'][self.chip_ic]['mem_size']
        self.fh_filename = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_filename']
        self.fh_filetype = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_filetype']
        self.fh_FileSize = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_FileSize']
        self.fh_Attributes = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_Attributes']
        self.fh_CRC = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_CRC']
        self.fh_Size = toc_config['eeprom_fs']['FH_attributes'][self.chip_ic]['fh_Size']
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

        # TOC size
        #       FreeMemSize, NumofFiles, FileList,   CRC,           TOC_SUM      , Default Block Size
        # 24C01 [ 1 byte ], [ 1 byte ], [ 3(3) bytes], [ 2 byte ] => [ 7 bytes ], [ 2 bytes ]
        # 24C02 [ 1 byte ], [ 1 byte ], [ 6(6) bytes], [ 2 byte ] => [ 10 bytes ], [ 2 bytes ]
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

        self.toc_NumberOfFiles_data = 0
        self.toc_NumberOfOrphanBlocks = 0
        self.toc_NumberOfDeletedBlocks = 0
        self.toc_NumberOfWriteBlocks = 0

        if self.toc_version == 3 :
           self.toc_version_data = self.TOC_version
        elif self.toc_version == 2 :
           self.toc_version_data = self.TOC_version[1:]

        #print (self.toc_version_data)

        self.toc_FreeMemorySize_data = self.mem_size - self.toc_DataBlock + 1

        if self.chip_ic in ('24c01','24c02') :
           self.toc_data_content = hex_to_bytes(self.toc_FreeMemorySize_data) + hex_to_bytes(self.toc_NumberOfFiles_data) + zero_to_bytes(self.toc_FileList[1] + 1)
           self.toc_crc_data = calculate_2byte_crc(self.toc_data_content)
           data_crc = self.toc_data_content + hex_to_2bytes(self.toc_crc_data)
           data_wipe = []
           for i in range(self.toc_FreeMemorySize_data) :
              data_wipe = data_wipe + [0x11]
           cmp = eeprom_write.writeNBytes(self.TOC_start_address, data_crc, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           cmp = eeprom_write.writeNBytes(self.toc_DataBlock, data_wipe, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

           self.toc_crc_data = calculate_2byte_crc(self.toc_data_content)
           data_crc = self.toc_data_content + hex_to_2bytes(self.toc_crc_data)
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

        self.toc_NumberOfFiles_data = 0
        self.toc_NumberOfOrphanBlocks = 0
        self.toc_NumberOfDeletedBlocks = 0
        self.toc_NumberOfWriteBlocks = 0

        if self.toc_version == 3 :
           self.toc_version_data = self.TOC_version
        elif self.toc_version == 2 :
           self.toc_version_data = self.TOC_version[1:]

        #print (self.toc_version_data)

        if self.chip_ic in ('24c01','24c02') :
           self.toc_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.toc_FreeMemorySize_data = self.toc_data_content [self.toc_FreeMemorySize[0]]
           self.toc_NumberOfFiles_data = self.toc_data_content [self.toc_NumberOfFiles[0]]
           self.toc_FileList_data = self.toc_data_content [self.toc_FileList[0]:self.toc_FileList[0]+self.toc_FileList[1]+1]
           stored_crc = str("{:02x}".format(self.toc_data_content[self.toc_crc[0]])) + str("{:02x}".format(self.toc_data_content[self.toc_crc[0] + self.toc_crc[1]]))
           calc_crc = str("{:04x}".format(calculate_2byte_crc(self.toc_data_content[:self.toc_crc[0]])))

           print("TOC_Data: ",self.toc_data_content)
           print("File List: ",self.toc_FileList_data)

           #print(stored_crc,calc_crc)
           if (stored_crc == calc_crc) :
              #print ('TOC CRC16 is ok')
              self.error_code['CRC16'] = self.CRC16_OK
              return (list(self.error_code.values())[-1])

           print ("Stored CRC: {:02x}{:02x}".format(self.toc_data_content[self.toc_crc[0]],self.toc_data_content[self.toc_crc[0] + self.toc_crc[1]]))
           print ("Calculated CRC: {:04x}".format(calculate_2byte_crc(self.toc_data_content[:self.toc_crc[0]])))

        self.error_code['CRC16'] = self.ERR_CRC16_WRONG
        return (list(self.error_code.values())[-1])

    def check_TOC_crc(self):
        if self.check_TOC() == self.ERR_CRC16_WRONG :
           print ("TOC is not consistent")
           self.error_code['check_TOC_crc'] = self.ERR_TOC_INCOSISTENT
           return (list(self.error_code.values())[-1])

        print ("TOC is consistent")

        self.error_code['check_TOC_crc'] = self.PASS
        return(list(self.error_code.values())[-1])

    def ls_eepromfs(self):

        self.toc_NumberOfFiles_data = 0
        self.toc_NumberOfOrphanBlocks = 0
        self.toc_NumberOfDeletedBlocks = 0
        self.toc_NumberOfWriteBlocks = 0
        eepromfs_list = []
        list_files = []
        list_freeSize = None
        list_freeFile = None
        file_cnt = 0

        if self.chip_ic in ('24c01','24c02') :
           self.check_TOC_crc()
           self.toc_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.toc_FreeMemorySize_data = self.toc_data_content [self.toc_FreeMemorySize[0]]
           self.toc_NumberOfFiles_data = self.toc_data_content [self.toc_NumberOfFiles[0]]
           self.toc_FileList_data = self.toc_data_content [self.toc_FileList[0]:self.toc_FileList[0]+self.toc_FileList[1]+1]
           stored_crc = str(hex(self.toc_data_content[self.toc_crc[0]])) + str(hex(self.toc_data_content[self.toc_crc[0] + self.toc_crc[1]]))[2:]
           for x in self.toc_FileList_data :
              if x != 0 :
                 file_cnt = file_cnt + 1
                 file_info = eeprom_read.readNBytes(self.TOC_start_address + x, self.TOC_start_address + x + self.fh_CRC[0] + self.fh_CRC[1], self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 filename = ""
                 filetype = ""
                 for y in file_info[:self.fh_filename[0] + self.fh_filename[1] + 1] :
                    filename = filename + chr(y)
                 filetype = self.code_filetype(file_info[self.fh_filetype[0]],1)
                 fileSize = file_info[self.fh_FileSize[0]]
                 print (file_info)
                 print ('fi:',file_info[self.fh_FileSize[0]])
                 #attribute = file_info[4]
                 list_files = rawline(str(fileSize) + "\t-rw " + filename + "." + filetype[0])
                 eepromfs_list.append(list_files.out())
              else :
                 break
        list_files = rawline("Free\t" + str(self.toc_FreeMemorySize_data) + "B Size")
        eepromfs_list.append(list_files.out())
        list_files = rawline("Free\t" + str(len(self.toc_FileList_data) - self.toc_NumberOfFiles_data) + "/" + str(len(self.toc_FileList_data)) + " File")
        eepromfs_list.append(list_files.out())

        # option
        self.file_matrix_24c01()

        self.size_spread = 0
        if self.file_gap != [] :
          self.size_end_block = self.file_gap[-1].out()[2]
          for x in range(len(self.file_gap)) :
             self.size_spread = self.size_spread + self.file_gap[x].out()[2]
        else :
          self.size_end_block = self.toc_FreeMemorySize_data
          self.size_spread = 0

        print("Size_End_Block: ",hex(self.size_end_block))
        print("Size_Spread: ",hex(self.size_spread))

        return (eepromfs_list)

    def add_file_to_TOC(self):
        # Add file information to TOC
        if self.chip_ic in ('24c01','24c02') :
           idx = 0
           for x in self.toc_FileList_data :
              if x == 0 :
                 print ("FreeMemorySize: ",self.toc_FreeMemorySize_data - self.fh_filesize_data - self.fh_CRC[0])
                 self.toc_FreeMemorySize_data = self.toc_FreeMemorySize_data - self.fh_filesize_data - self.fh_CRC[0] 
                 self.toc_NumberOfFiles_data = self.toc_NumberOfFiles_data + 1
                 self.toc_FileList_data[idx] = self.file_db_address
                 break
              idx = idx + 1
           print ("--> sync_TOC")
           self.sync_TOC()
        pass

    def sync_TOC(self):
        # Synchronize changes in TOC
        if self.chip_ic in ('24c01','24c02') :
           toc_data_content_read = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           toc_FreeMemorySize_data_read = self.toc_data_content [self.toc_FreeMemorySize[0]]
           if toc_FreeMemorySize_data_read != self.toc_FreeMemorySize_data :
                   cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FreeMemorySize[0], hex_to_bytes(self.toc_FreeMemorySize_data), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                   print("FreeMemorySize needs to sync")
           toc_NumberOfFiles_data_read = self.toc_data_content [self.toc_NumberOfFiles[0]]
           if toc_NumberOfFiles_data_read != self.toc_NumberOfFiles_data :
                   cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_NumberOfFiles[0], hex_to_bytes(self.toc_NumberOfFiles_data), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                   print("NumberOfFiles needs to sync")
           toc_FileList_data_read = self.toc_data_content [self.toc_FileList[0]:self.toc_FileList[0] + self.toc_FileList[1] + 1]
           if toc_FileList_data_read != self.toc_FileList_data :
                   idx = self.toc_FileList[0]
                   for x in self.toc_FileList_data :
                      cmp = eeprom_write.writeNBytes(self.TOC_start_address + idx, hex_to_bytes(x), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                      print("FileList[{}] needs to sync ..{:02x}".format(idx,x))
                      idx = idx + 1
           sync_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           calc_crc = calculate_2byte_crc(sync_data_content[:self.toc_crc[0]])
           print("calc_crc: {:04x}". format(calc_crc))
           cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_crc[0], hex_to_2bytes(calc_crc), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           print ("sync written CRC")
        pass

    def build_file_header(self, filename, filetype, FileSize, Option_RO = None) :

        if self.check_TOC_crc() != self.PASS:
           self.error_code['build_file_header'] = self.ERR_CRC16_WRONG
           return (list(self.error_code.values())[-1],None)

        if self.chip_ic in ('24c01','24c02') :
                
           #if FileSize % self.block_size != 0 :
           #   FileSize = FileSize + 1

           if self.toc_FreeMemorySize_data < FileSize:
              self.error_code['build_file_header'] = self.ERR_MEMORY_IS_FULL
              return (list(self.error_code.values())[-1], None)

           self.fh_filesize_data = FileSize
           print("File size: ",dec_to_list(self.fh_filesize_data))

           element = bytearray()
           print(filename)
           for x in filename :
              element = element + binascii.hexlify(bytes(x.encode()))
              tmp = element.decode('ascii')
           self.fh_filename_data = int(tmp,16)
           if self.chip_ic in ('24c01') :
              envelope = hex_to_2bytes(self.fh_filename_data) + hex_to_bytes(filetype) + dec_to_list(self.fh_filesize_data) + hex_to_bytes(0x40) # set Attribut InUse = 1, ReadOnly = 0
           elif self.chip_ic in ('24c02') :
              if filetype[0] == '' :
                 self.error_code['build_file_header'] = ERR_WRONG_FILETYPE
                 return (list(self.error_code.values())[-1], '')

              envelope = hex_to_4bytes(self.fh_filename_data) + hex_to_bytes(filetype[0]) + dec_to_list(self.fh_filesize_data) + hex_to_bytes(0x40) # set Attribut InUse = 1, ReadOnly = 0

           print ("Envelope: ",envelope)
           self.fh_crc_data = calculate_byte_crc(envelope)
           #print (self.fh_crc_data)

           self.fh_build_data.extend(envelope)
           self.fh_build_data.extend(hex_to_bytes(self.fh_crc_data))

           print ("File Header: {}".format(self.fh_build_data))

           #cmp = eeprom_write.writeNBytes(self.TOC_start_address, data_crc, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           #cmp = eeprom_write.writeNBytes(self.toc_DataBlock, data_wipe, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

           self.error_code['build_file_header'] = self.PASS

        return (list(self.error_code.values())[-1], self.fh_build_data)

    def read_file_header(self,fh_address) :

        print("fh_address: ",fh_address)
        if self.chip_ic in ('24c01','24c02') :
           return(eeprom_read.readNBytes(fh_address, fh_address + self.fh_CRC[0] + 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic))

        # File header size
        #        Filename,  FileType,    FileSize,   Attribute    CRC           TOC_SUM
        # 24C01 [ 2 byte ], [ 1 byte ], [ 1 byte], [ 1 byte],   [1 byte]     => [ 6 bytes ]      [InUse,Read,X,X,X,X,X,X]
        # 24C02 [ 4 byte ], [ 1 byte ], [ 1 byte], [ 1 byte],   [1 byte]     => [ 8 bytes ]      [InUse,Read,X,X,X,X,X,X]
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

    def code_filetype(self, filetype, reverse = None):
        # add byte code to filetype, see into filetype.yaml
        with open("config/filetype.yaml") as c:
           try:
              ft_config = yaml.safe_load(c)
           except yaml.YAMLError as exc:
              print(exc)

        if reverse == None :
           self.fh_filetype_data = ft_config['ft'][filetype]
           if self.fh_filetype_data is None:
              self.fh_filetype_data = 0xFF
        else :
           try :
              self.fh_filetype_data = ft_config['rft'][filetype]
           except:
              self.error_code['code_filetype'] = self.ERR_WRONG_FILETYPE
              return ('',list(self.error_code.values())[-1])
           if self.fh_filetype_data is None:
              self.fh_filetype_data = '.txt'

        c.close()
        self.error_code['code_filetype'] = self.FILETYPE_FIND
        return(self.fh_filetype_data,list(self.error_code.values())[-1])

    def file_matrix_24c01(self):
        print("matrix: ",self.toc_FileList_data)
        fh = []
        self.file_block.clear()
        self.file_gap.clear()
        
        for x in self.toc_FileList_data :
           if x != 0 :
              fh = self.read_file_header(x)
              self.file_block.append(StartEnd(start = x, end = x + fh[self.fh_FileSize[0]] + self.fh_CRC[0] - 1))   # end Start ob block data + file size + size of fh
           else :
              if self.file_block != []:
                 print("Last Addr in Matrix: [{}]". format(','.join(hex(x) for x in self.file_block[-1].out())))
                 self.file_db_address = self.file_block[-1].out()[1] + 1
              else :
                 self.file_db_address = self.toc_DataBlock + 0
              pass

        #self.file_gap.append(startEnd())
        start = self.TOC_start_address + self.toc_DataBlock - 1
        end = self.mem_size
        skip = 0
        v_start = start
        v_end = end

        x = 0
        while( x < len(self.file_block)) :
           if skip == 0 :
             (y_start,y_end) = self.file_block[x]
           first = 0
           for y in range (v_start,v_end) :
              if y <= y_start :
                 if first == 0 :
                    #print ("address1: {}".format(hex(y)))
                    low = y
                    first = 1
              else :
                 #print ("end address {}".format(hex(y-1)))
                 high  = y - 1
                 v_start = y_end
                 break
           self.file_gap.append(StartEndSize(start = low, end = high, size = (high - low)))
           #print("Gap in Matrix: [{}]". format(','.join(hex(x) for x in self.file_gap[-1].out())))
           try :
              x = x + 1
              (y_start,y_end) = self.file_block[x]
              skip = 1
           except :
              skip = 0
              v_start = y_end + 1
              v_end = self.mem_size
              self.file_gap.append(StartEndSize(start = v_start, end = v_end, size = v_start - v_end))
              break

        if self.file_block :
           for x in range(len(self.file_block)) :
              print("matrix_data: [{}]". format(','.join(hex(x) for x in self.file_block[x].out())))
              
        if self.file_gap :
           for x in range(len(self.file_gap)) :
              print("matrix_gap: [{}]". format(','.join(hex(x) for x in self.file_gap[x].out())))

        pass

    def defragment_matrix(self, data_content):
        sum = 0
        first = 0
        idx = 0
        move_idx = 0
        first_idx = 0
        first_addr = 0
        print ("Data Content: ", data_content)
        for x in self.file_gap :
           print ("SUM: {}:{}".format(hex(sum), hex(len(data_content))))
           if sum >= len(data_content) :
              print("Final idx: ", idx)
              break
           if x.out()[2] == 1 :
              move_idx = move_idx + 1
              print ("move_idx: ", move_idx)
           else :
              if first == 0 :
                 first = 1
                 first_idx = idx
                 first_addr = x.out()[0] + 1
              sum = sum + x.out()[2]
           idx = idx + 1
        print ("SUM: {}:{}".format(hex(sum), hex(len(data_content) )))
        print ("IDX_LIST: {}:{}:{} Addr: {}".format(idx, move_idx, first_idx, hex(first_addr)))
        if move_idx == 1 :
           self.file_db_address = self.file_gap[idx].out()[0] + 1
           print ("Just addr new file in GAP, move FileList in TOC")
           print ("Calc Addr: ",hex(self.file_db_address))
           print ("File Data Content: ", data_content)
           cmp = eeprom_write.writeNBytes(self.file_db_address, data_content, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           print ("Before FileList: [{}]". format(','.join(hex(z) for z in self.toc_FileList_data)))
           sub_content = [self.file_db_address]
           write_address = idx
           for y in self.toc_FileList_data[idx:] :
              print ("idx: ",idx)
              if y != 0 and idx <= self.toc_DataBlock - 2:
                 sub_content.append(y)
                 #print (y)
                 idx = idx + 1
              else :
                 break
           print ("Sub content: [{}]". format(','.join(hex(z) for z in sub_content)))
           self.toc_FreeMemorySize_data = self.toc_FreeMemorySize_data - len(data_content)
           self.toc_NumberOfFiles_data = self.toc_NumberOfFiles_data + 1
           cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FreeMemorySize[0], hex_to_bytes(self.toc_FreeMemorySize_data) + hex_to_bytes(self.toc_NumberOfFiles_data), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FileList[0] + write_address, sub_content, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.sync_TOC()
           self.error_code['defragment_matrix'] = self.DEFRAGMENT_INGAP
        elif sum >= len(data_content):
           self.file_db_address = first_addr
           print ("Before FileList: [{}]". format(','.join(hex(z) for z in self.toc_FileList_data)))
           print ("Move blocks and then add File")
           gap_move = []
           for x in range(first_idx,idx - 1) :
              gap_move.append(self.file_block[x].out())
           print ("Calc Addr: ",hex(self.file_db_address))
           for x in gap_move :
              #print ("Move gap: {}:{}". format(hex(x[0]),hex(x[1])))
              move_block = eeprom_read.readNBytes(x[0], x[1] + 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
              print ("Moved block: [{}]". format(','.join(hex(z) for z in move_block)))
              cmp = eeprom_write.writeNBytes(self.file_db_address, move_block, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
              #print ("Read Data: {}".format(hex(eeprom_read.readNBytes(self.TOC_start_address + self.toc_FileList[0] + first_idx, self.TOC_start_address + self.toc_FileList[0] + first_idx, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)[0])))
              self.toc_FileList_data[first_idx] = self.file_db_address
              print ("List address[{}] = {}".format(hex(self.TOC_start_address + self.toc_FileList[0] + first_idx),hex(self.file_db_address)))
              first_idx = first_idx + 1
              self.file_db_address = self.file_db_address + len(move_block) - 1
           self.toc_FileList_data[first_idx] = self.file_db_address
           cmp = eeprom_write.writeNBytes(self.file_db_address, data_content, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           print ("New File data content: [{}]". format(','.join(hex(z) for z in data_content)))
           print ("Read Data1: {}".format(eeprom_read.readNBytes(self.file_db_address, self.file_db_address + len(data_content), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)))
           cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FileList[0], self.toc_FileList_data, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           print ("After FileList: [{}]". format(','.join(hex(z) for z in self.toc_FileList_data)))
           self.toc_FreeMemorySize_data = self.toc_FreeMemorySize_data - len(data_content)
           self.toc_NumberOfFiles_data = self.toc_NumberOfFiles_data + 1
           cmp = eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FreeMemorySize[0], hex_to_bytes(self.toc_FreeMemorySize_data) + hex_to_bytes(self.toc_NumberOfFiles_data), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           #print ("Read Data1: {}".format(hex(eeprom_read.readNBytes(self.file_db_address, self.file_db_address + len(data_content), self.busnum,self.chip_address,self.writestrobe,self.chip_ic))))
           self.sync_TOC()
           #print ("Read Data2: {}".format(hex(eeprom_read.readNBytes(self.file_db_address, self.file_db_address + len(data_content), self.busnum,self.chip_address,self.writestrobe,self.chip_ic))))
           self.error_code['defragment_matrix'] = self.DEFRAGMENT_MOVE
        else :
           print ("Compare: ",sum," vs. ",len(data_content))
           print ("Not enough memory")
           self.error_code['defragment_matrix'] = self.ERR_MEMORY_IS_FULL
           return(list(self.error_code.values())[-1])
        self.error_code['defragment_matrix'] = self.FILE_WRITTEN
        return(list(self.error_code.values())[-1])

    def remove_file_from_TOC(self,offset):
        # delete file information from TOC
        idx = 0
        vshift = 0
        if self.chip_ic in ('24c01','24c02') :
           print("remove_file_from_TOC")
           self.get_file_from_TOC()
           self.toc_NumberOfFiles_data = self.toc_data_content [self.toc_NumberOfFiles[0]]
           self.toc_FreeMemorySize_data = self.toc_data_content [self.toc_FreeMemorySize[0]]
           for x in self.file_db_address_list :
              if (idx == offset or vshift == 1) :
                 vshift = 1
                 print ("address: ",idx,x)
                 if (idx > offset) :
                    print("shift <-: ", x)
                    eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FileList[0] + (idx - 1), hex_to_bytes(x), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                    print("set to 0", x)
                    eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FileList[0] + idx, hex_to_bytes(0x00), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 elif (idx == offset) :
                    print("set to 0", x)
                    eeprom_write.writeNBytes(self.TOC_start_address + self.toc_FileList[0] + idx, hex_to_bytes(0x00), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                    self.toc_NumberOfFiles_data = self.toc_NumberOfFiles_data - 1
                    #print (self.toc_FreeMemorySize_data, self.fh_filesize_data)
                    self.toc_FreeMemorySize_data = self.toc_FreeMemorySize_data + self.fh_filesize_data + self.fh_CRC[0] + 1
                 elif ( x == 0 ) :
                    break
                 idx = idx + 1
              else :
                 idx = idx + 1

           self.sync_TOC()
        pass

    def get_file_from_TOC(self):
        # get file information from TOC
        if self.chip_ic in ('24c01','24c02') :
           self.file_db_address_list  = eeprom_read.readNBytes(self.TOC_start_address + self.toc_FileList[0], self.TOC_start_address + self.toc_FileList[0] + self.toc_FileList[1], self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.toc_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
        pass

    def create_file(self, file_name, file_size, file_type):
        # check if there is enough space on the chip
        # create entry in TOC for new file
        # write file data to chip
        pass

    def write_file(self, data):
        # locate file in TOC
        # write data to chip
        data_content = []
        data_crc = []
        c = 1
        print('wf:',self.fh_build_data)
        for x in self.fh_build_data :
           data_content.append(x)
           if c <= (len(self.fh_build_data) - 2) :   #CRC & Attribute byte are excluded
              data_crc.append(x)
           c = c + 1
        print('wf2:',data_content)
        for x in data :
           data_content.append(ord(x))
           data_crc.append(ord(x))
        print('wf3:',data_content)

        #if len(data_content) % 2 != 0:
        #   print("write_file append 0x00")
        #   data_content.append(0x00)
        #   data_crc.append(0x00)

        #print("Data CRC: ",data_crc)
        print("Data CRC out: ",calculate_byte_crc(data_crc))
        data_content[self.fh_CRC[0]] = calculate_byte_crc(data_crc)

        self.file_matrix_24c01()

        if self.file_gap != [] :
          self.size_end_block = self.file_gap[-1].out()[2]
          for x in range(len(self.file_gap)) :
             self.size_spread = self.size_spread + self.file_gap[x].out()[2]
        else :
          self.size_end_block = self.toc_FreeMemorySize_data
          self.size_spread = 0

        calc_write_bytes = len(data_content)

        print("Size_End_Block: ",hex(self.size_end_block))
        print("Size_Spread: ",hex(self.size_spread))
        print("Calc_WriteBytes: ", hex(calc_write_bytes))

        if calc_write_bytes > self.size_end_block :
           if calc_write_bytes <= self.size_spread :
              if self.defragment_matrix(data_content) >= 0xa0 :
                 self.error_code['write_file'] = self.DEFRAGMENT_NOK
                 return(list(self.error_code.values())[-1])
              else :
                 self.error_code['write_file'] = self.DEFRAGMENT_OK
                 return(list(self.error_code.values())[-1])
           else : self.error_code['write_file'] = self.ERR_MEMORY_IS_FULL

        self.add_file_to_TOC()

        print("Data Content: ",data_content)
        cmp = eeprom_write.writeNBytes(self.file_db_address, data_content, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

        self.error_code['write_file'] = self.FILE_WRITTEN
        return(list(self.error_code.values())[-1])
        
    def load_file(self, filename, filetype) :
        self.fh_filename_data = self.get_file_from_TOC()
        file_found = 0
           
        if self.chip_ic in ('24c01','24c02') :
           for x in self.file_db_address_list :
              if x != 0 :
                 self.fh_data_content = eeprom_read.readNBytes(x, x + self.fh_CRC[0] + self.fh_CRC[1], self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 self.fh_filename_data = ""
                 for y in self.fh_data_content [:self.fh_filename[0] + self.fh_filename[1] + 1] :
                    if y != 0x00 :
                       self.fh_filename_data = self.fh_filename_data + chr(y)
                 self.fh_filetype_data = self.code_filetype(self.fh_data_content [self.fh_filetype[0]],1)
                 if (filename == self.fh_filename_data and filetype == self.fh_filetype_data[0]) :
                    file_found = 1
                    self.fh_filesize_data = self.fh_data_content [self.fh_FileSize[0]]
                    self.fh_attribute_data = self.fh_data_content [self.fh_Attributes[0]]
                    self.fh_crc_data = self.fh_data_content [self.fh_CRC[0]]
                    for z in eeprom_read.readNBytes(x + self.fh_CRC[0] + 1, x + self.fh_CRC[0] + 1 + self.fh_filesize_data -1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic):
                       #print(z,chr(z))
                       self.file_data = self.file_data + chr(z)
                    #print ("load_file: >",self.file_data,"<")
              else :
                 break
        
              
        if file_found == 0 :
           self.error_code['load_file'] = self.ERR_FILE_NOT_FOUND
        else :
           self.error_code['load_file'] = self.FILE_FOUND
        return (list(self.error_code.values())[-1])

    def remove_file(self, filename, filetype) :
        self.fh_filename_data = self.get_file_from_TOC()
        file_found = 0
        idx = 0
           
        if self.chip_ic in ('24c01','24c02') :
           for x in self.file_db_address_list :
              if x != 0 :
                 self.fh_data_content = eeprom_read.readNBytes(x, x + self.fh_CRC[0] + self.fh_CRC[1], self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 self.fh_filename_data = ""
                 for y in self.fh_data_content [:self.fh_filename[0] + self.fh_filename[1] + 1] :
                    if y != 0x00 :
                       self.fh_filename_data = self.fh_filename_data + chr(y)
                 self.fh_filetype_data = self.code_filetype(self.fh_data_content [self.fh_filetype[0]],1)
                 #print (self.fh_filename_data,".",self.fh_filetype_data[0])
                 if (filename == self.fh_filename_data and filetype == self.fh_filetype_data[0]) :
                    file_found = 1
                    self.fh_filesize_data = self.fh_data_content [self.fh_FileSize[0]]
                    self.fh_attribute_data = self.fh_data_content [self.fh_Attributes[0]]
                    self.fh_crc_data = self.fh_data_content [self.fh_CRC[0]]
                    idx_left = idx
                 else :
                    idx = idx + 1
              else :
                 break

        if file_found == 0 :
           self.error_code['remove_file'] = self.ERR_FILE_NOT_FOUND
        else :
           print ("set attribute InUse = disable idx: ",idx_left)
           self.remove_file_from_TOC(idx_left)
           self.error_code['remove_file'] = self.FILE_REMOVED
        return (list(self.error_code.values())[-1])


    def write_eepromfs(self,filename) :

        FileSize = 0
        rc = ()
        raw_line = list()

        f = open(filename,"r")
        file_data = ""
        while True :
           raw_line = ""
           try:
              raw_line = f.readline()
           except :
              self.error_code['write_eepromfs'] = self.ERR_FILE_NOT_FOUND

           if not raw_line:
              break
        
           file_data = file_data + raw_line
           #print (file_data)

           FileSize = FileSize + len(raw_line)

        f.close()

        tmp = filename
        pattern = '\/?(\w*)\.(.*)$'
        match = re.search(pattern,tmp)
        filename = match.group(1)
        filetype = self.code_filetype(match.group(2))
        print ("FileName: {}".format(filename))
        print ("FileType: {:02x}".format(filetype[0]))
        print ("FileSize: {}".format(FileSize))

        rc = self.build_file_header(filename, filetype, FileSize, Option_RO = False)
        if rc[0] > 0xa0 :
           self.error_code['write_eepromfs'] = self.ERR_BUILD_FILE_HEADER
           return (list(self.error_code.values())[-1])
        #print (file_data)
        if self.write_file(file_data) > 0xa0 :
           self.error_code['write_eepromfs'] = self.ERR_BUILD_FILE_HEADER
           return (list(self.error_code.values())[-1])
        self.error_code['write_eepromfs'] = self.FILE_WRITTEN

        return (list(self.error_code.values())[-1])

    def load_eepromfs(self, file_name) :
        raw_line = ""

        pattern = '\/{0,1}(.*)\.(.*)$'
        match = re.search(pattern,file_name)
        filename = match.group(1)
        filetype = match.group(2)
        print ("FileName: {}".format(filename))
        print ("FileType: {}".format(filetype))

        if self.load_file(filename, filetype) >= 0xa0 :
           self.error_code['load_eepromfs'] = self.ERR_FILE_NOT_FOUND
           return (list(self.error_code.values())[-1])
        else :
           self.error_code['load_eepromfs'] = self.FILE_FOUND

        f = open(file_name,"w")
        while True :
           try:
              raw_line = raw_line + f.writelines(self.file_data)
           except :
              self.error_code['load_eepromfs'] = self.ERR_WRITE_FILE
           if not raw_line :
              break


        FileSize = self.fh_filesize_data

        f.close()

        print ("FileSize: {}".format(FileSize))
        
        self.error_code['load_eepromfs'] = self.FILE_LOADED 
        return (list(self.error_code.values())[-1])
        
    def remove_eepromfs(self, file_name) :
        raw_line = ""

        pattern = '^(.*)\.(.*)$'
        match = re.search(pattern,file_name)
        filename = match.group(1)
        filetype = match.group(2)
        print ("FileName: {}".format(filename))
        print ("FileType: {}".format(filetype))

        self.remove_file(filename, filetype)
        
        self.error_code['remove_eepromfs'] = self.FILE_REMOVED 
        return (list(self.error_code.values())[-1])

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

