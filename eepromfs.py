import time
import sys, getopt
import yaml
import re
import binascii
import RPi.GPIO as rGPIO
from ecomet_i2c_sensors import i2c_command
from ecomet_i2c_sensors.eeprom import chip_list,eeprom_write,eeprom_read
from smbus2 import SMBus
from eeprom_math import hex_to_bytes,dec_to_list,hex_to_2bytes,hex_to_3bytes,calculate_byte_crc,calculate_2byte_crc

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

    ERR_TOC_INCOSISTENT = 0x10
    ERR_MEMORY_IS_FULL  = 0x11
    ERR_CRC16_WRONG = 0x12
    ERR_FILE_NOT_FOUND = 0x13
    ERR_WRITE_FILE = 0x14
    ERR_BUILD_FILE_HEADER = 0x15

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
        self.fh_crc = None # CRC, could be shared with Attributes and InUse

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
           self.toc_FreeMemorySize_data = 0x7F - self.toc_DataBlock + 1
           self.toc_data_content = hex_to_bytes(self.toc_FreeMemorySize_data) + hex_to_bytes(self.toc_NumberOfFiles) + hex_to_2bytes(0x0000)
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
           self.toc_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.toc_FreeMemorySize_data = self.toc_data_content [0]
           self.toc_NumberOfFiles_data = self.toc_data_content [1]
           self.toc_FileList_data = self.toc_data_content [2:4]
           stored_crc = "0x" + str("{:02x}".format(self.toc_data_content[4])) + str("{:02x}".format(self.toc_data_content[5]))#[2:]
           calc_crc = str(hex(calculate_2byte_crc(self.toc_data_content[:4])))
           
           print("TOC_Data: ",self.toc_data_content)
           print("File List: ",self.toc_FileList_data)

           #print(stored_crc,calc_crc)
           if (stored_crc == calc_crc) :
              #print ('TOC CRC16 is ok')
              self.error_code['CRC16'] = self.CRC16_OK
              return (list(self.error_code.values())[-1])

           print ("Stored CRC: {:02x}{:02x}".format(self.toc_data_content[4],self.toc_data_content[5]))
           print ("Calculated CRC: {:04x}".format(calculate_2byte_crc(self.toc_data_content[:4])))

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

        self.toc_NumberOfFiles = 0
        self.toc_NumberOfOrphanBlocks = 0
        self.toc_NumberOfDeletedBlocks = 0
        self.toc_NumberOfWriteBlocks = 0
        eepromfs_list = []
        list_files = []
        list_freeSize = None
        list_freeFile = None
        file_cnt = 0

        if self.chip_ic == '24c01' :
           self.check_TOC_crc()
           self.toc_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           self.toc_FreeMemorySize_data = self.toc_data_content [0]
           self.toc_NumberOfFiles_data = self.toc_data_content [1]
           self.toc_FileList_data = self.toc_data_content [2:4]
           stored_crc = str(hex(self.toc_data_content[4])) + str(hex(self.toc_data_content[5]))[2:]
           for x in self.toc_FileList_data :
              if x != 0 :
                 file_cnt = file_cnt + 1
                 file_info = eeprom_read.readNBytes(self.TOC_start_address + x, self.TOC_start_address + x + 6, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 filename = ""
                 filetype = ""
                 for y in file_info[:2] :
                    filename = filename + chr(y)
                 filetype = self.code_filetype(file_info[2],1)
                 fileSize = file_info[3]
                 #attribute = file_info[4]
                 list_files = rawline(str(fileSize) + "\t-rw " + filename + "." + filetype)
                 eepromfs_list.append(list_files.out())
              else :
                 break
        list_files = rawline("Free\t" + str(self.toc_FreeMemorySize_data) + "B Size")
        eepromfs_list.append(list_files.out())
        list_files = rawline("Free\t" + str(self.toc_FileList-file_cnt) + "/" + str(self.toc_FileList) + " File")
        eepromfs_list.append(list_files.out())

        return (eepromfs_list)

    def add_file_to_TOC(self):
        # Add file information to TOC
        if self.chip_ic == '24c01' :
           idx = 0
           for x in self.toc_FileList_data :
              if x == 0 :
                 print ("FreeMemorySize: ",self.toc_FreeMemorySize_data - self.fh_filesize_data - 6)
                 self.toc_FreeMemorySize_data = self.toc_FreeMemorySize_data - self.fh_filesize_data - 6 
                 self.toc_NumberOfFiles_data = self.toc_NumberOfFiles_data + 1
                 self.toc_FileList_data[idx] = self.file_db_address
                 break
              idx = idx + 1
           print ("--> sync_TOC")
           self.sync_TOC()
        pass

    def sync_TOC(self):
        # Synchronize changes in TOC
        if self.chip_ic == '24c01' :
           toc_data_content_read = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           toc_FreeMemorySize_data_read = self.toc_data_content [0]
           if toc_FreeMemorySize_data_read != self.toc_FreeMemorySize_data :
                   cmp = eeprom_write.writeNBytes(self.TOC_start_address, hex_to_bytes(self.toc_FreeMemorySize_data), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                   print("FreeMemorySize needs to sync")
           toc_NumberOfFiles_data_read = self.toc_data_content [1]
           if toc_NumberOfFiles_data_read != self.toc_NumberOfFiles_data :
                   cmp = eeprom_write.writeNBytes(self.TOC_start_address + 1, hex_to_bytes(self.toc_NumberOfFiles_data), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                   print("NumberOfFiles needs to sync")
           toc_FileList_data_read = self.toc_data_content [2:4]
           if toc_FileList_data_read != self.toc_FileList_data :
                   idx = 2
                   for x in self.toc_FileList_data :
                      cmp = eeprom_write.writeNBytes(self.TOC_start_address + idx, hex_to_bytes(x), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                      print("FileList[{}] needs to sync ..{:02x}".format(idx,x))
                      idx = idx + 1
           sync_data_content = eeprom_read.readNBytes(self.TOC_start_address, self.toc_DataBlock - 1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           calc_crc = calculate_2byte_crc(sync_data_content[:4])
           print("calc_crc: {:04x}". format(calc_crc))
           cmp = eeprom_write.writeNBytes(self.TOC_start_address + 4, hex_to_2bytes(calc_crc), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
           print ("sync written CRC")
        pass

    def build_file_header(self, filename, filetype, FileSize, Option_RO = None) :

        if self.check_TOC_crc() != self.PASS:
           self.error_code['build_file_header'] = self.ERR_CRC16_WRONG
           return (list(self.error_code.values())[-1],None)

        if self.chip_ic == '24c01' :
                
           if FileSize % self.block_size != 0 :
              FileSize = FileSize + 1

           if self.toc_FreeMemorySize_data < FileSize:
              self.error_code['build_file_header'] = self.ERR_MEMORY_IS_FULL
              return (list(self.error_code.values())[-1], None)

           self.fh_filesize_data = FileSize
           print("File size: ",dec_to_list(self.fh_filesize_data))

           element = bytearray()
           for x in filename :
              element = element + binascii.hexlify(bytes(x.encode()))
              tmp = element.decode('ascii')
           self.fh_filename_data = int(tmp,16)
           envelope = hex_to_2bytes(self.fh_filename_data) + hex_to_bytes(filetype) + dec_to_list(self.fh_filesize_data) + hex_to_bytes(0x40) # set Attribut InUse = 1, ReadOnly = 0

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
        if self.chip_ic == '24c01' :
           return(eeprom_read.readNBytes(fh_address, fh_address + 5, self.busnum,self.chip_address,self.writestrobe,self.chip_ic))

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

    def code_filetype(self, filetype, reverse = None):
        # add byte code to filetype, see into filetype.yaml
        with open("config/filetype.yaml") as c:
           try:
              ft_config = yaml.safe_load(c)
           except yaml.YAMLError as exc:
              print(exc)

        if reverse == None :
           self.fh_filetype = ft_config['ft'][filetype]
           if self.fh_filetype is None:
              self.fh_filetype = 0xFF
        else :
           self.fh_filetype = ft_config['rft'][filetype]
           if self.fh_filetype is None:
              self.fh_filetype = '.txt'

        c.close()
        return(self.fh_filetype)

    def file_matrix_24c01(self):
        print("matrix: ",self.toc_FileList_data)
        fh = []
        self.file_block.clear()
        self.file_gap.clear()
        
        for x in self.toc_FileList_data :
           if x != 0 :
              fh = self.read_file_header(x)
              self.file_block.append(StartEnd(start = x, end = x + fh[3] + 6))
           else :
              if self.file_block != []:
                 print("Last Addr in Matrix: [{}]". format(','.join(hex(x) for x in self.file_block[-1].out())))
                 self.file_db_address = self.file_block[-1].out()[1]
              else :
                 self.file_db_address = self.toc_DataBlock + 0
              pass

        #self.file_gap.append(startEnd())
        start = self.TOC_start_address + 6
        end = 0x7f
        for x in range(len(self.file_block)) :
           v_start = start
           v_end = end
           (y_start,y_end) = self.file_block[x]
           first = 0
           for y in range (v_start,v_end) :
              if y <= y_start :
                 if first == 0 :
                    #print ("address: {}".format(hex(y)))
                    low = y
                    first = 1
              else :
                 #print ("end address {}".format(hex(y-1)))
                 high  = y - 1
                 v_start = y_end
                 break
           self.file_gap.append(StartEndSize(start = low, end = high))
           try :
              (y_start,y_end) = self.file_block[x+1]
           except :
              v_start = y_end + 1
              v_end = 0x7f
              self.file_gap.append(StartEndSize(start = v_start, end = v_end))

           #for y in range (v_start,v_end) :
           #   print("x: {}{}".format(hex(y),hex(v_end)))
           #   if y >= y_end :
           #      print ("address: {}".format(hex(y)))
           #   else :
           #      print ("end address {}".format(hex(y)))
           #      v_start = v_end
           #      break
           

        if self.file_block :
           for x in range(len(self.file_block)) :
              print("matrix_data: [{}]". format(','.join(hex(x) for x in self.file_block[x].out())))
              
        if self.file_gap :
           for x in range(len(self.file_gap)) :
              print("matrix_gap: [{}]". format(','.join(hex(x) for x in self.file_gap[x].out())))
        
        #print("matrix_data: {}". format(self.file_block[1].out()))
        pass

    def remove_file_from_TOC(self,offset):
        # delete file information from TOC
        idx = 0
        vshift = 0
        if self.chip_ic == '24c01' :
           print("remove_file_from_TOC")
           self.get_file_from_TOC()
           self.toc_NumberOfFiles_data = self.toc_data_content [1]
           self.toc_FreeMemorySize_data = self.toc_data_content [0]
           for x in self.file_db_address_list :
              if (idx == offset or vshift == 1) :
                 vshift = 1
                 print ("address: ",idx,x)
                 if (idx > offset) :
                    print("shift <-: ", x)
                    eeprom_write.writeNBytes(self.TOC_start_address + 2 + (idx - 1), hex_to_bytes(x), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                    print("set to 0", x)
                    eeprom_write.writeNBytes(self.TOC_start_address + 2 + idx, hex_to_bytes(0x00), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 elif (idx == offset) :
                    print("set to 0", x)
                    eeprom_write.writeNBytes(self.TOC_start_address + 2 + idx, hex_to_bytes(0x00), self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                    self.toc_NumberOfFiles_data = self.toc_NumberOfFiles_data - 1
                    #print (self.toc_FreeMemorySize_data, self.fh_filesize_data)
                    self.toc_FreeMemorySize_data = self.toc_FreeMemorySize_data + self.fh_filesize_data + 6
                 elif ( x == 0 ) :
                    break
                 idx = idx + 1
              else :
                 idx = idx + 1

           self.sync_TOC()
        pass

    def get_file_from_TOC(self):
        # get file information from TOC
        if self.chip_ic == '24c01' :
           self.file_db_address_list  = eeprom_read.readNBytes(self.TOC_start_address + 2, self.TOC_start_address + 3, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
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
        for x in self.fh_build_data :
           data_content.append(x)
           if c <= (len(self.fh_build_data) - 2) :   #CRC & Attribute byte are excluded
              data_crc.append(x)
           c = c + 1
        for x in data :
           data_content.append(ord(x))
           data_crc.append(ord(x))

        if len(data_content) % 2 != 0:
           print("write_file append 0x00")
           data_content.append(0x00)
           data_crc.append(0x00)

        #print("Data CRC: ",data_crc)
        print("Data CRC out: ",calculate_byte_crc(data_crc))
        data_content[5] = calculate_byte_crc(data_crc)

        self.file_matrix_24c01()
        self.add_file_to_TOC()
        #print(data_content)
        cmp = eeprom_write.writeNBytes(self.file_db_address, data_content, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)

        pass
        
    def load_file(self, filename, filetype) :
        self.fh_filename_data = self.get_file_from_TOC()
        file_found = 0
           
        if self.chip_ic == '24c01' :
           for x in self.file_db_address_list :
              if x != 0 :
                 self.fh_data_content = eeprom_read.readNBytes(x, x + 5, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 self.fh_filename_data = ""
                 for y in self.fh_data_content [:2] :
                    self.fh_filename_data = self.fh_filename_data + chr(y)
                 self.fh_filetype_data = self.code_filetype(self.fh_data_content [2],1)
                 #print (self.fh_filename_data,".",self.fh_filetype_data)
                 if (filename == self.fh_filename_data and filetype == self.fh_filetype_data) :
                    file_found = 1
                    self.fh_filesize_data = self.fh_data_content [3]
                    self.fh_attribute_data = self.fh_data_content [4]
                    self.fh_crc_data = self.fh_data_content [5]
                    for z in eeprom_read.readNBytes(x + 6, x + 6 + self.fh_filesize_data -1, self.busnum,self.chip_address,self.writestrobe,self.chip_ic):
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
           
        if self.chip_ic == '24c01' :
           for x in self.file_db_address_list :
              if x != 0 :
                 self.fh_data_content = eeprom_read.readNBytes(x, x + 5, self.busnum,self.chip_address,self.writestrobe,self.chip_ic)
                 self.fh_filename_data = ""
                 for y in self.fh_data_content [:2] :
                    self.fh_filename_data = self.fh_filename_data + chr(y)
                 self.fh_filetype_data = self.code_filetype(self.fh_data_content [2],1)
                 #print (self.fh_filename_data,".",self.fh_filetype_data)
                 if (filename == self.fh_filename_data and filetype == self.fh_filetype_data) :
                    file_found = 1
                    self.fh_filesize_data = self.fh_data_content [3]
                    self.fh_attribute_data = self.fh_data_content [4]
                    self.fh_crc_data = self.fh_data_content [5]
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
        pattern = '^(.*)\.(.*)$'
        match = re.search(pattern,tmp)
        filename = match.group(1)
        filetype = self.code_filetype(match.group(2))
        print ("FileName: {}".format(filename))
        print ("FileType: {:02x}".format(filetype))
        print ("FileSize: {}".format(FileSize))

        rc = self.build_file_header(filename, filetype, FileSize, Option_RO = False)
        if rc[0] > 0x0a :
           self.error_code['write_eepromfs'] = self.ERR_BUILD_FILE_HEADER
           return (list(self.error_code.values())[-1])
        #print (file_data)
        self.write_file(file_data)
        self.error_code['write_eepromfs'] = self.FILE_WRITTEN

        return (list(self.error_code.values())[-1])

    def load_eepromfs(self, file_name) :
        raw_line = ""

        pattern = '\/{1}(.*)\.(.*)$'
        match = re.search(pattern,file_name)
        filename = match.group(1)
        filetype = match.group(2)
        print ("FileName: {}".format(filename))
        print ("FileType: {}".format(filetype))

        self.load_file(filename, filetype)
        print(self.file_data) 

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

