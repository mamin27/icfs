import time
import sys, getopt
import yaml
import re
import RPi.GPIO as rGPIO
from smbus2 import SMBus
from i2c import i2c_io

class EEPROM_FS(object):
        
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
        
        self.init_config()
        if chip_address is None:
           self.chip_address = 0x50
        if TOC_start_address is None:
          TOC_start_address = 0
        if busnum is None:
           self.busnum = 'i2c-0'
        
        self.build_TOC()
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
        
        with open("eepromfs.yaml") as d:
           try:
              toc_config = yaml.safe_load(d)
           except yaml.YAMLError as exc:
              print(exc)
              
        if self.TOC_start_address is None:
           self.TOC_start_address = toc_config['eeprom_fs']['TOC_start_address']      
        self.block_size = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['block_size'] 
        if self.chip_ic in ['24C04','24C08','24C16','24C32','24C64','24C128','24C256','24C512','24C1024','24C2048']:
           self.toc_version = toc_config['eeprom_fs']['TOC_version']
        self.toc_FreeMemorySize = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_FreeMemorySize']
        self.toc_NumberOfFiles = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfFiles']
        if self.chip_ic in ['24C04','24C08','24C16','24C32','24C64','24C128','24C256','24C512','24C1024','24C2048']:
           self.toc_NumberOfOrphanBlocks = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfOrphanBlocks']
           self.toc_NumberOfDeletedBlocks = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfDeletedBlocks']
           self.toc_NumberOfWriteBlocks = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_NumberOfWriteBlocks']
        if self.chip_ic in ['24C64','24C128','24C256','24C512','24C1024','24C2048']:
           self.toc_WearLevelThreshold = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_WearLevelThreshold']
           self.toc_WearLevelCount = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_WearLevelCount']
        self.toc_FileList = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_FileList']
        if self.chip_ic in ['24C256','24C512','24C1024','24C2048']:
           self.toc_OwnerHashList = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_OwnerHashList']
        self.toc_crc = toc_config['eeprom_fs']['TOC_attributes'][self.chip_ic]['toc_crc']

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
        # 24C256  [ 3 byte ], [ 3 byte ], [ 2 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 2 byte], [ 1 byte], [ 174(58) byte], [ 120(4) byte],  [ 2 byte] =>  [ 316 bytes ], [ 16 bytes ]
        # 24C512  [ 3 byte ], [ 3 byte ], [ 2 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 2 byte], [ 1 byte], [ 210(70) byte], [ 180(6) byte],  [ 2 byte] =>  [ 412 bytes ], [ 16 bytes ]
        # 24C1024 [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 1 byte], [ 1 byte], [ 270(90) byte], [ 240(8) byte],  [ 2 byte] =>  [ 532 bytes ], [ 32 bytes ]
        # 24C2048 [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 3 byte ], [ 1 byte], [ 1 byte], [ 360(120) byte], [ 300(10) byte],  [ 2 byte] =>  [ 682 bytes ], [ 40 bytes ]
        
        pass
        
    def build_file_header(self):
        self.fh_filename # Start Address of DataLocation
        self.fh_filetype # Type of filename
        self.fh_FilsSize # Free Memory Size
        self.fh_NumberOfUsedBlocks # NumberOfUsedBlocks
        self.fh_Attributes # File Attribute [Read Only],[Owner], shared with InUse [ 1 byte ] 
        self.fh_InUse # True/False with Attributes shared
        self.fh_ModificationDate # Modification Data as TIMESTAMP from year 2023 (10 years) (FFF)-(1F)-(3F) (DAYS)-(HH)-(MM) => (DDDD DDDD DDDD DDDD DDDD DDDD)-(HHHH HHHH HXMM MMMM MMMM)b => 3 byte
        self.fh_CreationDate # Creation Data as TIMESTAMP from year 2023 (10 years) (FFF)-(1F)-(3F) (DAYS)-(HH)-(MM) => (DDDD DDDD DDDD DDDD DDDD DDDD)-(HHHH HHHH HXMM MMMM MMMM)b => 3 byte
        self.toc_crc # CRC, could be shared with Attributes and InUse
        pass
        
        # File header size
        #        Filename,  FileType,    FileSize,   Attribute &   CRC           TOC_SUM
        # 24C01 [ 3 byte ], [ 1 byte ], [ 1 byte], [ 1 byte],               => [ 6 bytes ]      [InUse,Read,CRC5,CRC4,CRC3,CRC2,CRC1,CRC0]
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

    def write_file(self, file_name, data):
        # locate file in TOC
        # write data to chip
        pass
    
    def delete_file(self, file_name):
        # locate file in TOC
        # mark file as deleted in TOC
        # free up space on chip
        pass

    def check_fs(self):
        # check consistency of the file system
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

