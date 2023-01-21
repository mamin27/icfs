import time
import sys, getopt
import yaml
import re
import RPi.GPIO as rGPIO
from smbus2 import SMBus
from i2c import i2c_io

class EEPROM_FS(object):
        
    def __init__(self, chip_address=None, busnum=None, writestrobe=None, TOC_start_address=None, block_size=None, i2c=None, **kwargs) :
            
        self.chip_address = chip_address
        self.busnum = busnum
        self.writestrobe = writestrobe
        self.TOC_start_address = TOC_start_address
        self.block_size = block_size
        
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
        
        if self.chip_address is None:
           self.chip_address= config['i2c']['eeprom']['slaveaddr'] # for eeprom (main i2c address)
        if self.busnum is None:
           self.busnum = SMBus(int(re.search("^i2c-(\d+)$",config['i2c']['smb']).group(1))) # set bus i2c-1
        if self.writestrobe is None:
           self.writestrobe = config['i2c']['eeprom']['writestrobe'] # hold pin low to write to eeprom
        if self.TOC_start_address is None:
           self.TOC_start_address = config['eeprom_fs']['TOC_start_address'] = 0
        if self.block_size is None:
           self.block_size = config['eeprom_fs']['block_size'] = 2048
           
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

