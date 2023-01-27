#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs

toc = eepromfs.EEPROM_FS()

for i in toc.ls_eepromfs() :
   print (i)

print("Process Stats: {}".format(toc.error_code))
