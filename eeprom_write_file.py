#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs

toc = eepromfs.EEPROM_FS()

filename = 'data/AT.pdf'
rc = toc.write_eepromfs(filename)

print("Process Stats: {}".format(toc.error_code))
#print ("RC: {}".format(rc))
