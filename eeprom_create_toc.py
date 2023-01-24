#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs

toc = eepromfs.EEPROM_FS()
print("Mode: {}".format(toc.mode))
print ("Board version: {}".format(toc.version['TYPE']))
print ("Board RAM: {}".format(toc.version['RAM']))

toc.build_TOC()

toc.check_TOC()
