#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs

toc = eepromfs.EEPROM_FS()

toc.build_TOC()
toc.check_TOC()

print("Process Stats: {}".format(toc.error_code))
