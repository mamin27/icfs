#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs

toc = eepromfs.EEPROM_FS()

toc.build_TOC()
toc.check_TOC()

print("Process Stats: {}".format(toc.error_code))
if toc.error_code.popitem()[1] >= 0xa0 :
      print(toc.error_msg(toc.error_code.popitem()[1]))
