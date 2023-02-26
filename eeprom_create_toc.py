#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs
import logging

logging.basicConfig(level=logging.DEBUG,  # change level looging to (INFO, DEBUG, ERROR)
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M',
                    filename='eeprom.log',
                    filemode='w')
console = logging.StreamHandler()
console.setLevel(logging.INFO)
formatter = logging.Formatter('%(name)-12s: %(levelname)-8s %(message)s')
console.setFormatter(formatter)
logging.getLogger('').addHandler(console)

toc = eepromfs.EEPROM_FS()
toc._logger = logging.getLogger('eeprom_create_toc')

toc.build_TOC()
toc.check_TOC()

toc._logger.debug("Process Stats: {}".format(toc.error_code))
if toc.error_code.popitem()[1] >= 0xa0 :
      toc._logger.info(toc.error_msg(toc.error_code.popitem()[1]))
