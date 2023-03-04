#!/usr/bin/env python3
#import sys
#sys.path.append("..")

from icfs import eepromfs
import sys, getopt
import logging

def help() :
    print ('eeprom_remove_file.py <option>')
    print (' option:')
    print ('  -h help')
    print ('  -f file name, remove it in EEPROM')
    print ('     example: -f AT.pdf')

if __name__ == '__main__' :

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

   if len(sys.argv) == 1 :
         help()
         sys.exit(10)

   filename = None

   try:
      opts, args = getopt.getopt(sys.argv[1:],"hp:trwef:",["file="])
   except getopt.error:
      help()
      sys.exit(10)
   for opt, arg in opts:
      if opt == ('-h','--help'):
         help()
         sys.exit()
      elif opt in ('-f', '--file'):
         filename = arg

   toc = eepromfs.EEPROM_FS()
   toc._logger = logging.getLogger('eeprom_remove')

   if filename == None :
      filename = 'AT.pdf'
   rc = toc.remove_eepromfs(filename)

   toc._logger.debug("Process Stats: {}".format(toc.error_code))
   if toc.error_code.popitem()[1] >= 0xa0 :
      toc._logger.info(toc.error_msg(toc.error_code.popitem()[1]))
   #print ("RC: {}".format(rc))
