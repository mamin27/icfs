#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs
import sys, getopt

def help() :
    print ('eeprom_remove_file.py <option>')
    print (' option:')
    print ('  -h help')
    print ('  -f file name, remove it in EEPROM')
    print ('     example: -f AT.pdf')

if __name__ == '__main__' :

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

   if filename == None :
      filename = 'AT.pdf'
   rc = toc.remove_eepromfs(filename)

   print("Process Stats: {}".format(toc.error_code))
   if toc.error_code.popitem()[1] >= 0xa0 :
      print(toc.error_msg(toc.error_code.popitem()[1]))
   #print ("RC: {}".format(rc))
