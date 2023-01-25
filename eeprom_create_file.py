#!/usr/bin/env python3
#import sys
#sys.path.append("..")

import eepromfs
import re

toc = eepromfs.EEPROM_FS()

print("Mode: {}".format(toc.mode))
print ("Board version: {}".format(toc.version['TYPE']))
print ("Board RAM: {}".format(toc.version['RAM']))

filename = 'data/AT.pdf'
FileSize = 0
raw_line = list()
rc = 1

f = open(filename,"r")
while True :
    try:
       raw_line = f.readline()
    except :
       rc = 2
       
    if not raw_line:
        break
    FileSize = FileSize + len(raw_line)
    file_data = raw_line

f.close()

tmp = filename
pattern = '^(.*)\.(.*)$'
match = re.search(pattern,tmp)
filename = match.group(1)
filetype = toc.code_filetype(match.group(2))
print ("FileName: {}".format(filename))
print ("FileType: {:02x}".format(filetype))
print ("FileSize: {}".format(FileSize))

toc.build_file_header(filename, filetype, FileSize, Option_RO = False)
print (file_data)
toc.write_file(file_data)



print ("RC: {}".format(rc))
