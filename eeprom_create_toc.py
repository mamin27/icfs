import eepromfs

toc = eepromfs.EEPROM_FS()

print("Mode: {}".format(toc.mode))
print ("Board version: {}".format(toc.version['TYPE']))
print ("Board RAM: {}".format(toc.version['RAM']))
