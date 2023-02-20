#!/bin/bash

./eeprom_create_toc.py
./eeprom_write_file.py -f data/AT.pdf
./eeprom_write_file.py -f data/AAAA.txt
./eeprom_write_file.py -f data/BB.txt
./eeprom_write_file.py -f data/CC.txt
./eeprom_write_file.py -f data/DD.pdf
./eeprom_write_file.py -f data/GG.txt

#./eeprom_list.py

./eeprom_remove_file.py -f 'AAAA.txt'
./eeprom_remove_file.py -f 'DD.pdf'

./eeprom_write_file.py -f data/FF.pdf

./eeprom_list.py

./eeprom_write_file.py -f data/EE.pdf

