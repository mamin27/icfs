#!/bin/bash

./eeprom_create_toc.py
./eeprom_write_file.py -f data/AAAA.txt
./eeprom_write_file.py -f data/BB.txt
./eeprom_write_file.py -f data/CC.txt
./eeprom_write_file.py -f data/DD.pdf
./eeprom_write_file.py -f data/EE.pdf
./eeprom_write_file.py -f data/FF.pdf
#./eeprom_write_file.py -f data/GG.txt

#./eeprom_list.py

./eeprom_remove_file.py -f 'CC.txt'
./eeprom_remove_file.py -f 'FF.pdf'

./eeprom_write_file.py -f data/AT.pdf
./eeprom_write_file.py -f data/GG.txt

./eeprom_list.py

./eeprom_load_file.py -f AAAA.txt
./eeprom_load_file.py -f BB.txt
./eeprom_load_file.py -f CC.txt
./eeprom_load_file.py -f DD.pdf
./eeprom_load_file.py -f EE.pdf
./eeprom_load_file.py -f FF.pdf

hexyl data/AAAA.txt
hexyl AAAA.txt
hexyl data/BB.txt
hexyl BB.txt
hexyl data/CC.txt
hexyl CC.txt
hexyl data/DD.pdf
hexyl DD.pdf
hexyl data/EE.pdf
hexyl EE.pdf
hexyl data/FF.pdf
hexyl FF.pdf
hexyl data/GG.txt
hexyl GG.txt

