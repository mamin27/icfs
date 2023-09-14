# ecomet_eepromfs
Creation of driver for EEPROM ICs file system. Using I2C communication. Maintenance of read, write, delete, update file. Also creation TOC sector, data management, counter of write operation.

**Last modification:** 14.9.2023
**Contributor:** Marian Minar

<a href="https://www.buymeacoffee.com/scQ8LwgTBt"><img src="https://img.buymeacoffee.com/button-api/?text=Buy me a coffee&emoji=&slug=scQ8LwgTBt&button_colour=5F7FFF&font_colour=ffffff&font_family=Cookie&outline_colour=000000&coffee_colour=FFDD00" /></a>

**Twitter:** [News and statuses](https://twitter.com/mminar7)

**Setting of config file for ecomet-i2c-sensors python library:**

Library is looking in directory **~/.comet** for file config.yaml that contains i2c parameters for custom linux distribution and I2C EEPROM memory setting.
Here is a example of config file content:

```sh
comet@orangepizero2:~/.comet $ cat config.yaml
--- # The I2C config file

i2c:
    smb: i2c-3 # set bus i2c-1
    smb_detect: 0..3 # list of monitored smb bus by i2cdetect
    eeprom:
      ic: '24c01'
      slaveaddr: 0x50
      writestrobe: 26 # hold pin low to write to eeprom
```

| parameter | sub-parameter | description | example value |
| --------------- |:---------------:|:--------:|:---:|
| smb: | | name of active i2c in /dev | i2c-1 |
| smb_detect: | | list of monitored i2c neworks used by command **i2c_ecomet_detect** | 0..1 |
| eeprom: | | eeprom section | |
| | ic: | 24x00 series of eeprom chip name | '24c01' |
| | slaveaddr: | address of eeprom chip in hex | 0x50 |
| | writestrobe: | GPIO pin number used as CS (chip select) signal for eeprom IC | 26 |

**Note: Currently only one eeprom chip could be added.**
