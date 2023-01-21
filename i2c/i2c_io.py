def eeprom_set_addr(addr,smb,slaveaddr,chip) :
    
    haddr = 0x00 + addr
    hslaveaddr = 0x00 + slaveaddr

    if (chip <= 2) :
        smb.write_byte(hslaveaddr, addr%256)
    elif (chip <= 5) :
        if addr//256 > 0 :
            hslaveaddr = hslaveaddr | addr//256 
        smb.write_byte(hslaveaddr, addr%256)
    elif (chip <= 10) :
        smb.write_byte_data(slaveaddr, addr//256, addr%256)
    else :
        if addr//65535 > 0 :
            hslaveaddr = hslaveaddr | addr//65535
        smb.write_byte_data(hslaveaddr, addr//256, addr%256)
