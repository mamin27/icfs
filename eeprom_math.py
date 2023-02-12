from crc import Calculator, Crc16, Crc8

def hex_to_4bytes(n):
    return [(n >> 24) & 0xff, (n >> 16) & 0xff, (n >> 8) & 0xff, n & 0xff]

def hex_to_3bytes(n):
    return [n >> 16, (n >> 8) & 0xff, n & 0xff]

def hex_to_2bytes(n):
    return [(n >> 8) & 0xff, n & 0xff]

def hex_to_bytes(n):
    return [n & 0xff]

def zero_to_bytes(byte_nr):
    n = []
    for x in range (byte_nr) :
       n.append(0x00)
    return n

def calculate_2byte_crc(*data):
    calc = Calculator(Crc16.CCITT)
    return calc.checksum(data)
    
def calculate_byte_crc(*data):
    calc = Calculator(Crc8.CCITT)
    return calc.checksum(data)

def dec_to_list(n):
    return [n]
