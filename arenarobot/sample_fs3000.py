import smbus
import time

Bus = smbus.SMBus(1)
# max: 1675 but not really
while(True):
    time.sleep(0.05)
    try:
        data = Bus.read_i2c_block_data(0x28, 0x01, 5)
        data_high, data_low = data[1], data[2]

        data_high &= 7

        result = (data_high << 8) | data_low

        print(result)
    except:
        print('I/O Error')