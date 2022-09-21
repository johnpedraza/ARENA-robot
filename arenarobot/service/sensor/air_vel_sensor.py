import smbus
from time import sleep

# air velocity sensor address
sensor_addr = 0x28

# access i2c bus
i2c = smbus.SMBus(1)

