import smbus
import time

# Constants
I2C_ADDR = 0x40  # Default PCA9685 address
MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
Servo_0_ON = 0x06

# I2C bus 1 for Raspberry Pi
bus = smbus.SMBus(1)

def set_pwm_freq(freq_hz):
    prescaleval = 25000000.0    # 25MHz internal oscillator
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq_hz)
    prescaleval -= 1.0
    prescale = int(prescaleval + 0.5)

    oldmode = bus.read_byte_data(I2C_ADDR, MODE1)
    newmode = (oldmode & 0x7F) | 0x10  # sleep
    bus.write_byte_data(I2C_ADDR, MODE1, newmode)
    bus.write_byte_data(I2C_ADDR, PRESCALE, prescale)
    bus.write_byte_data(I2C_ADDR, MODE1, oldmode)
    time.sleep(0.005)
    bus.write_byte_data(I2C_ADDR, MODE1, oldmode | 0x80)  # restart

def set_pwm(channel, on, off):
    base = Servo_0_ON + 4 * channel
    bus.write_byte_data(I2C_ADDR, base, on & 0xFF)
    bus.write_byte_data(I2C_ADDR, base + 1, on >> 8)
    bus.write_byte_data(I2C_ADDR, base + 2, off & 0xFF)
    bus.write_byte_data(I2C_ADDR, base + 3, off >> 8)
def set_degree(channel, deg):
    deg = max(0, min(180, deg))

    ticks = int(105 + (deg / 180.0) * 407)

    base = 0x06 + 4 * channel  # LED0_ON_L is 0x06
    on = 0
    off = ticks

    bus.write_byte_data(I2C_ADDR, base, on & 0xFF)
    bus.write_byte_data(I2C_ADDR, base + 1, on >> 8)
    bus.write_byte_data(I2C_ADDR, base + 2, off & 0xFF)
    bus.write_byte_data(I2C_ADDR, base + 3, off >> 8)

    print(f"Channel {channel}, {deg}° → ticks={ticks}")

def check_connection():
    try:
        mode1 = bus.read_byte_data(I2C_ADDR, MODE1)
        mode2 = bus.read_byte_data(I2C_ADDR, MODE2)
        print(f"Connection OK ✅")
        print(f"MODE1 = 0x{mode1:02X}, MODE2 = 0x{mode2:02X}")
        return True
    except Exception as e:
        print(f"Connection failed ❌: {e}")
        return False
    
def dump_registers():
    """
    Read and print all 256 registers of the PCA9685.
    Useful for debugging and verifying default values.
    """
    print("Register dump (0x00–0xFF):")
    for row in range(0, 256, 16):
        line = f"0x{row:02X}: "
        try:
            regs = bus.read_i2c_block_data(I2C_ADDR, row, 16)
            line += " ".join(f"{val:02X}" for val in regs)
        except Exception as e:
            line += f"Error: {e}"
        print(line)

def wake_up():
        mode1 = bus.read_byte_data(I2C_ADDR, MODE1)
        new_mode1 = mode1 & ~0x10 #turn off sleep bit
        bus.write_byte_data(I2C_ADDR, MODE1, new_mode1)
def sweep(channel, start_deg=0, end_deg=180, step=1, delay=0.02):
    """
    Smoothly move a servo between angles.
    :param channel: PCA9685 channel (0–15)
    :param start_deg: starting angle (deg)
    :param end_deg: ending angle (deg)
    :param step: angle increment per step (deg)
    :param delay: pause between steps (s)
    """
    if start_deg < end_deg:
        rng = range(start_deg, end_deg + 1, step)
    else:
        rng = range(start_deg, end_deg - 1, -step)

    for deg in rng:
        set_degree(channel, deg)
        time.sleep(delay)
# Example usage:
check_connection()
# dump_registers()
wake_up()
# check_connection()
# Set frequency
set_pwm_freq(50)

servo_test_channel = 11
# Move servo on channel 3
# print("Min (0 deg)")
# # set_pwm(3, 0,105)
# set_degree(servo_test_channel, 0)
# time.sleep(1)

# print("Center (90 deg)")
# # set_pwm(3, 0, 512)
# set_degree(servo_test_channel, 90)
# time.sleep(1)

# print("Max (180 deg)")
# # set_pwm(3, 0, 2500)
# set_degree(servo_test_channel, 0)
# time.sleep(1)

sweep(servo_test_channel, 90, 135, step=2, delay=0.05)
time.sleep(1)
sweep(servo_test_channel, 135, 45, step=2, delay=0.05)
time.sleep(1)
sweep(servo_test_channel, 45, 90, step=2, delay=0.05)

