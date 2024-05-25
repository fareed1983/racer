import time
import smbus2

PCA9685_SUBADR1 = 0x02
PCA9685_SUBADR2 = 0x03
PCA9685_SUBADR3 = 0x04
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06
LED0_ON_H = 0x07
LED0_OFF_L = 0x08
LED0_OFF_H = 0x09
ALL_LED_ON_L = 0xFA
ALL_LED_ON_H = 0xFB
ALL_LED_OFF_L = 0xFC
ALL_LED_OFF_H = 0xFD

MODE1_RESTART = 0x80
MODE1_SLEEP = 0x10
MODE1_EXTCLK = 0x40
MODE1_AI = 0x20
MODE2_OUTDRV = 0x04

FREQUENCY_OSCILLATOR = 25000000

class Adafruit_PWMServoDriver:
    def __init__(self, address=0x40, busnum=1):
        self._i2caddr = address
        self._bus = smbus2.SMBus(busnum)
        self._oscillator_freq = FREQUENCY_OSCILLATOR

    def begin(self, prescale=0):
        self.reset()
        if prescale:
            self.setExtClk(prescale)
        else:
            self.setPWMFreq(1000)
        return True
    
    def reset(self):
        self.write8(PCA9685_MODE1, MODE1_RESTART)
        time.sleep(0.01)

    def sleep(self):
        awake = self.read8(PCA9685_MODE1)
        sleep = awake | MODE1_SLEEP
        self.write8(PCA9685_MODE1, sleep)
        time.sleep(0.005)

    def wakeup(self):
        sleep = self.read8(PCA9685_MODE1)
        wakeup = sleep & ~MODE1_SLEEP
        self.write8(PCA9685_MODE1, wakeup)

    def setExtClk(self, prescale):
        oldmode = self.read8(PCA9685_MODE1)
        newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP
        self.write8(PCA9685_MODE1, newmode)
        self.write8(PCA9685_MODE1, newmode | MODE1_EXTCLK)
        self.write8(PCA9685_PRESCALE, prescale)
        time.sleep(0.005)
        self.write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_RESTART | MODE1_AI)


    def setPWMFreq(self, freq):
        if freq < 1:
            freq = 1
        if freq > 3500:
            freq = 3500

        prescaleval = (self._oscillator_freq / (freq * 4096.0)) - 1
        prescale = int(prescaleval + 0.5)
        oldmode = self.read8(PCA9685_MODE1)
        newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP
        self.write8(PCA9685_MODE1, newmode)
        self.write8(PCA9685_PRESCALE, prescale)
        self.write8(PCA9685_MODE1, oldmode)
        time.sleep(0.005)
        self.write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI)

    def setOutputMode(self, totempole):
        oldmode = self.read8(PCA9685_MODE2)
        if totempole:
            newmode = oldmode | MODE2_OUTDRV
        else:
            newmode = oldmode & ~MODE2_OUTDRV
        self.write8(PCA9685_MODE2, newmode)

    def readPrescale(self):
        return self.read8(PCA9685_PRESCALE)

    def getPWM(self, num, off=False):
        base = LED0_ON_L + 4 * num
        if off:
            base += 2
        data = self.readBytes(base, 2)
        return data[0] | (data[1] << 8)

    def setPWM(self, num, on, off):
        data = [
            LED0_ON_L + 4 * num,
            on & 0xFF,
            on >> 8,
            off & 0xFF,
            off >> 8,
        ]
        self.writeBytes(data)

    def setPin(self, num, val, invert=False):
        val = min(val, 4095)
        if invert:
            if val == 0:
                self.setPWM(num, 4096, 0)
            elif val == 4095:
                self.setPWM(num, 0, 4096)
            else:
                self.setPWM(num, 0, 4095 - val)
        else:
            if val == 4095:
                self.setPWM(num, 4096, 0)
            elif val == 0:
                self.setPWM(num, 0, 4096)
            else:
                self.setPWM(num, 0, val)

    def writeMicroseconds(self, num, microseconds):
        pulse = microseconds
        pulselength = 1000000.0
        prescale = self.readPrescale() + 1
        pulselength *= prescale
        pulselength /= self._oscillator_freq
        pulse /= pulselength
        self.setPWM(num, 0, int(pulse))

    def getOscillatorFrequency(self):
        return self._oscillator_freq

    def setOscillatorFrequency(self, freq):
        self._oscillator_freq = freq

    def read8(self, addr):
        return self._bus.read_byte_data(self._i2caddr, addr)

    def write8(self, addr, d):
        self._bus.write_byte_data(self._i2caddr, addr, d)

    def readBytes(self, addr, length):
        return self._bus.read_i2c_block_data(self._i2caddr, addr, length)

    def writeBytes(self, data):
        self._bus.write_i2c_block_data(self._i2caddr, data[0], data[1:])
