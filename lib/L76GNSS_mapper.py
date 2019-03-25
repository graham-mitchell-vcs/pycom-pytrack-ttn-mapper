from machine import Timer
import time
import gc
import binascii


class L76GNSS:

    GPS_I2CADDR = const(0x10)

    def __init__(self, pytrack=None, sda='P22', scl='P21', timeout=None):
        if pytrack is not None:
            self.i2c = pytrack.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        self.chrono = Timer.Chrono()

        self.timeout = timeout
        self.timeout_status = True

        self.reg = bytearray(1)
        self.i2c.writeto(GPS_I2CADDR, self.reg)

    def _read(self):
        self.reg = self.i2c.readfrom(GPS_I2CADDR, 64)
        return self.reg

    def _convert_coords(self, ggall_s):
        lat = ggall_s[2]
        lat_d = (float(lat) // 100) + ((float(lat) % 100) / 60)
        lon = ggall_s[4]
        lon_d = (float(lon) // 100) + ((float(lon) % 100) / 60)
        if ggall_s[3] == 'S':
            lat_d *= -1
        if ggall_s[5] == 'W':
            lon_d *= -1
        return(lat_d, lon_d)

    def coordinates(self, debug=False):
        lat_d, lon_d, alt_m, hdop, debug_timeout = None, None, None, None, False
        if self.timeout is not None:
            self.chrono.reset()
            self.chrono.start()
        nmea = b''
        while True:
            if self.timeout is not None and self.chrono.read() >= self.timeout:
                self.chrono.stop()
                chrono_timeout = self.chrono.read()
                self.chrono.reset()
                self.timeout_status = False
                debug_timeout = True
            if not self.timeout_status:
                gc.collect()
                break
            nmea += self._read().lstrip(b'\n\n').rstrip(b'\n\n')
            ggall_idx = nmea.find(b'GPGGA')
            if ggall_idx >= 0:
                ggall = nmea[ggall_idx:]
                e_idx = ggall.find(b'\r\n')
                if e_idx >= 0:
                    try:
                        ggall = ggall[:e_idx].decode('ascii')
                        ggall_s = ggall.split(',')
                        lat_d, lon_d = self._convert_coords(ggall_s)
                        hdop = float(ggall_s[8])
                        alt_m = float(ggall_s[9])
                    except Exception:
                        pass
                    finally:
                        nmea = nmea[(ggall_idx + e_idx):]
                        gc.collect()
                        break
            else:
                gc.collect()
                if len(nmea) > 410: # i suppose it can be safely changed to 82, which is longest NMEA frame
                    nmea = nmea[-5:] # $GNGL without last L (check needed)
            time.sleep(0.1)
        self.timeout_status = True
        if debug and debug_timeout:
            print('GPS timed out after %f seconds' % (chrono_timeout))
            return(None, None, None, None)
        else:
            return(lat_d, lon_d, alt_m, hdop)
