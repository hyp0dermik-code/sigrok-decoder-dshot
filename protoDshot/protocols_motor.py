from functools import reduce
from enum import Enum
from .dshot_bits import Sequence

gcr_tables = {
    "0b11001": 0x0,
    "0b11011": 0x1,
    "0b10010": 0x2,
    "0b10011": 0x3,
    "0b11101": 0x4,
    "0b10101": 0x5,
    "0b10110": 0x6,
    "0b10111": 0x7,
    "0b11010": 0x8,
    "0b1001": 0x9,
    "0b1010": 0xa,
    "0b1011": 0xb,
    "0b11110": 0xc,
    "0b1101": 0xd,
    "0b1110": 0xe,
    "0b1111": 0xf
}



class DshotSettings():
    def __init__(self):
        self.samplerate = 0
        #print(self.samplerate)
        self.bidirectional = False
        self.dshot_kbaud = 300e3
        self.dshot_period = None
        self.samples_after_motorcmd = None
        self.samples_after_telempkt = None
        self.samples_pp = None
        self.telem_baudrate_midpoint = 0
        self.telem_start = None
        self.edt_force = False
        self.update()
        return

    def update(self):
        self.dshot_period = 1 / self.dshot_kbaud
        self.samples_pp = int(self.samplerate * self.dshot_period)
        self.samples_after_motorcmd = self.samples_pp * 3
        self.samples_after_telempkt = self.samples_pp * 3
        self.telem_baudrate_midpoint = int((self.samplerate / (self.dshot_kbaud * (5 / 4))) / 2.0)

class DshotCommon():
    def __init__(self,settings_Dshot=DshotSettings()):
        self.cfg = settings_Dshot
        self.crc_recv = None
        self.crc_calc = None
        self.crc_ok = False
        self.bits = 0
        self.results = []
        self.packet = Sequence()

    def checkCRC(self,data,crc_recv):
        self.crc_recv = crc_recv
        if self.cfg.bidirectional:
            # TODO: Move CRC out?
            self.crc_calc = int((~(data ^ (data >> 4) ^ (data >> 8))) & 0x0F)
        else:
            self.crc_calc = int(((data ^ (data >> 4) ^ (data >> 8))) & 0x0F)

        if not (self.crc_recv == self.crc_calc):
            self.crc_ok = False
            return False
        self.crc_ok = True
        return True

    def add_bit(self, seq):
        self.results += [seq]
        self.bits = self.bits << 1
        self.bits = self.bits | seq.bit_

class DshotCmd(DshotCommon):
    def __init__(self,*args):
        super().__init__(*args)

        self.dshot_value = None
        self.telem_request = None
        return
    def handle_bits_dshot(self):
        # ss, es, bit
        if len(self.results) != 16:
            return False
        # Get bits only
        bits = [bool(result) for result in self.results]
        # Convert to binary from list
        bits = reduce(lambda a, b: (a << 1) | b, bits)
        # Seperate CRC
        crc_recv = bits & 0xF
        bits = bits >> 4
        # Remainder is data
        data = bits
        # Telem request
        self.telem_request = bits & 0x1
        # Rest is dshot value
        bits = bits >> 1
        self.dshot_value = bits

        if not self.checkCRC(data,crc_recv):
            return False


        return True
            # TODO: Align this correctly


class BitException(ValueError):
    def __init__(self, arg1, arg2):
        super().__init__(arg1)
        print("Second argument is " + arg2)


class DshotTelem(DshotCommon):
    def __init__(self,*args):
        super().__init__(*args)

        self.dshot_value = None
        self.telem_request = None
        self.xor = 0b0
        return



    def bits_xor_next(self,bits):
        return bits ^ (bits >> 1)

    def bits_gcr(self,bits):
        try:
            return gcr_tables[bin(bits)]
        except:
            raise

    def process_telem_erpm(self):
        # Raw packet
        #self.put(start, end, self.out_ann, [6, ['%23s' % bin(packet)]])
        # XOR with next?
        bits = self.bits
        bits &= 0x0FFFFF
        bits = self.bits_xor_next(bits)
        #self.put(start,end, self.out_ann, [7, ['%23s' % bin(packet)]])
        # Undo GCR
        output = 0b0

        nibbles = 4
        bitmask = 0b11111 << ((nibbles - 1) * 5)

        for n in range(nibbles):
            try:
                gcr_n = bitmask & bits
                key = gcr_n >> (nibbles - (n + 1)) * 5
                ungcr = self.bits_gcr(key)
                output = (output << 4) | ungcr
                bitmask = (bitmask >> 5)
            except:
                raise BitException(bin(key),bin(gcr_n))

        # Compare CRC
        crc_recv = output & 0xF
        data = (output >> 4) & 0xFFF

        if not self.checkCRC(data,crc_recv):
            raise ValueError
        return True
        # self.put(end - ((self.telem_baudrate_midpoint * 2) * 4),
        #          end, self.out_ann,
        #          [7, ['%23s' % ("RX CRC: " + hex(crc_received) + " Calc CRC: " + hex(crc_calc))]])
        # if crc_calc != crc_received:
            # self.put(end - ((self.telem_baudrate_midpoint * 2) * 4),
            #          end, self.out_ann,
            #          [8, ['%23s' % ("CRC ERROR!")]])
        # The upper 12 bit contain the eperiod (1/erps) in the following bitwise encoding:
        #
        # e e e m m m m m m m m m
        #
        # The 9 bit value M needs to shifted left E times to get the period in micro seconds.
        # This gives a range of 1 us to 65408 us. Which translates to a min e-frequency of 15.29 hz or for 14 pole motors 3.82 hz.
        return

    def process_telem_edt(self):

        return

    def process_telem(self):
        if self.cfg.edt_force:
            self.process_telem_edt()
        else:
            self.process_telem_erpm()

        return