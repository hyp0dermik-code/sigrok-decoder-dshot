## Modified from rgb_led_ws281x - original license below:
## Copyright (C) 2023: hyp0dermik@gmail.com

##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2016 Vladimir Ermakov <vooon341@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
from functools import reduce



class SamplerateError(Exception):
    pass

class Decoder(srd.Decoder):
    api_version = 3
    id = 'dshot'
    name = 'DShot'
    longname = 'DShot RC Hobby Motor Protcol Decoder'
    desc = 'DShot RC Hobby Motor Protcol Decoder'
    license = 'gplv3+'
    inputs = ['logic']
    outputs = []
    tags = ['Display', 'IC']
    channels = (
        {'id': 'din', 'name': 'DIN', 'desc': 'DIN data line'},
    )

    options = (
        {'id': 'dshot_rate', 'desc': 'DShot Rate', 'default': '150','values': ('150', '300','600','1200')},
        { 'id': 'bidir', 'desc': 'Bidirectional DShot','default': 'False', 'values': ('True', 'False')},
        { 'id': 'log', 'desc': 'Write log file','default': 'no', 'values': ('yes', 'no')},
    )
    annotations = (
        ('bit', 'Bit'),
        ('cmd', 'Command'),
        ('throttle', 'Throttle'),
        ('checksum', 'CRC'),
        ('errors', 'Errors'),

    )
    annotation_rows = (
        ('bit', 'Bits', (0,)),
        ('throttle', 'Throttle', (1,2,3)),
        ('dshot_errors', 'Dshot Errors', (4,)),
    )

    dshot_period_lookup = {'150': 6.67e-6, '300': 3.33e-6,'600':1.67e-6,'1200':0.83e-6}

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.oldpin = None
        self.ss_packet = None
        self.ss = None
        self.es = None
        self.bits = []
        self.inreset = False
        self.bidirectional = False
        self.dshot_period = 3.33e-6
        self.actual_period = None
        

    def start(self):
        self.bidirectional = True if self.options['bidir'] == 'True' else False
        self.dshot_period = self.dshot_period_lookup[self.options['dshot_rate']]
        #print("start period",self.dshot_period)
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def handle_bits(self, samplenum):
        if len(self.bits) == 16:
            throttle = int(reduce(lambda a, b: (a << 1) | b, self.bits[:11]))
            telem_request = self.bits[11]
            received_crc = int(reduce(lambda a, b: (a << 1) | b, self.bits[12:]))
        
            value_tocrc = int(reduce(lambda a, b: (a << 1) | b, self.bits[:12]))
         
            calculated_crc = ((~(value_tocrc ^ (value_tocrc >> 4) ^ (value_tocrc >> 8))) & 0x0F)

            crc_ok = True if value_tocrc == calculated_crc else False

            # TODO: Align this correctly
            crc_startsample = samplenum-(self.actual_period*5)

            self.put(self.ss_packet, crc_startsample, self.out_ann,
                     [2, ['%04d' % throttle]])
            self.put(crc_startsample, samplenum, self.out_ann, [3, ['Calc CRC: '+('%04d' % calculated_crc)+' TXed CRC:'+('%04d' % received_crc)]])
            if not crc_ok:
                self.put(crc_startsample, samplenum, self.out_ann,
                     [4, ['CRC INVALID']])
         

            self.bits = []
            self.ss_packet = None
        else:
             self.put(self.es, self.samplenum, self.out_ann,
                         [1, ['ERROR: INVALID PACKET LENGTH', 'ERR', 'E']])


    def handle_bit(self):
        if self.ss and self.es:
            period = self.samplenum - self.ss
            duty = self.es - self.ss
            # Ideal duty for T0H: 33%, T1H: 66%.
            bit_ = (duty / period) > 0.5

            self.put(self.ss, self.samplenum, self.out_ann,
            [0, ['%d' % bit_]])

            self.bits.append(bit_)
            self.actual_period = period
            #self.handle_bits(self.samplenum)
        if self.ss_packet is None:
            self.ss_packet = self.samplenum
        self.ss = self.samplenum

    def handle_notbit(self):
        self.inreset = False
        self.es = self.samplenum

    def check_reset(self):
        # Decode last bit value.
        tH = (self.es - self.ss) / self.samplerate
        
        # High if greater than half the period
        bit_ = True if tH >= (self.dshot_period/2) else False

        self.bits.append(bit_)
        self.handle_bits(self.es)

        self.put(self.ss, self.es, self.out_ann, [0, ['%d' % bit_]])
        # self.put(self.es, self.samplenum, self.out_ann,
        #             [1, ['RESET', 'RST', 'R']])

        self.inreset = True
        self.bits = []
        self.ss_packet = None
        self.ss = None
    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')

        while True:
            # TODO: Come up with more appropriate self.wait() conditions.
            (pin,) = self.wait()

            if self.oldpin is None:
                self.oldpin = pin
                continue

            # Check idle condition if longer is greater than 2x max period
            # TODO: Confirm this with spec
            if not self.bidirectional:
                if not self.inreset and not pin and self.es is not None and \
                    self.ss is not None and \
                    (self.samplenum - self.es) / self.samplerate > self.dshot_period*2:
                        self.check_reset()
                if not self.oldpin and pin:
                    # Rising edge
                    self.handle_bit()
                elif self.oldpin and not pin:
                    # Falling edge
                    self.handle_notbit()
            else:
                if self.oldpin and not pin:
                        # Falling edge.
                    self.handle_bit()
                elif not self.oldpin and pin:
                             # Rising edge.
                    self.handle_notbit()

            self.oldpin = pin




