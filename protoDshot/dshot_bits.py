class Sequence():
    def __init__(self):
        self.ss = None
        self.ts = None
        self.es = None

    def samples(self):
        return self.es-self.ss

class BitDshot(Sequence):
    def __init__(self, ss, ts, es):
        super().__init__()
        self.period = None
        self.duty = None
        self.bit_ = None
        self.ss, self.ts, self.es = ss, ts, es
        self.process_bit()
        return
    def getBit(self):
        if self.bit_ is None:
            raise ValueError
        return self.bit_
    def process_bit(self):

        self.period = self.es - self.ss
        self.duty = self.ts - self.ss
        # Ideal duty for T0H: 33%, T1H: 66%.
        self.bit_ = (self.duty / self.period) > 0.5
        # TODO: Add tolerance
        return self.getBit()

    def __bool__(self):
        return bool(self.getBit())
class Bit_DshotTelem(Sequence):
    def __init__(self, ss, ts, es, matched):
        super().__init__()
        self.period = None
        self.duty = None
        self.bit_ = None

        # Only start sample on first bit is truly known, all other ss/es are guessed based on midpoint
        self.ss, self.ts, self.es = ss, ts, es
        self.process_bit(matched)

    def getBit(self):
        if self.bit_ is None:
            raise ValueError
        return self.bit_
    def process_bit(self,matched):
        # Low/High @ given sample
        if matched == (True, False):
            # 0 value
            self.bit_ = 0

        # High
        if matched == (False, True):
            # 1 value
            self.bit_ = 1
        return self.getBit()

    def __bool__(self):
        return bool(self.getBit())


