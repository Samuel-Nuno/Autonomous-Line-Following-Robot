from pyb import ADC, Pin


class linesensor:
    # Pololu QTR-MD-08A (8 analog outputs)

    def __init__(self, adc_pins):
        if len(adc_pins) != 8:
            raise ValueError("Need exactly 8 ADC pins")

        self._adcs = [
            ADC(Pin(p)) if not isinstance(p, Pin) else ADC(p)
            for p in adc_pins
        ]

        # Calibration (ADC counts)
        self._white = [4095] * 8
        self._black = [0] * 8
        self._calibrated = False

        # Positions (0..7)
        self._pos = list(range(8))

        # ---- Robustness knobs ----
        # If sum(norm) is below this, treat as "no line" (prevents bogus centroid at startup/off-line)
        self.min_strength = 0.40

        # If a sensor's calibration span is tiny, it will be noisy; ignore it
        self.min_span = 30  # ADC counts; tune 20-150 depending on noise

    def read_raw(self):
        return [adc.read() for adc in self._adcs]

    def calibrate_white(self, n=50):
        acc = [0] * 8
        for _ in range(n):
            r = self.read_raw()
            for i in range(8):
                acc[i] += r[i]
        self._white = [a // n for a in acc]

    def calibrate_black(self, n=50):
        acc = [0] * 8
        for _ in range(n):
            r = self.read_raw()
            for i in range(8):
                acc[i] += r[i]
        self._black = [a // n for a in acc]
        self._calibrated = True

    def norm_from_raw(self, raw):
        """
        Polarity-safe normalization in [0..1] per sensor.
        Works whether black>white OR white>black for each channel.
        """
        out = [0.0] * 8

        for i in range(8):
            w = self._white[i]
            b = self._black[i]
            span = b - w

            if span == 0 or abs(span) < self.min_span:
                out[i] = 0.0
                continue

            # Polarity-safe mapping
            if span > 0:
                x = (raw[i] - w) / span
            else:
                x = (w - raw[i]) / (w - b)

            # Clamp
            if x < 0.0:
                x = 0.0
            elif x > 1.0:
                x = 1.0

            out[i] = x

        return out

    def read_norm(self):
        raw = self.read_raw()
        return self.norm_from_raw(raw)

    def centroid_from_norm(self, w):
        """
        Centroid in [0..7] or None if no confident line.
        """
        if not self._calibrated:
            return None

        s = sum(w)

        # Stronger gating than your old 1e-6
        if s < self.min_strength:
            return None

        num = 0.0
        for i in range(8):
            num += w[i] * self._pos[i]

        return num / s

    def centroid(self):
        raw = self.read_raw()
        w = self.norm_from_raw(raw)
        return self.centroid_from_norm(w)

    # Optional: nice for debugging prints (raw/norm/centroid all from one sample)
    def read_all(self):
        raw = self.read_raw()
        norm = self.norm_from_raw(raw)
        cen = self.centroid_from_norm(norm)
        return raw, norm, cen