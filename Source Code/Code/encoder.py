from time import ticks_us, ticks_diff   # Use to get dt value in update()
import pyb

# update: Velocity conversion: counts/s to mm/s 
CPR = 1430                # counts per wheel revolution (encoder)
WHEEL_DIAMETER_MM = 70    # wheel diameter
CIRC_MM = 3.1415 * WHEEL_DIAMETER_MM
COUNTS_TO_MM = CIRC_MM / CPR   # ≈ 0.1538 mm per (count/s)

class Encoder:
    '''A quadrature encoder decoding interface encapsulated in a Python class'''

    def __init__(self, tim, chA_pin, chB_pin):
        '''Initializes an Encoder object'''
        self.tim = pyb.Timer(tim, period = 0xFFFF, prescaler = 0)
        self.chA = self.tim.channel(1, pin=chA_pin, mode=pyb.Timer.ENC_AB) 
        self.chB = self.tim.channel(2, pin=chB_pin, mode=pyb.Timer.ENC_AB)
    
        self.position   = 0     # Total accumulated position of the encoder
        self.prev_count = self.tim.counter()     # Counter value from the most recent update
        self.delta      = 0     # Change in count between last two updates
        self.dt         = 0     # Amount of time between last two updates
        self.prev_time  = ticks_us()     # used to keep time for dt
    
    def update(self):
        '''Runs one update step on the encoder's timer counter to keep
           track of the change in count and check for counter reload'''
        count = self.tim.counter()
        
        # AR correction 
        delta = count - self.prev_count
        if delta < - (0xFFFF+1)//2:
            delta = delta + (0xFFFF + 1)
        elif delta > (0xFFFF+1)//2:
            delta = delta - (0xFFFF + 1)
        self.delta = delta

        self.position += delta
        self.prev_count = count

        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_time)

        self.prev_time = now

        pass
            
    def get_position(self):
        '''Returns the most recently updated value of position as determined
           within the update() method'''
        return self.position
            
    def get_velocity(self):
        '''Returns wheel speed in mm/s using the most recently updated delta'''
        if self.dt <= 0:
            return 0

        # counts per second
        counts_per_sec = (self.delta * 1_000_000) / self.dt

        # convert to mm/s
        mm_per_sec = counts_per_sec * COUNTS_TO_MM
        return mm_per_sec
    
    def zero(self):
        '''Sets the present encoder position to zero and causes future updates
           to measure with respect to the new zero position'''
        self.position = 0
        self.prev_count = self.tim.counter()
        pass
