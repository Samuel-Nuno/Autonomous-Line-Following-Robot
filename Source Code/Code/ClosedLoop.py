import micropython

S0_INIT = micropython.const(0)
S1_CMP  = micropython.const(1)

class Closed_Loop:

    def __init__(self, spL_share, spR_share, vL_share, vR_share,
                 effL_share, effR_share, Kp_share, Ki_share,
                 lf_enable_share=None,
                 lf_v_share=None,
                 lf_centroid_share=None,
                 bump_share=None,
                 posL_share=None,
                 posR_share=None,
                 lf_deadband=0.5,
                 lf_dv_turn=1000,
                 lf_lost_val=-1):

        self._state = S0_INIT

        self._spL  = spL_share
        self._spR  = spR_share
        self._vL   = vL_share
        self._vR   = vR_share
        self._effL = effL_share
        self._effR = effR_share
        self._Kp   = Kp_share
        self._Ki   = Ki_share

        self._SAT = 100
        self._MIN_EFF = 0

        self._ERR_EPS = 5
        self._SP_EPS  = 10

        self._intL = 0.0
        self._intR = 0.0
        self.dt = 0.020

        self._last_spL = 0.0
        self._last_spR = 0.0

        # Line follow shares
        self._lf_en = lf_enable_share
        self._lf_v  = lf_v_share
        self._lf_c  = lf_centroid_share
        self._bump  = bump_share

        # Encoder position shares
        self._posL = posL_share
        self._posR = posR_share

        self._lf_center   = 4
        self._lf_deadband = float(lf_deadband)
        self._lf_dv_turn  = float(lf_dv_turn)
        self._lf_lost_val = lf_lost_val

        # Steering controller
        self._lf_Kp = 30.0
        self._lf_Ki = 0.0
        self._lf_int = 0.0
        self._lf_int_decay = 0.2
        self._lf_dv_slew = 180.0
        self._lf_last_dv = 0.0

        self._lf_need_lock = True
        self._lf_lock_count = 0
        self._lf_lock_required = 1
        self._lf_prev_enabled = 0

        self._cp2_straight_kp = 0.20
        self._cp2_straight_max = 20.0
        self._cp2_right_trim = -4.0

        # Garage / CP2 / CP4 script states
        # 0 = normal line follow
        # 1 = square-up forward
        # 2 = forward into garage
        # 3 = right turn 90 deg in garage
        # 4 = forward to wall with encoder straightening
        # 5 = hardcoded left turn at wall
        # 6 = drive straight from garage exit toward CP#2
        # 7 = scripted right turn at CP#2
        # 8 = slow forward reacquire onto slalom line
        # 9 = CP#4 short forward into turnaround
        # 10 = CP#4 rotate 180 deg
        # 11 = CP#4 slow forward reacquire onto return line
        # 12 = CP#4 small right bias to choose path toward CP#5
        self._garage_state = 0

        self._garage_lost_count = 0
        self._garage_lost_needed = 6   

        self._seg_start_L = 0
        self._seg_start_R = 0

        # Garage tuning
        self._garage_squareup_counts = 325
        self._garage_fwd1_counts = 1015
        self._garage_wall_max_counts = 4062
        self._garage_turn90_counts = 815

        self._garage_fwd_speed = 150.0
        self._garage_turn_speed = 85.0

        # Straight-to-wall correction
        self._garage_straight_kp = 0.20
        self._garage_straight_max = 18.0
        self._garage_right_trim = -4.0

        # Hardcoded left turn at the wall
        self._garage_left_turn_speed = 80.0
        self._garage_left_turn_counts = 780

        # CP#2 scripted move
        self._cp2_fwd_counts = 2500
        self._cp2_fwd_speed = 95.0

        self._cp2_turn_counts = 770
        self._cp2_turn_speed = 80.0

        self._cp2_reacquire_speed = 75.0
        self._cp2_reacquire_max_counts = 700
        self._cp2_seen_count = 0
        self._cp2_seen_needed = 3

        # CP#4 turnaround (180 deg turn)
        self._cp4_fwd_counts = 250
        self._cp4_turn180_counts = 1500
        self._cp4_turn_speed = 85.0
        self._cp4_reacquire_speed = 75.0
        self._cp4_reacquire_max_counts = 700
        self._cp4_seen_count = 0
        self._cp4_seen_needed = 3

        # CP#4 toward CP#5
        self._cp4_right_bias_counts = 200
        self._cp4_right_bias_speed = 75.0
        self._cp4_right_bias_fwd_counts = 500

        # Course progress flags 
        self._garage_done = False
        self._cp2_done = False
        self._cp4_done = False

    def _slew(self, target, last, step):
        if target > last + step:
            return last + step
        if target < last - step:
            return last - step
        return target

    def _reset_line_lock(self):
        self._lf_need_lock = True
        self._lf_lock_count = 0
        self._lf_last_dv = 0.0
        self._lf_int = 0.0

    def _get_posL(self):
        if self._posL is None:
            return 0
        try:
            return int(self._posL.get())
        except:
            return 0

    def _get_posR(self):
        if self._posR is None:
            return 0
        try:
            return int(self._posR.get())
        except:
            return 0

    def _mark_segment_start(self):
        self._seg_start_L = self._get_posL()
        self._seg_start_R = self._get_posR()

    def _segment_counts(self):
        dL = abs(self._get_posL() - self._seg_start_L)
        dR = abs(self._get_posR() - self._seg_start_R)
        return 0.5 * (dL + dR)

    def _segment_lr_counts(self):
        dL = abs(self._get_posL() - self._seg_start_L)
        dR = abs(self._get_posR() - self._seg_start_R)
        return dL, dR

    def _start_garage_script(self):
        self._garage_state = 1
        self._garage_lost_count = 0
        self._cp2_seen_count = 0
        self._mark_segment_start()

        self._reset_line_lock()
        self._intL = 0.0
        self._intR = 0.0

    def _start_cp2_script(self):
        self._cp2_seen_count = 0
        self._garage_state = 6
        self._mark_segment_start()

        self._reset_line_lock()
        self._intL = 0.0
        self._intR = 0.0

    def _start_cp4_script(self):
        self._cp4_seen_count = 0
        self._garage_state = 9
        self._mark_segment_start()

        self._reset_line_lock()
        self._intL = 0.0
        self._intR = 0.0

    def _exit_garage_to_line_follow(self):
        self._garage_state = 0
        self._garage_lost_count = 0
        self._cp2_seen_count = 0
        self._cp4_seen_count = 0
        self._mark_segment_start()

        self._reset_line_lock()
        self._intL = 0.0
        self._intR = 0.0

    def run(self):
        while True:

            if self._state == S0_INIT:
                self._intL = 0.0
                self._intR = 0.0
                self._last_spL = 0.0
                self._last_spR = 0.0
                self._effL.put(0)
                self._effR.put(0)
                self._state = S1_CMP

                self._reset_line_lock()
                self._lf_prev_enabled = 0
                self._garage_state = 0
                self._garage_lost_count = 0
                self._cp2_seen_count = 0
                self._cp4_seen_count = 0
                self._garage_done = False
                self._cp2_done = False
                self._cp4_done = False

            elif self._state == S1_CMP:

                use_line_follow = (
                    (self._lf_en is not None) and
                    (self._lf_v  is not None) and
                    (self._lf_c  is not None) and
                    (self._lf_en.get() != 0)
                )

                lf_now = 1 if use_line_follow else 0
                if lf_now == 1 and self._lf_prev_enabled == 0:
                    # Full course reset on each new user button press
                    self._reset_line_lock()

                    self._garage_state = 0
                    self._garage_lost_count = 0
                    self._cp2_seen_count = 0
                    self._cp4_seen_count = 0

                    self._garage_done = False
                    self._cp2_done = False
                    self._cp4_done = False

                    self._mark_segment_start()

                    self._intL = 0.0
                    self._intR = 0.0
                    self._last_spL = 0.0
                    self._last_spR = 0.0

                    if self._spL is not None:
                        self._spL.put(0)
                    if self._spR is not None:
                        self._spR.put(0)
                    if self._effL is not None:
                        self._effL.put(0)
                    if self._effR is not None:
                        self._effR.put(0)

                self._lf_prev_enabled = lf_now

                if not use_line_follow:
                    spL = self._spL.get()
                    spR = self._spR.get()

                else:
                    v = float(self._lf_v.get())
                    c = float(self._lf_c.get())

                    bump_pressed = False
                    if self._bump is not None:
                        bump_pressed = (self._bump.get() != 0)

                    # Scripted states 
                    if self._garage_state == 1:
                        # square-up forward
                        spL = int(self._garage_fwd_speed)
                        spR = int(self._garage_fwd_speed)

                        if self._segment_counts() >= self._garage_squareup_counts:
                            self._garage_state = 2
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 2:
                        # forward into garage
                        spL = int(self._garage_fwd_speed)
                        spR = int(self._garage_fwd_speed)

                        if self._segment_counts() >= self._garage_fwd1_counts:
                            self._garage_state = 3
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 3:
                        # right turn 90 deg in place
                        spL = int(self._garage_turn_speed)
                        spR = -int(self._garage_turn_speed)

                        if self._segment_counts() >= self._garage_turn90_counts:
                            self._garage_state = 4
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 4:
                        # forward to wall with encoder straightening
                        dL, dR = self._segment_lr_counts()
                        e_straight = dL - dR

                        corr = self._garage_straight_kp * e_straight
                        if corr > self._garage_straight_max:
                            corr = self._garage_straight_max
                        elif corr < -self._garage_straight_max:
                            corr = -self._garage_straight_max

                        base = self._garage_fwd_speed
                        spL = int(base - corr)
                        spR = int(base + corr + self._garage_right_trim)

                        if bump_pressed or (self._segment_counts() >= self._garage_wall_max_counts):
                            self._garage_state = 5
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 5:
                        # hardcoded left turn at the wall
                        spL = -int(self._garage_left_turn_speed)
                        spR =  int(self._garage_left_turn_speed)

                        if self._segment_counts() >= self._garage_left_turn_counts:
                            self._start_cp2_script()
                            spL = 0
                            spR = 0

                    elif self._garage_state == 6:
                        # drive straight using encoder correction 
                        dL, dR = self._segment_lr_counts()
                        e_straight = dL - dR

                        corr = self._cp2_straight_kp * e_straight

                        if corr > self._cp2_straight_max:
                            corr = self._cp2_straight_max
                        elif corr < -self._cp2_straight_max:
                            corr = -self._cp2_straight_max

                        base = self._cp2_fwd_speed

                        spL = int(base - corr)
                        spR = int(base + corr + self._cp2_right_trim)

                        if self._segment_counts() >= self._cp2_fwd_counts:
                            self._garage_state = 7
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 7:
                        # right turn at CP#2
                        spL = int(self._cp2_turn_speed)
                        spR = -int(self._cp2_turn_speed)

                        if self._segment_counts() >= self._cp2_turn_counts:
                            self._garage_state = 8
                            self._cp2_seen_count = 0
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 8:
                        # slow forward reacquire onto slalom line
                        spL = int(self._cp2_reacquire_speed)
                        spR = int(self._cp2_reacquire_speed)

                        if c != self._lf_lost_val:
                            self._cp2_seen_count += 1
                        else:
                            self._cp2_seen_count = 0

                        if self._cp2_seen_count >= self._cp2_seen_needed:
                            if self._lf_v is not None:
                                self._lf_v.put(105.0)   # slalom speed
                            self._garage_done = True
                            self._cp2_done = True
                            self._exit_garage_to_line_follow()
                            spL = 0
                            spR = 0

                        elif self._segment_counts() >= self._cp2_reacquire_max_counts:
                            spL = 0
                            spR = 0

                    elif self._garage_state == 9:
                        # CP#4 short forward into turnaround
                        spL = int(self._cp4_reacquire_speed)
                        spR = int(self._cp4_reacquire_speed)

                        if self._segment_counts() >= self._cp4_fwd_counts:
                            self._garage_state = 10
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 10:
                        # CP#4 rotate 180 degrees
                        spL = int(self._cp4_turn_speed)
                        spR = -int(self._cp4_turn_speed)

                        if self._segment_counts() >= self._cp4_turn180_counts:
                            self._garage_state = 11
                            self._cp4_seen_count = 0
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0

                    elif self._garage_state == 11:
                        # CP#4 slow forward reacquire onto return line
                        spL = int(self._cp4_reacquire_speed)
                        spR = int(self._cp4_reacquire_speed)

                        if c != self._lf_lost_val:
                            self._cp4_seen_count += 1
                        else:
                            self._cp4_seen_count = 0

                        if self._cp4_seen_count >= self._cp4_seen_needed:
                            self._garage_state = 12
                            self._mark_segment_start()
                            self._intL = 0.0
                            self._intR = 0.0
                            spL = 0
                            spR = 0

                        elif self._segment_counts() >= self._cp4_reacquire_max_counts:
                            spL = 0
                            spR = 0

                    elif self._garage_state == 12:
                        # CP#4: go forward a little, then arc gently right toward CP#5
                        seg_counts = self._segment_counts()

                        if seg_counts < self._cp4_right_bias_fwd_counts:
                            # short forward move first
                            spL = int(self._cp4_reacquire_speed)
                            spR = int(self._cp4_reacquire_speed)

                        else:
                            # gentle forward-right arc
                            spL = int(self._cp4_right_bias_speed)
                            spR = int(0.65 * self._cp4_right_bias_speed)

                        if seg_counts >= (self._cp4_right_bias_fwd_counts + self._cp4_right_bias_counts):
                            self._cp4_done = True
                            self._exit_garage_to_line_follow()
                            spL = 0
                            spR = 0

                    else:
                        # Normal line follow
                        if c == self._lf_lost_val:
                            self._garage_lost_count += 1

                            if self._garage_lost_count >= self._garage_lost_needed:
                                if not self._garage_done:
                                    self._start_garage_script()
                                    spL = int(self._garage_fwd_speed)
                                    spR = int(self._garage_fwd_speed)
                                elif self._cp2_done and not self._cp4_done:
                                    self._start_cp4_script()
                                    spL = int(self._cp4_reacquire_speed)
                                    spR = int(self._cp4_reacquire_speed)
                                else:
                                    self._reset_line_lock()
                                    spL = int(0.6 * v)
                                    spR = int(0.6 * v)
                            else:
                                self._reset_line_lock()
                                spL = int(0.6 * v)
                                spR = int(0.6 * v)

                        else:
                            self._garage_lost_count = 0
                            e = c - self._lf_center

                            if self._lf_need_lock:
                                self._lf_lock_count += 1
                                if self._lf_lock_count >= self._lf_lock_required:
                                    self._lf_need_lock = False
                                    self._lf_last_dv = 0.0
                                    self._lf_int = 0.0
                                dv_target = 0.0

                            else:
                                if abs(e) < self._lf_deadband:
                                    dv_target = 0.0
                                    if self._lf_int_decay > 0.0:
                                        self._lf_int *= (1.0 - self._lf_int_decay)
                                else:
                                    self._lf_int += e * self.dt
                                    if self._lf_Ki > 0.0:
                                        i_max = self._lf_dv_turn / self._lf_Ki
                                        if self._lf_int >  i_max:
                                            self._lf_int =  i_max
                                        if self._lf_int < -i_max:
                                            self._lf_int = -i_max
                                        dv_target = -(self._lf_Kp * e + self._lf_Ki * self._lf_int)
                                    else:
                                        dv_target = -(self._lf_Kp * e)

                            if dv_target > self._lf_dv_turn:
                                dv_target = self._lf_dv_turn
                            elif dv_target < -self._lf_dv_turn:
                                dv_target = -self._lf_dv_turn

                            dv = self._slew(dv_target, self._lf_last_dv, self._lf_dv_slew)
                            self._lf_last_dv = dv

                            spL = int(v - dv)
                            spR = int(v + dv)

                    self._spL.put(spL)
                    self._spR.put(spR)

                # Motor velocity PI control 
                vL = self._vL.get()
                vR = self._vR.get()

                Kp = self._Kp.get()
                Ki = self._Ki.get()

                if (spL == 0 and self._last_spL != 0) or (spL * self._last_spL < 0) or (abs(spL - self._last_spL) > 500):
                    self._intL = 0.0
                if (spR == 0 and self._last_spR != 0) or (spR * self._last_spR < 0) or (abs(spR - self._last_spR) > 500):
                    self._intR = 0.0

                self._last_spL = spL
                self._last_spR = spR

                errL = spL - vL
                errR = spR - vR

                if abs(spL) < self._SP_EPS:
                    self._intL = 0.0
                    effL = 0.0
                else:
                    if abs(errL) > self._ERR_EPS:
                        self._intL += errL * self.dt
                    effL = Kp * errL + Ki * self._intL

                if abs(spR) < self._SP_EPS:
                    self._intR = 0.0
                    effR = 0.0
                else:
                    if abs(errR) > self._ERR_EPS:
                        self._intR += errR * self.dt
                    effR = Kp * errR + Ki * self._intR

                if Ki != 0:
                    i_max = self._SAT / abs(Ki)
                    if self._intL >  i_max:
                        self._intL =  i_max
                    if self._intL < -i_max:
                        self._intL = -i_max
                    if self._intR >  i_max:
                        self._intR =  i_max
                    if self._intR < -i_max:
                        self._intR = -i_max

                if effL > self._SAT:
                    effL = self._SAT
                if effL < -self._SAT:
                    effL = -self._SAT
                if effR > self._SAT:
                    effR = self._SAT
                if effR < -self._SAT:
                    effR = -self._SAT

                self._effL.put(int(effL))
                self._effR.put(int(effR))

            yield self._state
