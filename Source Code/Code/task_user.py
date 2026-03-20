from pyb import USB_VCP
import micropython
import math

# States
S0_INIT      = micropython.const(0)
S1_PROMPT    = micropython.const(1)
S2_CMD       = micropython.const(2)

S3_SET_KP    = micropython.const(3)
S4_SET_KI    = micropython.const(4)

S5_SET_SP_L  = micropython.const(5)
S6_SET_SP_R  = micropython.const(6)

S7_COLLECT   = micropython.const(7)
S8_DISPLAY   = micropython.const(8)

S9_LF_VEL    = micropython.const(9)

S10_CAL_WHITE = micropython.const(10)
S11_CAL_BLACK = micropython.const(11)


class task_user:
    def __init__(self, leftMotorGo, rightMotorGo, setpointL, setpointR, effL, effR,
                 dataValuesL, dataValuesR,
                 timeValuesL, timeValuesR,
                 Kp_share=None, Ki_share=None,
                 lf_enable_share=None, lf_v_share=None,
                 lf_centroid_queue=None,
                 sensor=None,
                 # estimator outputs (optional)
                 xhat_s=None, xhat_psi=None, xhat_omL=None, xhat_omR=None,
                 # estimator task object (optional, for reset)
                 estimator_task=None):

        self._state = S0_INIT

        # Shares
        self._leftMotorGo  = leftMotorGo
        self._rightMotorGo = rightMotorGo
        self._setpointL    = setpointL
        self._setpointR    = setpointR

        self._effL         = effL
        self._effR         = effR

        self._Kp = Kp_share
        self._Ki = Ki_share

        # Line following shares/queues
        self._lf_en = lf_enable_share
        self._lf_v  = lf_v_share
        self._lf_cq = lf_centroid_queue
        self._lf_logging = False

        # Queues
        self._dataValuesL  = dataValuesL
        self._dataValuesR  = dataValuesR
        self._timeValuesL  = timeValuesL
        self._timeValuesR  = timeValuesR

        # Serial
        self._ser = USB_VCP()

        # UI state
        self._collect_started = False
        self._buf = ""
        self._cmd = None

        # Buffered values entered by user
        self._targetL = 0.0
        self._targetR = 0.0
        self._newKp   = 0.0
        self._newKi   = 0.0

        self._w("User Task object instantiated\r\n")

        self._sensor = sensor

        # estimator shares
        self._xhat_s   = xhat_s
        self._xhat_psi = xhat_psi
        self._xhat_omL = xhat_omL
        self._xhat_omR = xhat_omR

        # estimator task ref for reset
        self._est_task = estimator_task

        # dead-reckoned XY (mm) from (s, psi)
        self._pose_inited = False
        self._s_prev = 0.0
        self._x_mm = 0.0
        self._y_mm = 0.0

    def _w(self, msg):
        if isinstance(msg, str):
            self._ser.write(msg.encode())
        else:
            self._ser.write(msg)

    def _print_menu(self):
        self._w("\r\n+--------------------------------------------------+\r\n")
        self._w("| ME 405 Romi Tuning Interface Help Menu           |\r\n")
        self._w("+---+----------------------------------------------+\r\n")
        self._w("| h | Print help menu                              |\r\n")
        self._w("| c | Calibrate line sensor                        |\r\n")
        self._w("| k | Enter new gain values (Kp, Ki)               |\r\n")
        self._w("| s | Choose a new setpoint (Left, Right)          |\r\n")
        self._w("| l | Toggle line following and set base speed     |\r\n")
        self._w("| r | Read line sensor values (debug)              |\r\n")
        self._w("| g | Trigger step response and print results      |\r\n")
        self._w("| x | EMERGENCY STOP (motors off + dump centroid)  |\r\n")
        self._w("| p | Print estimated pose (s, psi, x, y)          |\r\n")
        self._w("| z | Zero pose (reset observer + x/y)             |\r\n")
        self._w("+---+----------------------------------------------+\r\n")

    def _read_cmd_nonblocking(self):
        while self._ser.any():
            b = self._ser.read(1)
            if not b:
                return None
            try:
                ch = b.decode()
            except:
                continue

            self._w(ch)
            if ch in ("\r", "\n"):
                continue
            return ch
        return None

    def _read_float_nonblocking(self):
        digits = "0123456789"
        term = ("\r", "\n")

        while self._ser.any():
            b = self._ser.read(1)
            if not b:
                return None
            try:
                ch = b.decode()
            except:
                continue

            self._w(ch)

            if ch in digits:
                self._buf += ch
            elif ch == "." and "." not in self._buf:
                self._buf += ch
            elif ch == "-" and len(self._buf) == 0:
                self._buf += ch
            elif ch in ("\b", "\x7f"):
                if len(self._buf) > 0:
                    self._buf = self._buf[:-1]
            elif ch in term:
                s = self._buf.strip()
                self._buf = ""
                if len(s) == 0 or s in ("-", "."):
                    return "BAD"
                try:
                    return float(s)
                except:
                    return "BAD"

        return None

    def _print_run_header(self):
        kp_txt = "n/a"
        ki_txt = "n/a"
        try:
            if self._Kp is not None:
                kp_txt = "{:.6g}".format(self._Kp.get())
        except:
            pass
        try:
            if self._Ki is not None:
                ki_txt = "{:.6g}".format(self._Ki.get())
        except:
            pass

        self._w("\r\n--------------------------------------------------\r\n")
        self._w("Setpoint L: {:g} mm/s\r\n".format(self._targetL))
        self._w("Setpoint R: {:g} mm/s\r\n".format(self._targetR))
        self._w("Kp:         {}\r\n".format(kp_txt))
        self._w("Ki:         {}\r\n".format(ki_txt))
        self._w("--------------------------------------------------\r\n")
        self._w("Time [s],Left,Right\r\n")

    def _update_pose_once(self):
        if (self._xhat_s is None) or (self._xhat_psi is None):
            return
        try:
            s_now = float(self._xhat_s.get())
            psi = float(self._xhat_psi.get())
        except:
            return

        if not self._pose_inited:
            self._s_prev = s_now
            self._x_mm = 0.0
            self._y_mm = 0.0
            self._pose_inited = True
            return

        ds = s_now - self._s_prev
        self._s_prev = s_now
        self._x_mm += ds * math.cos(psi)
        self._y_mm += ds * math.sin(psi)

    def run(self):
        while True:
            self._update_pose_once()

            if self._state == S0_INIT:
                self._print_menu()
                self._w("\r\n>: ")
                self._state = S2_CMD

            elif self._state == S2_CMD:
                ch = self._read_cmd_nonblocking()
                if ch is not None:
                    self._cmd = ch.lower()

                    if self._cmd == "h":
                        self._print_menu()
                        self._w("\r\n>: ")

                    elif self._cmd == "c":
                        if self._sensor is None:
                            self._w("\r\nNo sensor connected.\r\n>: ")
                        else:
                            self._w("\r\nPlace robot on WHITE background and press Enter...\r\n")
                            self._state = S10_CAL_WHITE

                    elif self._cmd == "k":
                        if (self._Kp is None) or (self._Ki is None):
                            self._w("\r\n(No Kp/Ki shares connected in main.py)\r\n>: ")
                        else:
                            self._w("\r\nEnter proportional gain, Kp:\r\n>: ")
                            self._buf = ""
                            self._state = S3_SET_KP

                    elif self._cmd == "s":
                        self._w("\r\nEnter Left setpoint (mm/s):\r\n>: ")
                        self._buf = ""
                        self._state = S5_SET_SP_L

                    elif self._cmd == "g":
                        # Read current setpoints from shares
                        self._targetL = self._setpointL.get()
                        self._targetR = self._setpointR.get()

                        # Prevent a step to zero that looks like nothing is happening
                        if (abs(self._targetL) < 1e-6) and (abs(self._targetR) < 1e-6):
                            self._w("\r\nSetpoints are 0. Use 's' to enter nonzero setpoints, then press 'g'.\r\n>: ")
                            self._state = S2_CMD
                        else:
                            self._setpointL.put(self._targetL)
                            self._setpointR.put(self._targetR)
                            self._w("\r\nStep response triggered...\r\n")
                            self._state = S7_COLLECT

                    elif self._cmd == "l":
                        en = self._lf_en.get() if self._lf_en is not None else 0
                        if en:
                            if self._lf_en is not None:
                                self._lf_en.put(0)

                            self._leftMotorGo.put(0)
                            self._rightMotorGo.put(0)
                            self._setpointL.put(0)
                            self._setpointR.put(0)
                            self._effL.put(0)
                            self._effR.put(0)

                            self._w("\r\nLine following DISABLED.\r\n>: ")
                        else:
                            self._w("\r\nEnter line-follow base speed (mm/s):\r\n>: ")
                            self._buf = ""
                            self._state = S9_LF_VEL

                    elif self._cmd == "r":
                        if self._sensor is None:
                            self._w("\r\nNo sensor attached.\r\n>: ")
                        else:
                            raw  = self._sensor.read_raw()
                            norm = self._sensor.read_norm()
                            cen  = self._sensor.centroid()

                            self._w("\r\nRaw:   " + ", ".join(str(v) for v in raw) + "\r\n")
                            self._w("Norm:  " + ", ".join("{:.2f}".format(v) for v in norm) + "\r\n")
                            if cen is None:
                                self._w("Centroid: LOST\r\n")
                            else:
                                self._w("Centroid: {:.2f}\r\n".format(cen))
                            self._w(">: ")

                    elif self._cmd == "x":
                        self._w("\r\n[UI] EMERGENCY STOP command received.\r\n")

                        if self._lf_en is not None:
                            self._lf_en.put(0)

                        self._leftMotorGo.put(0)
                        self._rightMotorGo.put(0)

                        self._setpointL.put(0)
                        self._setpointR.put(0)
                        self._effL.put(0)
                        self._effR.put(0)

                        self._lf_logging = False
                        self._collect_started = False

                        self._w("EMERGENCY STOP: Motors OFF\r\n")
                        self._w("Time [s],Centroid\r\n")

                        if (self._timeValuesL is not None) and (self._lf_cq is not None):
                            while self._timeValuesL.any() and self._lf_cq.any():
                                t_us = self._timeValuesL.get()
                                c    = self._lf_cq.get()
                                t_s  = t_us / 1_000_000.0
                                self._w("{:.6f},{}\r\n".format(t_s, c))

                        self._w(">: ")

                    elif self._cmd == "p":
                        if (self._xhat_s is None) or (self._xhat_psi is None):
                            self._w("\r\n(No estimator shares connected in main.py)\r\n>: ")
                        else:
                            try:
                                s_now = float(self._xhat_s.get())
                            except:
                                s_now = 0.0
                            try:
                                psi = float(self._xhat_psi.get())
                            except:
                                psi = 0.0

                            self._w("\r\nEstimated pose:\r\n")
                            self._w("  s   = {:.1f} mm\r\n".format(s_now))
                            self._w("  psi = {:.3f} rad\r\n".format(psi))
                            self._w("  x   = {:.1f} mm\r\n".format(self._x_mm))
                            self._w("  y   = {:.1f} mm\r\n".format(self._y_mm))
                            self._w(">: ")

                    elif self._cmd == "z":
                        # Reset the observer state + reset local x/y accumulator
                        if self._est_task is not None:
                            try:
                                self._est_task.reset()
                            except:
                                pass

                        self._pose_inited = False
                        self._s_prev = 0.0
                        self._x_mm = 0.0
                        self._y_mm = 0.0

                        self._w("\r\nPose reset to zero.\r\n>: ")

                    else:
                        self._w("\r\nUnknown command. Type 'h' for help.\r\n>: ")

            elif self._state == S3_SET_KP:
                val = self._read_float_nonblocking()
                if val == "BAD":
                    self._w("\r\nInvalid number. Try again.\r\n>: ")
                    self._buf = ""
                elif val is not None:
                    self._newKp = val
                    self._Kp.put(self._newKp)
                    self._w("\r\nKp set to {}\r\n".format(self._newKp))
                    self._w("\r\nEnter integral gain, Ki:\r\n>: ")
                    self._buf = ""
                    self._state = S4_SET_KI

            elif self._state == S4_SET_KI:
                val = self._read_float_nonblocking()
                if val == "BAD":
                    self._w("\r\nInvalid number. Try again.\r\n>: ")
                    self._buf = ""
                elif val is not None:
                    self._newKi = val
                    self._Ki.put(self._newKi)
                    self._w("\r\nKi set to {}\r\n".format(self._newKi))
                    self._w("\r\n>: ")
                    self._state = S2_CMD

            elif self._state == S5_SET_SP_L:
                val = self._read_float_nonblocking()
                if val == "BAD":
                    self._w("\r\nInvalid number. Try again.\r\n>: ")
                    self._buf = ""
                elif val is not None:
                    self._targetL = val
                    self._w("\r\nLeft setpoint buffered: {}\r\n".format(self._targetL))
                    self._w("\r\nEnter Right setpoint (mm/s):\r\n>: ")
                    self._buf = ""
                    self._state = S6_SET_SP_R

            elif self._state == S6_SET_SP_R:
                val = self._read_float_nonblocking()
                if val == "BAD":
                    self._w("\r\nInvalid number. Try again.\r\n>: ")
                    self._buf = ""
                elif val is not None:
                    self._targetR = val
                    self._w("\r\nRight setpoint buffered: {}\r\n".format(self._targetR))
                    self._setpointL.put(self._targetL)
                    self._setpointR.put(self._targetR)
                    self._w("\r\nSetpoints updated.\r\n>: ")
                    self._state = S2_CMD

            elif self._state == S7_COLLECT:
                if not self._collect_started:
                    self._dataValuesL.clear()
                    self._dataValuesR.clear()
                    self._timeValuesL.clear()
                    self._timeValuesR.clear()

                    if self._lf_cq is not None:
                        self._lf_cq.clear()

                    self._lf_logging = (
                        (self._lf_en is not None)
                        and (self._lf_en.get() != 0)
                        and (self._lf_cq is not None)
                    )

                    self._setpointL.put(self._targetL)
                    self._setpointR.put(self._targetR)

                    self._leftMotorGo.put(1)
                    self._rightMotorGo.put(1)

                    self._w("Data collecting...\r\n")
                    self._collect_started = True

                # auto-stop when buffers fill
                buffers_full = (
                    (self._dataValuesL is not None and self._dataValuesL.full()) or
                    (self._timeValuesL is not None and self._timeValuesL.full()) or
                    (self._dataValuesR is not None and self._dataValuesR.full()) or
                    (self._timeValuesR is not None and self._timeValuesR.full())
                )

                if buffers_full:
                    # ensure motors stopped so printing can proceed
                    self._leftMotorGo.put(0)
                    self._rightMotorGo.put(0)

                # End when motors stopped (external) or because buffers filled
                if ((not self._leftMotorGo.get()) and (not self._rightMotorGo.get())) or buffers_full:
                    self._w("Step response complete, printing data...\r\n")
                    self._print_run_header()
                    self._state = S8_DISPLAY
                    self._collect_started = False

            elif self._state == S8_DISPLAY:
                if (self._dataValuesL.any()
                        and self._dataValuesR.any()
                        and self._timeValuesL.any()):
                    t_us = self._timeValuesL.get()
                    vL   = self._dataValuesL.get()
                    vR   = self._dataValuesR.get()
                    t_s = t_us / 1_000_000.0

                    if self._lf_logging and (self._lf_cq is not None) and self._lf_cq.any():
                        c = self._lf_cq.get()
                        self._w("{:.6f},{},{},{}\r\n".format(t_s, vL, vR, c))
                    else:
                        self._w("{:.6f},{},{}\r\n".format(t_s, vL, vR))

            elif self._state == S9_LF_VEL:
                val = self._read_float_nonblocking()
                if val == "BAD":
                    self._w("\r\nInvalid number. Try again.\r\n>: ")
                    self._buf = ""
                    self._state = S2_CMD

                elif val is not None:
                    v = val

                    self._timeValuesL.clear()
                    if self._lf_cq is not None:
                        self._lf_cq.clear()

                    if self._lf_v is not None:
                        self._lf_v.put(v)

                    if self._lf_en is not None:
                        self._lf_en.put(1)
                    self._leftMotorGo.put(1)
                    self._rightMotorGo.put(1)

                    self._w("\r\nLine following ENABLED at {:.1f} mm/s.\r\n>: ".format(v))
                    self._state = S2_CMD

            elif self._state == S10_CAL_WHITE:
                if self._ser.any():
                    _ = self._ser.read()
                    self._w("\r\nCalibrating WHITE...\r\n")
                    self._sensor.calibrate_white(100)
                    self._w("White calibration complete.\r\n")
                    self._w("\r\nNow place robot on BLACK line and press Enter...\r\n")
                    self._state = S11_CAL_BLACK

            elif self._state == S11_CAL_BLACK:
                if self._ser.any():
                    _ = self._ser.read()
                    self._w("\r\nCalibrating BLACK...\r\n")
                    self._sensor.calibrate_black(100)
                    self._w("Black calibration complete.\r\n")
                    self._w("\r\nCalibration DONE.\r\n>: ")
                    self._state = S2_CMD

            yield self._state
