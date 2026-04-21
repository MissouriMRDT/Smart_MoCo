import can
import struct
import time
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.widgets import CheckButtons
import tkinter.font as tkFont
import multiprocessing
import serial.tools.list_ports

import numpy as np

SMOCO_WIDTH_DID = 5
SMOCO_WIDTH_MID = 6

SMOCO_MID_POSITION = 0x30
SMOCO_MID_POSITION_CALIBRATED = 0x31
SMOCO_MID_ERROR = 0x32
SMOCO_MID_ECHO_REPLY = 0x3F
SMOCO_MID_STOP = 0x00
SMOCO_MID_RAMP_RATE = 0x01
SMOCO_MID_PI = 0x02
SMOCO_MID_D = 0x03
SMOCO_MID_IGNORE_LIMIT = 0x04
SMOCO_MID_SOFT_LIMIT = 0x05
SMOCO_MID_CALIBRATE = 0x06
SMOCO_MID_DEBUG = 0x07
SMOCO_MID_DUTY_CYCLE_RANGE = 0x08
SMOCO_MID_ECHO_REQUEST = 0x0F
SMOCO_MID_OPEN_LOOP = 0x10
SMOCO_MID_TARGET_POSITION = 0x11
SMOCO_MID_TARGET_VELOCITY = 0x12
SMOCO_MID_TARGET_CURRENT = 0x13

SMOCO_WIDTH = {
    SMOCO_MID_POSITION: 8,
    SMOCO_MID_POSITION_CALIBRATED: 0,
    SMOCO_MID_ERROR: 1,
    SMOCO_MID_ECHO_REPLY: 8,
    SMOCO_MID_STOP: 0,
    SMOCO_MID_RAMP_RATE: 4,
    SMOCO_MID_PI: 8,
    SMOCO_MID_D: 4,
    SMOCO_MID_IGNORE_LIMIT: 1,
    SMOCO_MID_SOFT_LIMIT: 8,
    SMOCO_MID_CALIBRATE: 6,
    SMOCO_MID_DEBUG: 1,
    SMOCO_MID_DUTY_CYCLE_RANGE: 8,
    SMOCO_MID_ECHO_REQUEST: 8,
    SMOCO_MID_OPEN_LOOP: 2,
    SMOCO_MID_TARGET_POSITION: 6,
    SMOCO_MID_TARGET_VELOCITY: 6,
    SMOCO_MID_TARGET_CURRENT: 4,
}

SMOCO_FORMAT = {
    SMOCO_MID_POSITION: "<ihBB",
    SMOCO_MID_POSITION_CALIBRATED: "",
    SMOCO_MID_ERROR: "<B",
    SMOCO_MID_ECHO_REPLY: "<Q",
    SMOCO_MID_STOP: "",
    SMOCO_MID_RAMP_RATE: "<f",
    SMOCO_MID_PI: "<ff",
    SMOCO_MID_D: "<f",
    SMOCO_MID_IGNORE_LIMIT: "<B",
    SMOCO_MID_SOFT_LIMIT: "<ii",
    SMOCO_MID_CALIBRATE: "<hi",
    SMOCO_MID_DEBUG: "<B",
    SMOCO_MID_DUTY_CYCLE_RANGE: "<hhhh",
    SMOCO_MID_ECHO_REQUEST: "<Q",
    SMOCO_MID_OPEN_LOOP: "<h",
    SMOCO_MID_TARGET_POSITION: "<hi",
    SMOCO_MID_TARGET_VELOCITY: "<hf",
    SMOCO_MID_TARGET_CURRENT: "<hh",
}

SMOCO_ID_DEBUG = 0x7F0
SMOCO_WIDTH_DEBUG = 4

DEBUG_DATA_LABELS = [
    "Position",
    "Velocity",
    "Current",
    "P Out",
    "I Out",
    "D Out",
    "Error",
    "DeltaT",
    "Target",
]

DEBUG_DATA_FORMATS = [
    ",-r",
    ",-g",
    ",-b",
    ",:r",
    ",:g",
    ",:b",
    ",--r",
    ",--g",
    ",--b",
]

AX1_SERIES = [3, 4, 5, 6]


def label(text, bg, r, c, rs=1, cs=1):
    if bg is None:
        return tk.Label(text=text, anchor="e").grid(
            row=r, column=c, rowspan=rs, columnspan=cs, sticky="nsew"
        )
    return tk.Label(text=text, bg=bg, anchor="e").grid(
        row=r, column=c, rowspan=rs, columnspan=cs, sticky="nsew"
    )

can_active = False

class App(tk.Frame):
    def __init__(
        self, master, can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue
    ):
        color = [
            "#71ff61",
            "#a3a3a3",
            "#787878",
            "#ffffff",
            "#a3a3a3",
            "#787878",
            "#ffffff",
            "#a3a3a3",
            "#787878",
            "#6785FF",
        ]

        super().__init__(master)
        self.can_send = can_send
        self.can_recv = can_recv
        font = tkFont.nametofont("TkFixedFont")
        font.configure(size=10)
        self.option_add("*Font", font)

        self.image = tk.PhotoImage(file="logo.png")
        tk.Label(image=self.image).grid(
            row=0, column=0, rowspan=2, columnspan=2, sticky="nw"
 
        )
        tk.Label(
            text="Developed by Brendan Westley\nfor testing and configuring the\nSmart Motor Controller.\nMars Rover Design Team 2026"
        ).grid(row=0, column=2, rowspan=1, columnspan=2, sticky="nsew")
        self.serial_port = tk.StringVar(value="Select Serial Port")
        self.serial_dropdown = tk.OptionMenu(master, self.serial_port, "Select Serial Port", *serial.tools.list_ports.comports(), command=self.open_serial)
        self.serial_dropdown.grid(
            row=1, column=2, rowspan=1, columnspan=2, sticky="nw"
        )
        label("ID 0x", color[0], 0, 4)
        self.id = tk.StringVar(value="0B")
        tk.Entry(textvariable=self.id, bg=color[0]).grid(row=0, column=5, sticky="nsew")
        tk.Button(text="Stop and Reset", command=self.send_stop, bg=color[0]).grid(
            row=0, column=6, sticky="nsew"
        )
        label("Ramp Rate (1/s)", color[1], 1, 4)
        self.ramp_rate = tk.DoubleVar(value=1.0)
        tk.Entry(textvariable=self.ramp_rate, bg=color[1]).grid(
            row=1, column=5, sticky="nsew"
        )
        tk.Button(text="Set Ramp Rate", command=self.send_ramp_rate, bg=color[1]).grid(
            row=1, column=6, sticky="nsew"
        )
        label("P", color[2], 2, 0)
        self.p = tk.DoubleVar(value=0.7)
        tk.Entry(textvariable=self.p, bg=color[2]).grid(row=2, column=1, sticky="nsew")
        label("I", color[2], 2, 2)
        self.i = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.i, bg=color[2]).grid(row=2, column=3, sticky="nsew")
        label("D", color[2], 2, 4)
        self.d = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.d, bg=color[2]).grid(row=2, column=5, sticky="nsew")
        tk.Button(text="Set PID", command=self.send_pid, bg=color[2]).grid(
            row=2, column=6, sticky="nsew"
        )
        label("Fwd Max Min", color[3], 3, 0)
        self.fwd_max = tk.IntVar(value=2**15 - 1)
        tk.Entry(textvariable=self.fwd_max, bg=color[3]).grid(
            row=3, column=1, sticky="nsew"
        )
        self.fwd_min = tk.IntVar(value=0)
        tk.Entry(textvariable=self.fwd_min, bg=color[3]).grid(
            row=3, column=2, sticky="nsew"
        )
        self.rev_min = tk.IntVar(value=0)
        label("Rev Min Max", color[3], 3, 3)
        tk.Entry(textvariable=self.rev_min, bg=color[3]).grid(
            row=3, column=4, sticky="nsew"
        )
        self.rev_max = tk.IntVar(value=-(2**15))
        tk.Entry(textvariable=self.rev_max, bg=color[3]).grid(
            row=3, column=5, sticky="nsew"
        )
        tk.Button(
            text="Set Duty Cycle Range", command=self.send_duty_cycle_range, bg=color[3]
        ).grid(row=3, column=6, sticky="nsew")
        label("A", color[4], 4, 0)
        self.soft_limit_a = tk.IntVar(value=-(2**31))
        tk.Entry(textvariable=self.soft_limit_a, bg=color[4]).grid(
            row=4, column=1, sticky="nsew"
        )
        label("B", color[4], 4, 2)
        self.soft_limit_b = tk.IntVar(value=2**31 - 1)
        tk.Entry(textvariable=self.soft_limit_b, bg=color[4]).grid(
            row=4, column=3, sticky="nsew"
        )
        tk.Button(
            text="Set Soft Limit Position", command=self.send_soft_limits, bg=color[4]
        ).grid(row=4, column=4, columnspan=2, sticky="nsew")
        label("Duty Cycle", color[5], 5, 0)
        self.calibration_duty_cycle = tk.IntVar(value=16384)
        tk.Entry(textvariable=self.calibration_duty_cycle, bg=color[5]).grid(
            row=5, column=1, sticky="nsew"
        )
        label("Position", color[5], 5, 2)
        self.limit_switch_position = tk.IntVar(value=0)
        tk.Entry(textvariable=self.limit_switch_position, bg=color[5]).grid(
            row=5, column=3, sticky="nsew"
        )
        tk.Button(
            text="Start Position Calibration", command=self.send_calibrate, bg=color[5]
        ).grid(row=5, column=4, columnspan=2, sticky="nsew")
        label("Ignore Limit", color[6], 8, 0)
        self.ignore_limit_a = tk.IntVar()
        tk.Checkbutton(
            text="A",
            variable=self.ignore_limit_a,
            onvalue=1,
            offvalue=0,
            command=self.send_ignore_limit,
            bg=color[6],
        ).grid(row=8, column=1, sticky="nsew")
        self.ignore_limit_b = tk.IntVar()
        tk.Checkbutton(
            text="B",
            variable=self.ignore_limit_b,
            onvalue=1,
            offvalue=0,
            command=self.send_ignore_limit,
            bg=color[6],
        ).grid(row=8, column=2, sticky="nsew")
        label("Duty Cycle", color[7], 8, 4)
        self.duty_cycle = tk.IntVar(value=16384)
        tk.Entry(textvariable=self.duty_cycle, bg=color[7]).grid(
            row=8, column=5, sticky="nsew"
        )
        tk.Button(text="Open Loop", command=self.send_open_loop, bg=color[7]).grid(
            row=8, column=6, sticky="nsew"
        )
        label("Feed Forward", color[8], 9, 0)
        self.feed_forward = tk.IntVar(value=0)
        tk.Entry(textvariable=self.feed_forward, bg=color[8]).grid(
            row=9, column=1, sticky="nsew"
        )
        label("Target", color[8], 9, 2)
        self.target = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.target, bg=color[8]).grid(
            row=9, column=3, sticky="nsew"
        )
        tk.Button(
            text="Target Position", command=self.send_target_position, bg=color[8]
        ).grid(row=9, column=4, sticky="nsew")
        tk.Button(
            text="Target Velocity", command=self.send_target_velocity, bg=color[8]
        ).grid(row=9, column=5, sticky="nsew")
        tk.Button(
            text="Target Current", command=self.send_target_current, bg=color[8]
        ).grid(row=9, column=6, sticky="nsew")

        self.data = tk.StringVar(
            value="Position (step): ??????, Angular Velocity (step/s): ??????, Current (A): ??.??\nLimit A: ?, Limit B: ?, Soft Limit A: ?, Soft Limit B: ?"
        )
        tk.Label(textvariable=self.data, bg=color[0], anchor="w").grid(
            row=10, column=0, sticky="nsew", columnspan=5
        )

        self.ping_reply = tk.StringVar(value="Ping")
        tk.Button(
            textvariable=self.ping_reply, command=self.send_ping, bg=color[0]
        ).grid(row=10, column=5, columnspan=2, sticky="nsew")

        self.debugText = tk.StringVar()
        tk.Label(textvariable=self.debugText, bg=color[9], anchor="w").grid(
            row=11, column=0, sticky="nsew", columnspan=5
        )

        self.figure = Figure(figsize=(5, 4), dpi=100)
        self.ax1 = self.figure.add_subplot()
        self.ax2 = self.ax1.twinx()
        self.time = np.zeros(100)
        self.debugData = np.zeros((len(DEBUG_DATA_LABELS), self.time.shape[0]))
        self.lines = [
            (self.ax1 if i in AX1_SERIES else self.ax2).plot(
                self.time,
                self.debugData[i],
                DEBUG_DATA_FORMATS[i],
                label=DEBUG_DATA_LABELS[i],
            )[0]
            for i in range(len(DEBUG_DATA_LABELS))
        ]
        self.lines_by_label = {line.get_label(): line for line in self.lines}

        line_colors = [line.get_color() for line in self.lines]
        self.check_buttons = CheckButtons(
            ax=self.ax1.inset_axes([0.0, 0.0, 0.12, 0.4]),
            labels=self.lines_by_label.keys(),
            actives=[line.get_visible() for line in self.lines],
            label_props={"color": line_colors},
            frame_props={"edgecolor": line_colors},
            check_props={"facecolor": line_colors},
        )
        self.check_buttons.on_clicked(self.toggle_graph_line)

        self.ax1.set_xlabel("Time")
        self.ax1.set_ylabel(", ".join([DEBUG_DATA_LABELS[i] for i in AX1_SERIES]))
        self.ax2.set_ylabel(
            ", ".join(
                [
                    DEBUG_DATA_LABELS[i]
                    for i in range(len(DEBUG_DATA_LABELS))
                    if i not in AX1_SERIES
                ]
            )
        )
        self.ax1.legend(loc="upper left")
        self.ax2.legend(loc="upper right")
        self.canvas = FigureCanvasTkAgg(self.figure, master=master)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=12, column=0, sticky="nsew", columnspan=7)
        self.redraw_graph_after = time.time() + 1
        self.graph_paused = False
        tk.Button(
            text="Enable Debug Telemetry",
            command=lambda: self.send_debug_telemetry(True),
            bg=color[9],
        ).grid(row=4, column=6, sticky="nsew")
        tk.Button(
            text="Disable Debug Telemetry",
            command=lambda: self.send_debug_telemetry(False),
            bg=color[9],
        ).grid(row=5, column=6, sticky="nsew")
        tk.Button(
            text="Pause Graph",
            command=lambda: self.__setattr__("graph_paused", True),
            bg=color[9],
        ).grid(row=11, column=5, sticky="nsew")
        tk.Button(
            text="Unpause Graph",
            command=lambda: self.__setattr__("graph_paused", False),
            bg=color[9],
        ).grid(row=11, column=6, sticky="nsew")

        self.update_telemetry()

        self.last_ports = set()
        self.update_serial_dropdown()


    def update_serial_dropdown(self):
        global can_active
        if not can_active:
            self.serial_port.set("Select Serial Port")
        current_ports = set(serial.tools.list_ports.comports())
        if current_ports != self.last_ports:
            print("Change in USB devices detected")
            self.serial_dropdown["menu"].delete(0, "end")
            for port in serial.tools.list_ports.comports():
                self.serial_dropdown["menu"].add_command(label=port, command=tk._setit(self.serial_port, port, self.open_serial))
        self.last_ports = current_ports
        self.after(1000, self.update_serial_dropdown)

    def open_serial(self, selection):
        global can_active
        self.close_serial()
        ports = serial.tools.list_ports.comports()
        if selection not in ports:
            return
        port = ports[ports.index(selection)].device
        self.serial_port.set(port)
        print(f"Opening port {self.serial_port.get()}")
        can_active = True
        # clear queues
        while not self.can_send.empty():
            self.can_send.get()
        while not self.can_recv.empty():
            self.can_recv.get()
        self.can_send.empty()
        self.can_process = multiprocessing.Process(target=can_main, args=(port, self.can_send, self.can_recv))
        self.can_process.start()
    
    def close_serial(self):
        global can_active
        if hasattr(self, "can_process"):
            print(f"Closing port {self.serial_port.get()}")
            self.can_process.kill()
            self.can_process.join()
            can_active = False

    def get_id(self):
        return int(self.id.get(), 16)

    def get_shifted_id(self):
        return int(self.id.get(), 16) << SMOCO_WIDTH_MID

    def update_telemetry(self):
        if time.time() > self.redraw_graph_after and not self.graph_paused:
            self.redraw_graph_after = time.time() + 1
            self.debugText.set(
                f"Time: {self.time[-1]:.4f}, Position: {self.debugData[0, -1]:.0f}, Velocity: {self.debugData[1, -1]:.2f}, Current: {self.debugData[2, -1]:.0f}\nP Out: {self.debugData[3, -1]:.4f}, I Out: {self.debugData[4, -1]:.4f}, D Out: {self.debugData[5, -1]:.4}\nError: {self.debugData[6, -1]:.4f}, DeltaT: {self.debugData[7, -1]:.5f}, Target: {self.debugData[8, -1]:.2f}"
            )
            self.debugData = np.nan_to_num(self.debugData, 0)
            for i in range(self.debugData.shape[0]):
                self.lines[i].set_data(self.time, self.debugData[i])
            self.ax1.relim()
            self.ax2.relim()
            self.ax1.autoscale(True)
            self.ax2.autoscale(True)
            self.canvas.draw()
        try:
            while True:
                message: can.Message = self.can_recv.get(False)
                if message.arbitration_id & SMOCO_ID_DEBUG == SMOCO_ID_DEBUG:
                    self.process_debug_message(message)
                elif message.arbitration_id >> SMOCO_WIDTH_MID == self.get_id():
                    self.process_rx_message(message)
                else:
                    print(
                        f"RX ID: 0x{message.arbitration_id:03X}, Data: 0x{' '.join((f'{byte:02X}' for byte in message.data))}"
                    )
        except:
            pass
        self.after(50, self.update_telemetry)

    def process_debug_message(self, message: can.Message):
        # Update debug data.
        if message.arbitration_id == SMOCO_ID_DEBUG:
            if self.time[-1] == 0:
                # Upon receiving the first time, set the x-axis to the previous 10 seconds.
                current_time = struct.unpack("<Q", message.data)[0] / 2000000
                self.time = np.arange(current_time - 10, current_time, 0.1)
            else:
                self.time = np.roll(self.time, -1, 0)
                self.time[-1] = struct.unpack("<Q", message.data)[0] / 2000000
            self.debugData = np.roll(self.debugData, -1, 1)
            # Use last target as current target.
            self.debugData[8, -1] = self.debugData[8, -2]
        elif message.arbitration_id & 0xF == 0x1:
            self.debugData[(message.arbitration_id & 0xF) - 1, -1] = struct.unpack(
                "<q", message.data
            )[0]
        else:
            self.debugData[(message.arbitration_id & 0xF) - 1, -1] = struct.unpack(
                "<f", message.data
            )[0]

    def set_graph_paused(self, paused: bool):
        self.graph_paused = paused

    def toggle_graph_line(self, label):
        line = self.lines_by_label[label]
        line.set_visible(not line.get_visible())
        line.figure.canvas.draw_idle()

    def process_rx_message(self, message: can.Message):
        mid = message.arbitration_id & ((1 << SMOCO_WIDTH_MID) - 1)
        if message.is_remote_frame:
            print(f"RX ID: 0x{message.arbitration_id:03X}, Remote")
            if mid == SMOCO_MID_RAMP_RATE:
                self.send_ramp_rate()
            elif mid == SMOCO_MID_PI:
                self.send_pi()
            elif mid == SMOCO_MID_D:
                self.send_d()
            elif mid == SMOCO_MID_IGNORE_LIMIT:
                self.send_ignore_limit()
            elif mid == SMOCO_MID_SOFT_LIMIT:
                self.send_soft_limits()
            elif mid == SMOCO_MID_DUTY_CYCLE_RANGE:
                self.send_duty_cycle_range()
        else:
            if mid == SMOCO_MID_POSITION:
                position, velocity, current, flags = struct.unpack(
                    "<ihcc", message.data
                )
                current = current[0] / 8
                limit_a = flags[0] & 0b1 != 0
                limit_b = flags[0] & 0b10 != 0
                soft_limit_a_reached = flags[0] & 0b100 != 0
                soft_limit_b_reached = flags[0] & 0b1000 != 0
                self.data.set(
                    f"Position (step): {position:06}, Angular Velocity (step/s): {velocity:06}, Current (A): {current:05.2f}\nLimit A: {limit_a:1}, Limit B: {limit_b:1}, Soft Limit A: {soft_limit_a_reached:1}, Soft Limit B: {soft_limit_b_reached:1}",
                )
            elif mid == SMOCO_MID_POSITION_CALIBRATED:
                print(f"RX ID: 0x{message.arbitration_id:03X}, Position Calibrated")
            elif mid == SMOCO_MID_ERROR:
                command_id = struct.unpack("<c", message.data)[0]
                print(
                    f"RX ID: 0x{message.arbitration_id:03X}, Command Error, Command ID: {command_id}"
                )
            elif mid == SMOCO_MID_ECHO_REPLY:
                self.ping_reply.set(
                    f"Ping Reply in {int(time.time() * 1000) - struct.unpack('<Q', message.data)[0]}ms"
                )
                print(
                    f"RX ID: 0x{message.arbitration_id:03X}, Echo Reply, Payload: 0x{' '.join((f'{byte:02X}' for byte in message.data))}"
                )
            else:
                print(
                    f"RX ID: 0x{message.arbitration_id:03X}, Undefined, Data: 0x{' '.join((f'{byte:02X}' for byte in message.data))}"
                )

    def send_data(self, mid, *data):
        global can_active
        if not can_active:
            return
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | mid,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=SMOCO_WIDTH[mid],
                data=struct.pack(SMOCO_FORMAT[mid], *data),
            ),
            False,
        )

    def send_ramp_rate(self):
        self.send_data(SMOCO_MID_RAMP_RATE, self.ramp_rate.get())

    def send_pid(self):
        self.send_pi()
        self.send_d()

    def send_pi(self):
        self.send_data(SMOCO_MID_PI, self.p.get(), self.i.get())

    def send_d(self):
        self.send_data(SMOCO_MID_D, self.d.get())

    def send_ignore_limit(self):
        self.send_data(
            SMOCO_MID_IGNORE_LIMIT,
            self.ignore_limit_a.get() | (self.ignore_limit_b.get() << 1),
        )

    def send_soft_limits(self):
        self.send_data(
            SMOCO_MID_SOFT_LIMIT, self.soft_limit_a.get(), self.soft_limit_b.get()
        )

    def send_calibrate(self):
        self.send_data(
            SMOCO_MID_CALIBRATE,
            self.calibration_duty_cycle.get(),
            self.limit_switch_position.get(),
        )

    def send_duty_cycle_range(self):
        self.send_data(
            SMOCO_MID_DUTY_CYCLE_RANGE,
            self.fwd_max.get(),
            self.fwd_min.get(),
            self.rev_min.get(),
            self.rev_max.get(),
        )

    def send_ping(self):
        self.ping_reply.set("Pinging")
        self.send_data(SMOCO_MID_ECHO_REQUEST, int(time.time() * 1000))

    def send_stop(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | SMOCO_MID_STOP,
                is_extended_id=False,
                dlc=0,
            ),
            False,
        )

    def send_open_loop(self):
        self.send_data(SMOCO_MID_OPEN_LOOP, self.duty_cycle.get())

    def send_target_position(self):
        target = int(self.target.get())
        ff = self.feed_forward.get()
        self.debugData[8, -1] = target
        self.send_data(SMOCO_MID_TARGET_POSITION, ff, target)

    def send_target_velocity(self):
        target = self.target.get()
        ff = self.feed_forward.get()
        self.debugData[8, -1] = target
        self.send_data(SMOCO_MID_TARGET_VELOCITY, ff, target)

    def send_target_current(self):
        target = int(self.target.get())
        ff = self.feed_forward.get()
        self.debugData[8, -1] = target
        self.send_data(SMOCO_MID_TARGET_CURRENT, ff, target)

    def send_debug_telemetry(self, enabled: bool):
        self.send_data(SMOCO_MID_DEBUG, 1 if enabled else 0)


def app_main(can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue):
    root = tk.Tk()
    root.wm_title("Smart Motor Controller CAN UI")
    app = App(root, can_send, can_recv)
    app.mainloop()
    app.close_serial()


def can_main(comport: str, can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue):
    global can_active
    can_active = True
    # bus = can.Bus(channel=0, interface="gs_usb", bitrate=125 * 1000)
    try:
        bus = can.interface.Bus(channel=comport, interface="serial", baudrate=115200, timeout=0.1, rtscts=False)
        print("Success")
    except Exception as e:
        print(f"Port {comport} not found:", e)
        can_active = False
        return

    while True:
        try:
            message = bus.recv(1)
            if message is not None:
                can_recv.put(message, False)
                print(message)
        except Exception as e:
            print(f"Disconnected from port {comport}:", e, message)
            can_active = False
            return

        try:
            message = can_send.get(False)
            print(
                f"TX ID: 0x{message.arbitration_id:03X}, Data: 0x{' '.join((f'{byte:02X}' for byte in message.data))}, Success: ",
                end="",
            )
            try:
                bus.send(message, 0.5)
                print("true")
            except Exception as e:
                print(f"Disconnected from port {comport}:", e)
                can_active = False
                return
        except:
            pass


if __name__ == "__main__":
    multiprocessing.freeze_support()
    can_send = multiprocessing.Queue()
    can_recv = multiprocessing.Queue()
    app_process = multiprocessing.Process(target=app_main, args=(can_send, can_recv))
    app_process.start()
    app_process.join()
