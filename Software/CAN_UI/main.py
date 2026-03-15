import can, struct, time
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.widgets import CheckButtons
import tkinter.font as tkFont
import multiprocessing

import numpy as np

MESSAGE_ID_POSITION = 0x30
MESSAGE_ID_POSITION_CALIBRATED = 0x31
MESSAGE_ID_ERROR = 0x32
MESSAGE_ID_ECHO_REPLY = 0x3F
MESSAGE_ID_STOP = 0x00
MESSAGE_ID_RAMP_RATE = 0x01
MESSAGE_ID_PI = 0x02
MESSAGE_ID_D = 0x03
MESSAGE_ID_IGNORE_LIMIT = 0x04
MESSAGE_ID_SOFT_LIMIT = 0x05
MESSAGE_ID_CALIBRATE = 0x06
MESSAGE_ID_DEBUG = 0x07
MESSAGE_ID_DUTY_CYCLE_RANGE = 0x08
MESSAGE_ID_ECHO_REQUEST = 0x0F
MESSAGE_ID_OPEN_LOOP = 0x10
MESSAGE_ID_TARGET_POSITION = 0x11
MESSAGE_ID_TARGET_VELOCITY = 0x12
MESSAGE_ID_TARGET_CURRENT = 0x13
MESSAGE_ID_DEBUG_OFFSET = 0x7F0

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


class App(tk.Frame):
    def __init__(
        self, master, can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue
    ):
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
        ).grid(row=0, column=2, rowspan=2, columnspan=2)

        tk.Label(text="ID 0x").grid(row=0, column=4, sticky="e")
        self.id = tk.StringVar(value="0B")
        tk.Entry(textvariable=self.id).grid(row=0, column=5, sticky="nsew")
        tk.Button(text="Stop and Reset", command=self.send_stop).grid(
            row=0, column=6, sticky="nsew"
        )
        tk.Label(text="Ramp Rate (1/s)").grid(row=1, column=4, sticky="e")
        self.ramp_rate = tk.DoubleVar(value=1.0)
        tk.Entry(textvariable=self.ramp_rate).grid(row=1, column=5, sticky="nsew")
        tk.Button(text="Set Ramp Rate", command=self.send_ramp_rate).grid(
            row=1, column=6, sticky="nsew"
        )
        tk.Label(text="P").grid(row=2, column=0, sticky="e")
        self.p = tk.DoubleVar(value=0.7)
        tk.Entry(textvariable=self.p).grid(row=2, column=1, sticky="nsew")
        tk.Label(text="I").grid(row=2, column=2, sticky="e")
        self.i = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.i).grid(row=2, column=3, sticky="nsew")
        tk.Label(text="D").grid(row=2, column=4, sticky="e")
        self.d = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.d).grid(row=2, column=5, sticky="nsew")
        tk.Button(text="Set PID", command=self.send_pid).grid(
            row=2, column=6, sticky="nsew"
        )
        tk.Label(text="Fwd Max Min").grid(row=3, column=0, sticky="e")
        self.fwd_max = tk.DoubleVar(value=2**31 - 1)
        tk.Entry(textvariable=self.fwd_max).grid(row=3, column=1, sticky="nsew")
        self.fwd_min = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.fwd_min).grid(row=3, column=2, sticky="nsew")
        self.rev_min = tk.DoubleVar(value=0)
        tk.Label(text="Rev Min Max").grid(row=3, column=3, sticky="e")
        tk.Entry(textvariable=self.rev_min).grid(row=3, column=4, sticky="nsew")
        self.rev_max = tk.DoubleVar(value=-(2**31))
        tk.Entry(textvariable=self.rev_max).grid(row=3, column=5, sticky="nsew")
        tk.Button(text="Set Duty Cycle Range", command=self.send_duty_cycle_range).grid(
            row=3, column=6, sticky="nsew"
        )
        tk.Label(text="A").grid(row=4, column=0, sticky="e")
        self.soft_limit_a = tk.IntVar(value=-(2**31))
        tk.Entry(textvariable=self.soft_limit_a).grid(row=4, column=1, sticky="nsew")
        tk.Label(text="B").grid(row=4, column=2, sticky="e")
        self.soft_limit_b = tk.IntVar(value=2**31 - 1)
        tk.Entry(textvariable=self.soft_limit_b).grid(row=4, column=3, sticky="nsew")
        tk.Button(text="Set Soft Limit Position", command=self.send_soft_limits).grid(
            row=4, column=5, sticky="nsew"
        )
        tk.Label(text="Duty Cycle").grid(row=5, column=0, sticky="e")
        self.calibration_duty_cycle = tk.DoubleVar(value=0.5)
        tk.Entry(textvariable=self.calibration_duty_cycle).grid(
            row=5, column=1, sticky="nsew"
        )
        tk.Label(text="Position").grid(row=5, column=2, sticky="e")
        self.limit_switch_position = tk.IntVar(value=0)
        tk.Entry(textvariable=self.limit_switch_position).grid(
            row=5, column=3, sticky="nsew"
        )
        tk.Button(text="Start Position Calibration", command=self.send_calibrate).grid(
            row=5, column=5, sticky="nsew"
        )
        tk.Label(text="Ignore Limit").grid(row=8, column=0, sticky="e")
        self.ignore_limit_a = tk.IntVar()
        tk.Checkbutton(text="A", variable=self.ignore_limit_a, onvalue=1, offvalue=0).grid(
            row=8, column=1
        )
        self.ignore_limit_b = tk.IntVar()
        tk.Checkbutton(text="B", variable=self.ignore_limit_b, onvalue=1, offvalue=0).grid(
            row=8, column=2
        )
        tk.Label(text="Duty Cycle").grid(row=8, column=3, sticky="e")
        self.duty_cycle = tk.DoubleVar(value=0.5)
        tk.Entry(textvariable=self.duty_cycle).grid(row=8, column=4, sticky="nsew")
        tk.Button(text="Open Loop", command=self.send_open_loop).grid(
            row=8, column=5, sticky="nsew"
        )
        tk.Label(text="Feed Forward").grid(row=9, column=0, sticky="e")
        self.feed_forward = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.feed_forward).grid(row=9, column=1, sticky="nsew")
        tk.Label(text="Target").grid(row=9, column=2, sticky="e")
        self.target = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.target).grid(row=9, column=3, sticky="nsew")
        tk.Button(text="Target Position", command=self.send_target_position).grid(
            row=8, column=6, sticky="nsew"
        )
        tk.Button(text="Target Velocity", command=self.send_target_velocity).grid(
            row=9, column=5, sticky="nsew"
        )
        tk.Button(text="Target Current", command=self.send_target_current).grid(
            row=9, column=6, sticky="nsew"
        )
        tk.Button(
            text="Enable Debug Telemetry",
            command=lambda: self.send_debug_telemetry(True),
        ).grid(row=4, column=6, sticky="nsew")
        tk.Button(
            text="Disable Debug Telemetry",
            command=lambda: self.send_debug_telemetry(False),
        ).grid(row=5, column=6, sticky="nsew")

        self.data = tk.StringVar(
            value="Position (step): , Angular Velocity (step/s): , Current (A): \nLimit A: , Limit B: , Soft Limit A: , Soft Limit B: "
        )
        tk.Label(textvariable=self.data).grid(row=10, column=0, sticky="w", columnspan=5)

        self.ping_reply = tk.StringVar(value="Ping")
        tk.Button(textvariable=self.ping_reply, command=self.send_ping).grid(
            row=10, column=5, columnspan=2, sticky="nsew"
        )

        self.debugText = tk.StringVar()
        tk.Label(textvariable=self.debugText).grid(
            row=11, column=0, sticky="w", columnspan=4
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
            text="Pause Graph", command=lambda: self.__setattr__("graph_paused", True)
        ).grid(row=11, column=5, sticky="nsew")
        tk.Button(
            text="Unpause Graph",
            command=lambda: self.__setattr__("graph_paused", False),
        ).grid(row=11, column=6, sticky="nsew")

        self.update_telemetry()

    def get_shifted_id(self):
        return int(self.id.get(), 16) << 6

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
                if message.arbitration_id & MESSAGE_ID_DEBUG_OFFSET == MESSAGE_ID_DEBUG_OFFSET:
                    self.process_debug_message(message)
                elif message.arbitration_id & 0x7C == self.get_shifted_id():
                    self.process_rx_message(message)
                else:
                    print(
                        f"RX ID: 0x{message.arbitration_id:03X}, Data: 0x{" ".join((f"{byte:02X}" for byte in message.data))}"
                    )
        except:
            pass
        self.after(50, self.update_telemetry)

    def process_debug_message(self, message: can.Message):
        # Update debug data.
        if message.arbitration_id & 0xF == 0x0:
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
        if message.is_remote_frame:
            print(f"RX ID: 0x{message.arbitration_id:03X}, Remote")
            if message.arbitration_id & 0x3F == MESSAGE_ID_RAMP_RATE:
                self.send_ramp_rate()
            if message.arbitration_id & 0x3F ==  MESSAGE_ID_PI:
                self.send_pi()
            if message.arbitration_id & 0x3F ==  MESSAGE_ID_D:
                self.send_d()
            if message.arbitration_id & 0x3F ==  MESSAGE_ID_IGNORE_LIMIT:
                self.send_ignore_limit()
            if message.arbitration_id & 0x3F ==  MESSAGE_ID_SOFT_LIMIT:
                self.send_soft_limits()
            if message.arbitration_id & 0x3F ==  MESSAGE_ID_DUTY_CYCLE_RANGE:
                self.send_duty_cycle_range()
        else:
            print(
                f"RX ID: 0x{message.arbitration_id:03X}, Undefined, Data: 0x{" ".join((f"{byte:02X}" for byte in message.data))}"
            )

    def send_ramp_rate(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_RAMP_RATE,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=4,
                data=struct.pack("<f", self.ramp_rate.get()),
            ),
            False,
        )

    def send_pid(self):
        self.send_pi()
        self.send_d()

    def send_pi(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_PI,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=8,
                data=struct.pack(
                    "<ff",
                    self.p.get(),
                    self.i.get(),
                ),
            ),
            False,
        )

    def send_d(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_D,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=4,
                data=struct.pack(
                    "<f",
                    self.d.get(),
                ),
            ),
            False,
        )

    def send_ignore_limit(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_IGNORE_LIMIT,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=1,
                data=struct.pack(
                    "<B",
                    (self.ignore_limit_a.get() << 7) | (self.ignore_limit_b.get() << 6),
                ),
            ),
            False,
        )

    def send_soft_limits(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_SOFT_LIMIT,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=8,
                data=struct.pack(
                    "<ii",
                    self.soft_limit_a.get(),
                    self.soft_limit_b.get(),
                ),
            ),
            False,
        )

    def send_calibrate(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_CALIBRATE,
                is_extended_id=False,
                data=struct.pack(
                    "<hi",
                    int(self.calibration_duty_cycle.get() * 32768),
                    self.limit_switch_position.get(),
                ),
                dlc=6,
            ),
            False,
        )

    def send_duty_cycle_range(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_DUTY_CYCLE_RANGE,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=8,
                data=struct.pack(
                    "<hhhh",
                    self.fwd_max.get(),
                    self.fwd_min.get(),
                    self.rev_min.get(),
                    self.rev_max.get(),
                ),
            ),
            False,
        )

    def send_ping(self):
        self.ping_reply.set("Pinging")
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_ECHO_REQUEST,
                is_extended_id=False,
                dlc=8,
                data=struct.pack("<Q", int(time.time() * 1000)),
            )
        )

    def send_stop(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_STOP,
                is_extended_id=False,
                dlc=0,
            ),
            False,
        )

    def send_open_loop(self):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_OPEN_LOOP,
                is_extended_id=False,
                data=struct.pack(
                    "<h",
                    int(self.duty_cycle.get() * 32768),
                ),
                dlc=2,
            ),
            False,
        )

    def send_target_position(self):
        target = int(self.target.get())
        self.debugData[8, -1] = target
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_TARGET_POSITION,
                is_extended_id=False,
                data=struct.pack(
                    "<hl",
                    int(self.feed_forward.get() * 32768),
                    target,
                ),
                dlc=6,
            ),
            False,
        )

    def send_target_velocity(self):
        target = int(self.target.get())
        self.debugData[8, -1] = target
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_TARGET_VELOCITY,
                is_extended_id=False,
                data=struct.pack(
                    "<hf",
                    int(self.feed_forward.get() * 32768),
                    target,
                ),
                dlc=6,
            ),
            False,
        )

    def send_target_current(self):
        target = int(self.target.get())
        self.debugData[8, -1] = target
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_TARGET_CURRENT,
                is_extended_id=False,
                data=struct.pack(
                    "<hh",
                    int(self.feed_forward.get()),
                    target,
                ),
                dlc=4,
            ),
            False,
        )

    def send_debug_telemetry(self, enabled: bool):
        self.can_send.put(
            can.Message(
                arbitration_id=self.get_shifted_id() | MESSAGE_ID_DEBUG,
                is_extended_id=False,
                data=struct.pack("<B", 1 if enabled else 0),
                dlc=1,
            ),
            False,
        )


def app_main(can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue):
    root = tk.Tk()
    root.wm_title("Smart Motor Controller CAN UI")
    app = App(root, can_send, can_recv)
    app.mainloop()


def can_main(can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue):
    with can.Bus(channel=0, interface="gs_usb", bitrate=125 * 1000) as bus:
        last_message = can.Message()
        while True:
            message = bus.recv(1)
            if message != None and (
                message.arbitration_id != last_message.arbitration_id
                or message.dlc != last_message.dlc
                or message.data != last_message.data
            ):
                last_message = message
                can_recv.put(message, False)

            try:
                message = can_send.get(False)
                print(
                    f"TX ID: 0x{message.arbitration_id:03X}, Data: 0x{" ".join((f"{byte:02X}" for byte in message.data))}, Success: ",
                    end="",
                )
                try:
                    bus.send(message, 0.5)
                    print("true")
                except:
                    print("false")
            except:
                pass


if __name__ == "__main__":
    multiprocessing.freeze_support()
    can_send = multiprocessing.Queue()
    can_recv = multiprocessing.Queue()
    can_process = multiprocessing.Process(target=can_main, args=(can_send, can_recv))
    print("Starting can_process.")
    can_process.start()
    app_process = multiprocessing.Process(target=app_main, args=(can_send, can_recv))
    print("Starting app_process.")
    app_process.start()
    print("Joining app_process.")
    app_process.join()
    print("Killing can_process.")
    can_process.kill()
    print("Joining can_process.")
    can_process.join()
