import can, struct, time
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.widgets import CheckButtons
import tkinter.font as tkFont
import multiprocessing

import numpy as np

DEBUG_DATA_LABELS = [
    "Angle",
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
        tk.Label(text="Alpha").grid(row=1, column=4, sticky="e")
        self.alpha = tk.DoubleVar(value=0.3)
        tk.Entry(textvariable=self.alpha).grid(row=1, column=5, sticky="nsew")
        tk.Button(text="Set Low-Pass Smoothing Factor", command=self.send_alpha).grid(
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
        tk.Label(text="A").grid(row=3, column=0, sticky="e")
        self.soft_limit_a = tk.IntVar(value=-(2**31))
        tk.Entry(textvariable=self.soft_limit_a).grid(row=3, column=1, sticky="nsew")
        tk.Label(text="B").grid(row=3, column=2, sticky="e")
        self.soft_limit_b = tk.IntVar(value=2**31 - 1)
        tk.Entry(textvariable=self.soft_limit_b).grid(row=3, column=3, sticky="nsew")
        tk.Button(text="Set Soft Limit Position", command=self.send_soft_limits).grid(
            row=3, column=5, sticky="nsew"
        )
        tk.Label(text="Duty Cycle").grid(row=4, column=0, sticky="e")
        self.calibration_duty_cycle = tk.DoubleVar(value=0.5)
        tk.Entry(textvariable=self.calibration_duty_cycle).grid(
            row=4, column=1, sticky="nsew"
        )
        tk.Label(text="Position").grid(row=4, column=2, sticky="e")
        self.limit_switch_position = tk.IntVar(value=0)
        tk.Entry(textvariable=self.limit_switch_position).grid(
            row=4, column=3, sticky="nsew"
        )
        tk.Button(text="Start Position Calibration", command=self.send_calibrate).grid(
            row=4, column=5, sticky="nsew"
        )
        tk.Label(text="Ignore Limit").grid(row=7, column=0, sticky="e")
        self.ignore_limit = tk.IntVar()
        tk.Checkbutton(variable=self.ignore_limit, onvalue=1, offvalue=0).grid(
            row=7, column=1
        )
        tk.Label(text="Duty Cycle").grid(row=7, column=2, sticky="e")
        self.duty_cycle = tk.DoubleVar(value=0.5)
        tk.Entry(textvariable=self.duty_cycle).grid(row=7, column=3, sticky="nsew")
        tk.Button(text="Open Loop", command=self.send_open_loop).grid(
            row=7, column=5, sticky="nsew"
        )
        tk.Label(text="Error Gain").grid(row=8, column=0, sticky="e")
        self.error_gain = tk.DoubleVar(value=0.05)
        tk.Entry(textvariable=self.error_gain).grid(row=8, column=1, sticky="nsew")
        tk.Label(text="Target").grid(row=8, column=2, sticky="e")
        self.target = tk.DoubleVar(value=0)
        tk.Entry(textvariable=self.target).grid(row=8, column=3, sticky="nsew")
        tk.Button(text="Target Position", command=self.send_target_position).grid(
            row=7, column=6, sticky="nsew"
        )
        tk.Button(text="Target Velocity", command=self.send_target_velocity).grid(
            row=8, column=5, sticky="nsew"
        )
        tk.Button(text="Target Current", command=self.send_target_current).grid(
            row=8, column=6, sticky="nsew"
        )
        tk.Button(
            text="Enable Debug Telemetry",
            command=lambda: self.send_debug_telemetry(True),
        ).grid(row=3, column=6, sticky="nsew")
        tk.Button(
            text="Disable Debug Telemetry",
            command=lambda: self.send_debug_telemetry(False),
        ).grid(row=4, column=6, sticky="nsew")

        self.data = tk.StringVar(
            value="Angle (step): , Angular Velocity (step/s): , Current (A): \nLimit A: , Limit B: , Soft Limit A: , Soft Limit B: "
        )
        tk.Label(textvariable=self.data).grid(row=9, column=0, sticky="w", columnspan=5)

        self.ping_reply = tk.StringVar(value="Ping")
        tk.Button(textvariable=self.ping_reply, command=self.send_ping).grid(
            row=9, column=5, columnspan=2, sticky="nsew"
        )

        self.debugText = tk.StringVar()
        tk.Label(textvariable=self.debugText).grid(
            row=10, column=0, sticky="w", columnspan=4
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
        self.canvas.get_tk_widget().grid(row=11, column=0, sticky="nsew", columnspan=7)
        self.redraw_graph_after = time.time() + 1
        self.graph_paused = False
        tk.Button(
            text="Pause Graph", command=lambda: self.__setattr__("graph_paused", True)
        ).grid(row=10, column=5, sticky="nsew")
        tk.Button(
            text="Unpause Graph",
            command=lambda: self.__setattr__("graph_paused", False),
        ).grid(row=10, column=6, sticky="nsew")

        self.update_telemetry()

    def update_telemetry(self):
        if time.time() > self.redraw_graph_after and not self.graph_paused:
            self.redraw_graph_after = time.time() + 1
            self.debugText.set(
                f"Time: {self.time[-1]:.4f}, Angle: {self.debugData[0, -1]:.0f}, Velocity: {self.debugData[1, -1]:.2f}, Current: {self.debugData[2, -1]:.0f}\nP Out: {self.debugData[3, -1]:.4f}, I Out: {self.debugData[4, -1]:.4f}, D Out: {self.debugData[5, -1]:.4}\nError: {self.debugData[6, -1]:.4f}, DeltaT: {self.debugData[7, -1]:.5f}, Target: {self.debugData[8, -1]:.2f}"
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
                if message.arbitration_id & 0x7F0 == 0x7F0:
                    self.process_debug_message(message)
                elif message.arbitration_id & 0x7F0 == int(self.id.get() + "0", 16):
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
                "<d", message.data
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
            match message.arbitration_id & 0x00F:
                case 0x3:
                    self.send_alpha()
                case 0x4:
                    self.send_pid()
                case 0x5:
                    self.send_soft_limits()
        else:
            match message.arbitration_id & 0xF:
                case 0x0:
                    angle, velocity, current, flags = struct.unpack(
                        "<ihcc", message.data
                    )
                    current = current[0] / 8
                    limit_a = flags[0] & 0b10000000 != 0
                    limit_b = flags[0] & 0b01000000 != 0
                    soft_limit_a_reached = flags[0] & 0b00100000 != 0
                    soft_limit_b_reached = flags[0] & 0b00010000 != 0
                    self.data.set(
                        f"Angle (step): {angle:06}, Angular Velocity (step/s): {velocity:06}, Current (A): {current:05.2f}\nLimit A: {limit_a:1}, Limit B: {limit_b:1}, Soft Limit A: {soft_limit_a_reached:1}, Soft Limit B: {soft_limit_b_reached:1}",
                    )
                case 0x1:
                    print(f"RX ID: 0x{message.arbitration_id:03X}, Position Calibrated")
                case 0xD:
                    command_id = struct.unpack("<c", message.data)[0]
                    print(
                        f"RX ID: 0x{message.arbitration_id:03X}, Command Error, Command ID: {command_id}"
                    )
                case 0xF:
                    self.ping_reply.set(
                        f"Ping Reply in {int(time.time()*1000) - struct.unpack("<Q", message.data)[0]}ms"
                    )
                    print(
                        f"RX ID: 0x{message.arbitration_id:03X}, Echo Reply, Payload: 0x{" ".join((f"{byte:02X}" for byte in message.data))}"
                    )
                case _:
                    print(
                        f"RX ID: 0x{message.arbitration_id:03X}, Undefined, Data: 0x{" ".join((f"{byte:02X}" for byte in message.data))}"
                    )

    def send_pid(self):
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x4,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=6,
                data=struct.pack(
                    "<HHH",
                    int(self.p.get() * 256),
                    int(self.i.get() * 256),
                    int(self.d.get() * 256),
                ),
            ),
            False,
        )

    def send_alpha(self):
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x3,
                is_extended_id=False,
                is_remote_frame=False,
                dlc=2,
                data=struct.pack("<H", int(self.alpha.get() * 32768)),
            ),
            False,
        )

    def send_soft_limits(self):
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x5,
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
                arbitration_id=int(self.id.get() + "0", 16) | 0x6,
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

    def send_ping(self):
        self.ping_reply.set("Pinging")
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0xE,
                is_extended_id=False,
                dlc=8,
                data=struct.pack("<Q", int(time.time() * 1000)),
            )
        )

    def send_stop(self):
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0xC,
                is_extended_id=False,
                dlc=0,
            ),
            False,
        )

    def send_open_loop(self):
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x2,
                is_extended_id=False,
                data=struct.pack(
                    "<Bh",
                    0 + self.ignore_limit.get(),
                    int(self.duty_cycle.get() * 32768),
                ),
                dlc=3,
            ),
            False,
        )

    def send_target_position(self):
        target = int(self.target.get())
        self.debugData[8, -1] = target
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x2,
                is_extended_id=False,
                data=struct.pack(
                    "<BHi",
                    2 + self.ignore_limit.get(),
                    int(self.error_gain.get() * 1024),
                    target,
                ),
                dlc=7,
            ),
            False,
        )

    def send_target_velocity(self):
        target = int(self.target.get())
        self.debugData[8, -1] = target
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x2,
                is_extended_id=False,
                data=struct.pack(
                    "<BHi",
                    4 + self.ignore_limit.get(),
                    int(self.error_gain.get() * 1024),
                    target,
                ),
                dlc=7,
            ),
            False,
        )

    def send_target_current(self):
        target = int(self.target.get())
        self.debugData[8, -1] = target
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x2,
                is_extended_id=False,
                data=struct.pack(
                    "<BHh",
                    6 + self.ignore_limit.get(),
                    int(self.error_gain.get() * 1024),
                    target,
                ),
                dlc=5,
            ),
            False,
        )

    def send_debug_telemetry(self, enabled: bool):
        self.can_send.put(
            can.Message(
                arbitration_id=int(self.id.get() + "0", 16) | 0x7,
                is_extended_id=False,
                data=struct.pack("<B", 1 if enabled else 0),
                dlc=5,
            ),
            False,
        )


def app_main(can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue):
    root = tk.Tk()
    root.wm_title("Smart Motor Controller CAN UI")
    app = App(root, can_send, can_recv)
    app.mainloop()


def can_main(can_send: multiprocessing.Queue, can_recv: multiprocessing.Queue):
    bus = can.Bus(channel=0, interface="gs_usb", bitrate=125 * 1000)
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
