import math
import sys
from dataclasses import dataclass

import serial
from serial.tools import list_ports

from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QFrame,
)

from matplotlib.backends.backend_qtagg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar,
)
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator

from style_config import *

@dataclass
class ControlData:
    rh: float
    temp: float
    moisture: float

# Firmware constants.
MAX_MEASUREMENTS = 576
DATA_PACKET_LENGTH = 21
CONTROL_SETTINGS_LENGTH = 10

class DataManager:
    # Moisture voltage calibration.
    # Max voltage is dry, min voltage is moist.
    MOISTURE_MAX_VOLTAGE = 2.35
    MOISTURE_MIN_VOLTAGE = 1.65
    
    # Firmware constants.
    MEASUREMENT_PERIOD_SECS = 60

    VENT_ON_FLAG = (1 << 6)
    WATER_PUMPED_FLAG = (1 << 5)

    SHT31_ERR_FLAG = (1)
    BME280_ERR_FLAG = (1 << 1)
    BH1750_ERR_FLAG = (1 << 2)
    ADC_ERR_FLAG = (1 << 3)

    def __init__(self):
        self.data_buffer = None
        self._clear_processed_data()

    def _clear_processed_data(self):
        self.temp_inside = []
        self.temp_outside = []
        self.rh_inside = []
        self.rh_outside = []
        self.light = []
        self.pressure = []
        self.moisture = []
        self.vent_open_spans = []
        self.water_pumped_events = []
        self.time = []
        self.dt = None
        self.time_unit = None

    def _update_time(self, data_length):
        operation_time_sec = (data_length // 21) * self.MEASUREMENT_PERIOD_SECS
        if (operation_time_sec > 60 * 60):
            # Use hours as our time unit.
            self.dt = self.MEASUREMENT_PERIOD_SECS / (60 * 60)
            self.time_unit = "h"
        elif (operation_time_sec > 60):
            # Use minutes.
            self.dt = self.MEASUREMENT_PERIOD_SECS / (60)
            self.time_unit = "min"
        else:
            # Seconds.
            self.dt = self.MEASUREMENT_PERIOD_SECS
            self.time_unit = "s"

        self.time = [i * self.dt for i in range(data_length // 21)]

    def _calculate_raw_voltage(self, moisture):
        moisture /= 100.0
        return self.MOISTURE_MIN_VOLTAGE + (1 - moisture) * (self.MOISTURE_MAX_VOLTAGE - self.MOISTURE_MIN_VOLTAGE)

    def _calculate_moisture(self, raw_voltage):
        normalized_voltage = (raw_voltage - self.MOISTURE_MIN_VOLTAGE) / (self.MOISTURE_MAX_VOLTAGE - self.MOISTURE_MIN_VOLTAGE)
        return max(0.0, min(100.0, (1.0 - normalized_voltage) * 100.0))

    def process_data_buffer(self, data: bytes) -> None:
        self.data_buffer = data
        self._clear_processed_data()
        self._update_time(len(data))
        
        # Keep track of the spans where vents were open.
        span_start = None
        sample_idx = -1
        for i in range(0, len(data), DATA_PACKET_LENGTH):
            chunk = data[i:i + DATA_PACKET_LENGTH]
            sample_idx = i // DATA_PACKET_LENGTH

            # Update the open vent spans.
            vents_open = bool(chunk[0] & self.VENT_ON_FLAG)
            if vents_open and span_start is None:
                span_start = sample_idx
            elif not vents_open and span_start is not None:
                self.vent_open_spans.append((span_start, sample_idx))
                span_start = None

            # Add watering events.
            if (chunk[0] & self.WATER_PUMPED_FLAG):
                self.water_pumped_events.append(sample_idx)

            # Add measurements from SHT31.
            if (chunk[0] & self.SHT31_ERR_FLAG):
                self.temp_inside.append(float("nan"))
                self.rh_inside.append(float("nan"))
            else:
                temp_inside_raw = int.from_bytes(chunk[1:3], byteorder="big", signed=False)
                self.temp_inside.append(-45.0 + 175.0 * (temp_inside_raw / 65535.0))
                rh_inside_raw = int.from_bytes(chunk[3:5], byteorder="big", signed=False)
                self.rh_inside.append(100.0 * (rh_inside_raw / 65535.0))
            
            # Add measurement from BH1750.
            if (chunk[0] & self.BH1750_ERR_FLAG):
                self.light.append(float("nan"))
            else:
                self.light.append(int.from_bytes(chunk[5:7], byteorder="big", signed=False))
            
            # Add measurements from BME280.
            if (chunk[0] & self.BME280_ERR_FLAG):
                self.temp_outside.append(float("nan"))
                self.rh_outside.append(float("nan"))
                self.pressure.append(float("nan"))
            else:
                self.temp_outside.append(int.from_bytes(chunk[7:11], byteorder="big", signed=True) / 100.0)
                self.rh_outside.append(int.from_bytes(chunk[11:15], byteorder="big", signed=False) / 1024.0)
                self.pressure.append(int.from_bytes(chunk[15:19], byteorder="big", signed=False) / 25600.0)
            
            # Add moisture measurement, converted to percents from the raw voltage.
            if (chunk[0] & self.ADC_ERR_FLAG):
                self.moisture.append(float("nan"))
            else:
                raw_voltage = (3.3 / 4096.0) * int.from_bytes(chunk[19:21], byteorder="big", signed=False)
                self.moisture.append(self._calculate_moisture(raw_voltage))

        if span_start is not None:
            self.vent_open_spans.append((span_start, sample_idx))

    def convert_ctrl_data_from_raw(self, raw_ctrl: bytes) -> ControlData:
        """Converts raw control data payload to the ControlData dataclass."""
        rh = int.from_bytes(raw_ctrl[0:4], byteorder="big", signed=False) / 1024.0
        temp = int.from_bytes(raw_ctrl[4:8], byteorder="big", signed=True) / 100.0
        moisture = self._calculate_moisture((3.3 / 4096.0) * int.from_bytes(raw_ctrl[8:10], byteorder="big", signed=False))
        return ControlData(rh, temp, moisture)

    def convert_ctrl_data_to_raw(self, ctrl: ControlData) -> bytes:
        """Converts ControlData to a raw data payload."""
        rh_raw = round(ctrl.rh * 1024.0)
        temp_raw = round(ctrl.temp * 100.0)
        voltage_raw = round(self._calculate_raw_voltage(ctrl.moisture) * 4096.0 / 3.3)
        return (
            rh_raw.to_bytes(4, byteorder="big", signed=False)
            + temp_raw.to_bytes(4, byteorder="big", signed=True)
            + voltage_raw.to_bytes(2, byteorder="big", signed=False)
        )


class CommunicationManager:
    # Firmware UART constants.
    UART_GET_LAST_MEASUREMENT = 0x0
    UART_GET_ALL_MEASUREMENTS = 0x1
    UART_CHANGE_CONTROL = 0x3
    UART_GET_CONTROL = 0x2
    UART_OK = 0x4
    UART_NOT_OK = 0x5

    PL2303_VID = 0x067B
    PL2303_PID = 0x23A3

    def __init__(self):
        self.port = None
        self.data_manager = DataManager()
        
    def initialize(self, port_name: str = None) -> bool:
        if (self.port != None):
            self.port.close()
            self.port = None
        com_port = port_name
        if com_port is None:
            com_port = self._find_pl2303_port()
        if com_port is not None:
            self._initialize_port(com_port)
        if self.port is not None:
            return True
        else:
            return False

    def _find_pl2303_port(self) -> str:
        matches = [
            p.device
            for p in list_ports.comports()
            if p.vid == self.PL2303_VID and p.pid == self.PL2303_PID
        ]

        if len(matches) == 0:
            return None

        return matches[0]

    def _initialize_port(self, port_name: str) -> None:
        try:
            if (self.port != None):
                self.port.close()
                self.port = None

            # Initialize the port to match the firmware settings.
            port = serial.Serial(
                port=port_name,
                baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=3.0,
                inter_byte_timeout=0.030,
                write_timeout=0.05,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )

            if port.is_open:
                self.port = port   
            else:
                port.close()
                self.port = None
        except serial.SerialException:
            return

    def _write(self, data: bytes) -> bool:
        try:
            return self.port.write(data) == len(data)
        except serial.SerialException:
            return False

    def _read(self, length: int) -> bytes:
        try:
            return self.port.read(length)
        except serial.SerialException:
            return b""

    def read_control(self) -> (ControlData):
        if self.port is None:
            return None
        if self._write(bytes([self.UART_GET_CONTROL])):
            read_data = self._read(CONTROL_SETTINGS_LENGTH)
            if len(read_data) == CONTROL_SETTINGS_LENGTH:
                return self.data_manager.convert_ctrl_data_from_raw(read_data)
            else:
                return None
        else:
            return None

    def set_control(self, ctrl: ControlData) -> bool:
        if self.port is None:
            return False
        raw_ctrl = self.data_manager.convert_ctrl_data_to_raw(ctrl)
        if self._write(bytes([self.UART_CHANGE_CONTROL]) + raw_ctrl):
            read_data = self._read(1)
            if len(read_data) == 1 and read_data[0] == self.UART_OK:
                return True
            else:
                return False
        else:
            return False

    def read_all_measurements(self) -> bool:
        if self.port is None:
            return False
        if self._write(bytes([self.UART_GET_ALL_MEASUREMENTS])):
            read_data = self._read(MAX_MEASUREMENTS * DATA_PACKET_LENGTH)
            if len(read_data) > 0 and len(read_data) % DATA_PACKET_LENGTH == 0:
                self.data_manager.process_data_buffer(read_data)
                return True
            else:
                return False
        else:
            return False


class MainWindow(QMainWindow):
    # Window geometry.
    WINDOW_W = 900
    WINDOW_H = 550
    SIDEBAR_WIDTH = 260
    OUTER_MARGIN = 8
    COLUMN_SPACING = 10
    TOOLBAR_H = 28
    DPI = 100

    # Readings ranges.
    TEMP_RANGE = (-40.0, 125.0)
    RH_RANGE = (0.0, 100.0)
    MOISTURE_RANGE = (0.0, 100.0)


    def __init__(self):
        super().__init__()

        self.communication_manager = CommunicationManager()

        self.setWindowTitle("Greenhouse Monitor")
        self.setFixedSize(self.WINDOW_W, self.WINDOW_H)
        self.setStyleSheet(STYLESHEET)

        central = QWidget()
        central.setObjectName("central")
        main_layout = QHBoxLayout(central)
        m = self.OUTER_MARGIN
        main_layout.setContentsMargins(m, m, m, m)
        main_layout.setSpacing(self.COLUMN_SPACING)
        self.setCentralWidget(central)

        self._build_plot_panel(main_layout)
        self._build_control_panel(main_layout)

        self.update_plots()


    def _plot_size_px(self):
        w = (
            self.WINDOW_W
            - 2 * self.OUTER_MARGIN
            - self.COLUMN_SPACING
            - self.SIDEBAR_WIDTH
        )
        h = self.WINDOW_H - 2 * self.OUTER_MARGIN - self.TOOLBAR_H
        return w, h

    def _build_plot_panel(self, parent_layout):
        w_px, h_px = self._plot_size_px()

        self.plot_panel = QWidget()
        self.plot_panel.setFixedSize(w_px, h_px + self.TOOLBAR_H)

        layout = QVBoxLayout(self.plot_panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.figure = Figure(
            figsize=(w_px / self.DPI, h_px / self.DPI),
            dpi=self.DPI,
            facecolor=BG,
        )
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setFixedSize(w_px, h_px)

        self.toolbar = NavigationToolbar(self.canvas, self.plot_panel)
        self.toolbar.setFixedHeight(self.TOOLBAR_H)

        # 3 rows and 2 columns, with the bottom plot spanning both columns.
        gs = self.figure.add_gridspec(
            3,
            2,
            left=0.075,
            right=0.975,
            top=0.935,
            bottom=0.105,
            hspace=0.45,
            wspace=0.22,
        )
        self.axes = [
            self.figure.add_subplot(gs[0, 0]),  # temperature
            self.figure.add_subplot(gs[0, 1]),  # humidity
            self.figure.add_subplot(gs[1, 0]),  # light
            self.figure.add_subplot(gs[1, 1]),  # pressure
            self.figure.add_subplot(gs[2, :]),  # moisture
        ]

        self.lines = []
        self.event_artists = []
        self.min_spans = [spec[3] for spec in PLOT_SPECS]

        for ax, (title, unit, line_specs, _min_span) in zip(self.axes, PLOT_SPECS):
            self._style_axis(ax, title, unit)
 
            plot_lines = []
            for label, colour in line_specs:
                (line,) = ax.plot([], [], color=colour, linewidth=1.2, label=label)
                plot_lines.append(line)
 
            if len(line_specs) > 1:
                ax.legend(
                    loc="lower right",
                    bbox_to_anchor=(1.0, 1.0),
                    borderaxespad=0.2,
                    ncol=len(line_specs),
                    frameon=False,
                    labelcolor=TEXT,
                    handlelength=1.2,
                    handletextpad=0.4,
                    columnspacing=0.8,
                    prop={"family": MONO, "size": 6.5},
                )
 
            self.lines.append(plot_lines)
            ax.set_xlim(0, 1)

        # Label the time only on the bottom plot.
        self.axes[-1].set_xlabel(
            "TIME (s)",
            color=DIM,
            fontsize=8,
            family=MONO,
            labelpad=3,
        )

        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)

        parent_layout.addWidget(self.plot_panel, 0)

    @staticmethod
    def _style_axis(ax, title, unit):
        ax.set_facecolor(BG)
        ax.set_title(
            f"{title}  [{unit}]",
            loc="left",
            x=-TITLE_OFFSET_PX / (ax.get_position().width * ax.figure.get_figwidth() * ax.figure.dpi),
            color=TEXT,
            fontsize=8.5,
            fontweight="bold",
            family=MONO,
            pad=5,
        )
        ax.grid(True, color=GRID, linewidth=0.8, linestyle="--")
        ax.set_axisbelow(True)
        ax.ticklabel_format(axis="y", style="plain", useOffset=False)
        ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
        ax.tick_params(
            colors=DIM,
            labelsize=7,
            length=3,
            width=0.8,
            pad=2,
        )
        for label in ax.get_xticklabels() + ax.get_yticklabels():
            label.set_family(MONO)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(BORDER)

    def _build_control_panel(self, parent_layout):
        self.control_panel = QWidget()
        self.control_panel.setFixedWidth(self.SIDEBAR_WIDTH)

        layout = QVBoxLayout(self.control_panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        actions_group = QGroupBox("CONNECTION")
        actions_layout = QVBoxLayout(actions_group)
        actions_layout.setSpacing(6)

        self.connect_button = QPushButton("Connect")
        self.connect_button.clicked.connect(self.connect_device)

        actions_layout.addWidget(self.connect_button)
        layout.addWidget(actions_group)

        settings_group = QGroupBox("CONTROL SETTINGS")
        settings_layout = QVBoxLayout(settings_group)
        settings_layout.setSpacing(4)

        self.get_ctrl_button = QPushButton("See current control settings")
        self.get_ctrl_button.clicked.connect(self.get_ctrl_settings)
        settings_layout.addWidget(self.get_ctrl_button)
        settings_layout.addSpacing(3)

        settings_layout.addWidget(QLabel("Temperature"))
        self.temp_input = QLineEdit()
        self.temp_input.setPlaceholderText("Enter temperature in °C")
        settings_layout.addWidget(self.temp_input)

        settings_layout.addWidget(QLabel("Humidity"))
        self.rh_input = QLineEdit()
        self.rh_input.setPlaceholderText("Enter relative humidity in %")
        settings_layout.addWidget(self.rh_input)

        settings_layout.addWidget(QLabel("Moisture"))
        self.moist_input = QLineEdit()
        self.moist_input.setPlaceholderText("Enter moisture in %")
        settings_layout.addWidget(self.moist_input)

        self.apply_button = QPushButton("Apply control settings")
        self.apply_button.clicked.connect(self.apply_settings)
        settings_layout.addSpacing(2)
        settings_layout.addWidget(self.apply_button)
        layout.addWidget(settings_group)

        plot_group = QGroupBox("PLOT")
        plot_layout = QVBoxLayout(plot_group)
        plot_layout.setSpacing(6)

        self.refresh_button = QPushButton("Refresh plots")
        self.refresh_button.clicked.connect(self.update_plots)
        plot_layout.addWidget(self.refresh_button)
        layout.addWidget(plot_group)

        status_group = QGroupBox("STATUS")
        status_layout = QVBoxLayout(status_group)

        self.status_label = QLabel("Ready.")
        self.status_label.setWordWrap(True)
        status_layout.addWidget(self.status_label)
        layout.addWidget(status_group)

        layout.addStretch(1)
        parent_layout.addWidget(self.control_panel, 0)

        self.set_status("Ready.")

        legend_group = QGroupBox("LEGEND")
        legend_layout = QVBoxLayout(legend_group)
        legend_layout.setSpacing(4)

        vent = QColor(VENTS_OPENED)
        vent_rgba = f"rgba({vent.red()}, {vent.green()}, {vent.blue()}, {int(VENT_BAND_ALPHA * 255)})"
        legend_layout.addLayout(
            self._legend_row(f"background: {vent_rgba};", "Vents open")
        )

        pump_style = (
            "border-left: 8px solid transparent;"
            "border-right: 8px solid transparent;"
            f"background: {COLOR_MOISTURE_WATERED};"
            "background-clip: padding;"
        )
        legend_layout.addLayout(self._legend_row(pump_style, "Water pumped"))
        layout.addWidget(legend_group)

    @staticmethod
    def _legend_row(swatch_style, text):
        row = QHBoxLayout()
        swatch = QFrame()
        swatch.setFixedSize(18, 12)
        swatch.setStyleSheet(swatch_style)
        row.addWidget(swatch)
        label = QLabel(text)
        label.setWordWrap(True)
        row.addWidget(label, 1)
        return row

    def set_status(self, message, error=False):
        colour = ERR if error else OK
        self.status_label.setText(f"> {message}")
        self.status_label.setStyleSheet(
            f"color: {colour}; font-weight: bold; background: transparent;"
        )

    def connect_device(self):
        if not self.communication_manager.initialize():
            self.set_status("No PL2303 device found / could not open port.", error=True)
            return
        port_name = self.communication_manager.port.port
        self.set_status(f"Connected on {port_name}.")
 
    def get_ctrl_settings(self):
        if self.communication_manager.port is None:
            self.set_status("Not connected.", error=True)
            return
        ctrl = self.communication_manager.read_control()
        if ctrl is None:
            self.set_status("Could not read control settings.", error=True)
            return
        self.temp_input.setText(f"{ctrl.temp:.2f}")
        self.rh_input.setText(f"{ctrl.rh:.1f}")
        self.moist_input.setText(f"{ctrl.moisture:.1f}")
        self.set_status("Control settings loaded.")
 
    @staticmethod
    def _parse_value(text, name, low, high, unit):
        """Parse a float from text and range-check it. Returns (value, error)."""
        text = text.strip().replace(",", ".")
        if not text:
            return None, f"{name} is empty."
        try:
            value = float(text)
        except ValueError:
            return None, f"{name} must be a number."
        if not math.isfinite(value) or not (low <= value <= high):
            return None, f"{name} must be {low:g} to {high:g} {unit}."
        return value, None
 
    def apply_settings(self):
        if self.communication_manager.port is None:
            self.set_status("Not connected.", error=True)
            return
 
        temp, err = self._parse_value(self.temp_input.text(), "Temperature", *self.TEMP_RANGE, "°C")
        if err:
            self.set_status(err, error=True)
            return
        rh, err = self._parse_value(self.rh_input.text(), "Humidity", *self.RH_RANGE, "%")
        if err:
            self.set_status(err, error=True)
            return
        moisture, err = self._parse_value(self.moist_input.text(), "Moisture", *self.MOISTURE_RANGE, "%")
        if err:
            self.set_status(err, error=True)
            return
 
        if self.communication_manager.set_control(ControlData(rh=rh, temp=temp, moisture=moisture)):
            self.set_status("Control settings applied.")
        else:
            self.set_status("Device did not accept the settings.", error=True)
 

    def _draw_events(self, dm):
        """Redraw vent bands and pump lines."""
        # Remove the previous event artists first.
        for artist in self.event_artists:
            artist.remove()
        self.event_artists.clear()
 
        # Draw vent open bands.
        for ax in (self.axes[AX_TEMP], self.axes[AX_RH]):
            for start_idx, end_idx in dm.vent_open_spans:
                if end_idx == start_idx:
                    end_idx += 1  # a one-sample span still gets a visible width
                band = ax.axvspan(
                    start_idx * dm.dt,
                    end_idx * dm.dt,
                    color=VENTS_OPENED,
                    alpha=VENT_BAND_ALPHA,
                    linewidth=0,
                    zorder=0,
                )
                self.event_artists.append(band)
 
        # Each pump event is a vertical line.
        moisture_ax = self.axes[AX_MOISTURE]
        for idx in dm.water_pumped_events:
            line = moisture_ax.axvline(
                idx * dm.dt,
                color=COLOR_MOISTURE_WATERED,
                linewidth=1.0,
                zorder=1,
            )
            self.event_artists.append(line)
 
    def update_plots(self):
        if not self.communication_manager.read_all_measurements():
            self.set_status("Plots could not be updated.", error=True)
            return
        
        dm = self.communication_manager.data_manager
        x = dm.time
 
        self.lines[AX_TEMP][0].set_data(x, dm.temp_inside)
        self.lines[AX_TEMP][1].set_data(x, dm.temp_outside)
 
        self.lines[AX_RH][0].set_data(x, dm.rh_inside)
        self.lines[AX_RH][1].set_data(x, dm.rh_outside)
 
        self.lines[AX_LIGHT][0].set_data(x, dm.light)
        self.lines[AX_PRESSURE][0].set_data(x, dm.pressure)
        self.lines[AX_MOISTURE][0].set_data(x, dm.moisture)
 
        x_max = max(x[-1], dm.dt)
        for ax in self.axes:
            ax.set_xlim(0, x_max)
 
        self._draw_events(dm)
 
        self.axes[-1].set_xlabel(
            f"TIME ({dm.time_unit})",
            color=DIM,
            fontsize=8,
            family=MONO,
            labelpad=3,
        )
 
        for ax, min_span in zip(self.axes, self.min_spans):
            ax.relim()
            ax.autoscale_view(scalex=False)
            ax.margins(y=0.12)

            low, high = ax.get_ylim()
            if high - low < min_span:
                mid = (low + high) / 2
                ax.set_ylim(mid - min_span / 2, mid + min_span / 2, auto=True)
    
        self.canvas.draw_idle()
        self.set_status("Plots updated.")


def main():
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()