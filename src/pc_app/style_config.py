# Colors used in the app.
BG = "#0b0f0c"
PANEL_BG = "#111812"
BORDER = "#2b3b2f"
GRID = "#1f2d23"
TEXT = "#b7d3b0"
DIM = "#6f8a6c"
OK = "#5cff8a"
ERR = "#ff5a4d"
MONO = "DejaVu Sans Mono"

STYLESHEET = f"""
QMainWindow, QWidget#central {{
    background: {BG};
}}
QWidget {{
    color: {TEXT};
    font-family: "Consolas", "Courier New", monospace;
    font-size: 11px;
}}
QGroupBox {{
    background: {PANEL_BG};
    border: 1px solid {BORDER};
    margin-top: 9px;
    padding: 10px 6px 6px 6px;
    font-weight: bold;
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 8px;
    padding: 0 4px;
    color: {OK};
    background: {BG};
}}
QLabel {{
    background: transparent;
    color: {DIM};
}}
QLineEdit {{
    background: {BG};
    border: 1px solid {BORDER};
    padding: 3px 5px;
    color: {OK};
    selection-background-color: {OK};
    selection-color: {BG};
}}
QLineEdit:focus {{
    border: 1px solid {OK};
}}
QPushButton {{
    background: {BG};
    border: 1px solid {DIM};
    padding: 4px 8px;
    color: {TEXT};
    text-transform: uppercase;
}}
QPushButton:hover {{
    background: {OK};
    color: {BG};
    border: 1px solid {OK};
}}
QPushButton:pressed {{
    background: {TEXT};
    color: {BG};
}}
QCheckBox {{
    background: transparent;
    spacing: 7px;
}}
QCheckBox::indicator {{
    width: 11px;
    height: 11px;
    border: 1px solid {DIM};
    background: {BG};
}}
QCheckBox::indicator:checked {{
    background: {OK};
    border: 1px solid {OK};
}}
QToolBar {{
    background: #cfc9ae;
    border: 1px solid {BORDER};
    spacing: 2px;
}}
QToolBar QLabel {{
    color: #2a2a22;
    background: transparent;
}}
"""

# Colors used in the plots.
VENT_BAND_ALPHA = 0.45
VENTS_OPENED = "#0e7f15"
COLOR_TEMP_INSIDE = "#ffb347"
COLOR_TEMP_OUTSIDE = "#ff5300"
COLOR_RH_INSIDE = "#c3b7ff"
COLOR_RH_OUTSIDE = "#7f519d"
COLOR_LIGHT = "#fff36b"
COLOR_PRESSURE = "#3dedd4"
COLOR_MOISTURE = "#4f18ff"
COLOR_MOISTURE_WATERED = "#4fb8ff"

# (Plot title, unit, [(label, colour), ...], minimum y-axis span)
PLOT_SPECS = [
    ("TEMPERATURE", "°C", [("inside", COLOR_TEMP_INSIDE), ("outside", COLOR_TEMP_OUTSIDE)], 5.0),
    ("REL. HUMIDITY", "%", [("inside", COLOR_RH_INSIDE), ("outside", COLOR_RH_OUTSIDE)], 10.0),
    ("LIGHT", "lx", [("light", COLOR_LIGHT)], 100.0),
    ("PRESSURE", "hPa", [("pressure", COLOR_PRESSURE)], 2.0),
    ("MOISTURE", "%", [("moisture", COLOR_MOISTURE)], 20.0),
]

# Title offsets (to the left).
TITLE_OFFSET_PX = 20 
AX_TEMP, AX_RH, AX_LIGHT, AX_PRESSURE, AX_MOISTURE = range(5)