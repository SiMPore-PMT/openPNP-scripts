# adhesive_calibration_gui.py
#
# Simple adhesive dispense calibration GUI for OpenPnP Jython 2.7.

import imp
import json
import os
import sys

from java.awt import BorderLayout, Dimension, FlowLayout, GridBagConstraints, GridBagLayout, Insets
from java.lang import Runnable
from java.util.concurrent import Callable
from javax.swing import (
    BorderFactory, Box, BoxLayout, JButton, JCheckBox, JComboBox, JFrame, JLabel,
    JOptionPane, JPanel, JScrollPane, JSplitPane, JTextArea, JTextField,
    SwingUtilities, SwingWorker
)
from org.openpnp.model import Configuration, LengthUnit
from org.openpnp.spi import Camera as SpiCamera


def existing_dir(path):
    try:
        if path and os.path.isdir(str(path)):
            return os.path.abspath(str(path))
    except:
        pass
    return None


def candidate_script_dirs():
    out = []
    try:
        out.append(scripting.getScriptsDirectory().toString())
    except:
        pass
    try:
        out.append(os.path.dirname(os.path.abspath(__file__)))
    except:
        pass
    try:
        out.append(os.path.join(os.path.expanduser("~"), ".openpnp2", "scripts"))
    except:
        pass
    out.append(os.getcwd())
    seen = {}
    unique = []
    for item in out:
        path = existing_dir(item)
        if path is not None and path not in seen:
            seen[path] = True
            unique.append(path)
    return unique


def resolve_scripts_root():
    for path in candidate_script_dirs():
        backend = os.path.join(path, "Events", "Homing", "adhesive_calibration.py")
        if os.path.exists(backend):
            return path
    candidates = candidate_script_dirs()
    if len(candidates):
        return candidates[0]
    return os.getcwd()


SCRIPT_DIR = resolve_scripts_root()
HOMING_DIR = os.path.join(SCRIPT_DIR, "Events", "Homing")
if HOMING_DIR not in sys.path:
    sys.path.append(HOMING_DIR)
BACKEND_PATH = os.path.join(HOMING_DIR, "adhesive_calibration.py")
CONFIG_PATH = os.path.join(SCRIPT_DIR, "adhesive_calibration_config.json")


def load_backend():
    return imp.load_source("adhesive_calibration_backend", BACKEND_PATH)


def reload_backend():
    global adhesive_calibration
    adhesive_calibration = load_backend()
    return adhesive_calibration


adhesive_calibration = load_backend()


IDLE = "IDLE"
RUNNING = "RUNNING"
READY = "READY"
ERROR = "ERROR"
SAVED = "SAVED"


def get_machine():
    try:
        return machine
    except:
        return Configuration.get().getMachine()


def require_machine_execute():
    machine_obj = get_machine()
    if not hasattr(machine_obj, "execute"):
        raise Exception("OpenPnP machine task execution is not available")
    return machine_obj


def load_config_file():
    return adhesive_calibration.load_config(CONFIG_PATH)


def save_config_file(cfg):
    return adhesive_calibration.save_config(cfg, CONFIG_PATH)


def compact_json(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"))


def combo_text(combo):
    item = combo.getSelectedItem()
    if item is None:
        return ""
    return str(item).strip()


def unique_sorted(values):
    seen = {}
    out = []
    for value in values:
        text = str(value or "").strip()
        if text and text not in seen:
            seen[text] = True
            out.append(text)
    out.sort()
    return out


def available_nozzle_names():
    machine_obj = get_machine()
    names = []
    for head in adhesive_calibration.all_heads(machine_obj):
        try:
            for nozzle in adhesive_calibration.java_list_to_py(head.getNozzles()):
                names.append(adhesive_calibration.object_name(nozzle))
        except:
            pass
    return unique_sorted(names)


def available_actuator_names():
    machine_obj = get_machine()
    names = []
    owners = [machine_obj]
    for head in adhesive_calibration.all_heads(machine_obj):
        owners.append(head)
    for owner in owners:
        try:
            for actuator in adhesive_calibration.java_list_to_py(owner.getActuators()):
                names.append(adhesive_calibration.object_name(actuator))
        except:
            pass
    return unique_sorted(names)


def find_top_camera(machine_obj):
    try:
        camera = machine_obj.getDefaultHead().getDefaultCamera()
        if camera is not None:
            return camera
    except:
        pass
    try:
        for camera in adhesive_calibration.java_list_to_py(machine_obj.getCameras()):
            try:
                if camera.getLooking() == SpiCamera.Looking.Down:
                    return camera
            except:
                pass
    except:
        pass
    for head in adhesive_calibration.all_heads(machine_obj):
        try:
            for camera in adhesive_calibration.java_list_to_py(head.getCameras()):
                try:
                    if camera.getLooking() == SpiCamera.Looking.Down:
                        return camera
                except:
                    pass
        except:
            pass
    raise Exception("No top camera found")


def set_combo_items(combo, values, selected):
    current = str(selected or "").strip()
    combo.removeAllItems()
    if current:
        combo.addItem(current)
    for value in values:
        text = str(value or "").strip()
        if text and text != current:
            combo.addItem(text)
    combo.setSelectedItem(current)


class AdhesiveCalibrationGui(object):
    def __init__(self):
        self.cfg = load_config_file()
        self.saved_snapshot = compact_json(adhesive_calibration.public_clean_config(self.cfg))
        self.worker_running = False
        self.state = IDLE
        self.fields = {}
        self.combos = {}
        self.checks = {}
        self.action_buttons = []
        self.status_area = JTextArea()
        self.result_area = JTextArea()
        self.log_area = JTextArea()

    def build(self):
        self.frame = JFrame("Adhesive Calibration")
        self.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)

        root = JPanel(BorderLayout(6, 6))
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))
        root.add(self.build_buttons(), BorderLayout.NORTH)

        split = JSplitPane(JSplitPane.VERTICAL_SPLIT)
        split.setResizeWeight(0.72)
        split.setTopComponent(JScrollPane(self.build_form()))
        split.setBottomComponent(self.build_bottom_panel())
        root.add(split, BorderLayout.CENTER)

        self.frame.add(root, BorderLayout.CENTER)
        self.load_into_ui(self.cfg)
        self.refresh_selectors()
        self.set_state(IDLE)
        self.frame.setPreferredSize(Dimension(760, 820))
        self.frame.pack()
        self.frame.setLocationRelativeTo(None)
        self.frame.setVisible(True)

    def build_buttons(self):
        panel = JPanel(FlowLayout(FlowLayout.LEFT, 5, 2))
        specs = [
            ("Load Config", self.load_action),
            ("Save Config", self.save_config_action),
            ("Refresh Lists", self.refresh_selectors),
            ("Grab Nozzle DRO", lambda: self.capture_origin_action("nozzle")),
            ("Grab Camera DRO", lambda: self.capture_origin_action("camera")),
            ("Go To Origin Safe Z", self.go_origin_action),
            ("Run Calibration", self.run_calibration_action)
        ]
        for label, callback in specs:
            button = JButton(label)
            button.addActionListener(lambda event, cb=callback: cb())
            self.action_buttons.append(button)
            panel.add(button)
        return panel

    def build_form(self):
        panel = JPanel()
        panel.setLayout(BoxLayout(panel, BoxLayout.Y_AXIS))
        panel.add(self.build_selector_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_origin_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_motion_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_line_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_matrix_section())
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.build_park_section())
        panel.add(Box.createVerticalGlue())
        return panel

    def build_bottom_panel(self):
        panel = JPanel(BorderLayout(4, 4))
        self.configure_text_area(self.status_area, 2)
        self.configure_text_area(self.result_area, 5)
        self.configure_text_area(self.log_area, 9)
        top = JPanel(BorderLayout(4, 4))
        top.add(JLabel("Status"), BorderLayout.NORTH)
        top.add(JScrollPane(self.status_area), BorderLayout.CENTER)
        top.add(JScrollPane(self.result_area), BorderLayout.SOUTH)
        panel.add(top, BorderLayout.NORTH)
        panel.add(JScrollPane(self.log_area), BorderLayout.CENTER)
        return panel

    def configure_text_area(self, area, rows):
        area.setRows(rows)
        area.setEditable(False)
        area.setLineWrap(True)
        area.setWrapStyleWord(True)

    def section(self, title):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder(title))
        return panel

    def gbc(self, x, y, weightx):
        c = GridBagConstraints()
        c.gridx = x
        c.gridy = y
        c.insets = Insets(2, 4, 2, 4)
        c.anchor = GridBagConstraints.WEST
        c.fill = GridBagConstraints.HORIZONTAL
        c.weightx = weightx
        return c

    def add_row(self, panel, row, label, component):
        panel.add(JLabel(label), self.gbc(0, row, 0.0))
        panel.add(component, self.gbc(1, row, 1.0))

    def add_field(self, panel, row, key, label, width=10):
        field = JTextField(width)
        self.fields[key] = field
        self.add_row(panel, row, label, field)
        return field

    def build_selector_section(self):
        panel = self.section("OpenPnP Objects")
        specs = [("nozzle_name", "Nozzle"), ("actuator_name", "Dispense Actuator")]
        for row, spec in enumerate(specs):
            key, label = spec
            combo = JComboBox()
            combo.setEditable(True)
            combo.setPreferredSize(Dimension(300, combo.getPreferredSize().height))
            self.combos[key] = combo
            self.add_row(panel, row, label, combo)
        return panel

    def build_origin_section(self):
        panel = self.section("Work Area Origin")
        specs = [
            ("origin_location.x", "X mm"),
            ("origin_location.y", "Y mm"),
            ("origin_location.z", "Z mm"),
            ("origin_location.rotation", "R deg")
        ]
        for row, spec in enumerate(specs):
            self.add_field(panel, row, spec[0], spec[1])
        return panel

    def build_motion_section(self):
        panel = self.section("Motion And Trigger")
        specs = [
            ("safe_z_mm", "Safe Z mm"),
            ("travel_speed", "Travel Speed"),
            ("dispense_speed", "Dispense Speed"),
            ("dispense_z_offset_mm", "Dispense Z Offset mm"),
            ("pre_trigger_dwell_ms", "Pre Trigger Dwell ms"),
            ("trigger_pulse_ms", "Trigger On Dwell ms"),
            ("actuator_settle_ms", "Actuator Settle ms")
        ]
        for row, spec in enumerate(specs):
            self.add_field(panel, row, spec[0], spec[1])
        return panel

    def build_line_section(self):
        panel = self.section("Line")
        enabled = JCheckBox("Enabled")
        self.checks["line.enabled"] = enabled
        panel.add(enabled, self.gbc(0, 0, 0.0))
        specs = [
            ("line.length_mm", "Length mm"),
            ("line.pitch_mm", "Pitch mm"),
            ("line.lift_mm", "Lift mm"),
            ("line.post_trigger_dwell_ms", "Post Trigger Dwell ms")
        ]
        for i, spec in enumerate(specs):
            self.add_field(panel, i + 1, spec[0], spec[1])
        return panel

    def build_matrix_section(self):
        panel = self.section("Matrix")
        enabled = JCheckBox("Enabled")
        self.checks["matrix.enabled"] = enabled
        panel.add(enabled, self.gbc(0, 0, 0.0))
        specs = [
            ("matrix.offset_x_mm", "Offset X mm"),
            ("matrix.rows", "Rows"),
            ("matrix.columns", "Columns"),
            ("matrix.pitch_x_mm", "Pitch X mm"),
            ("matrix.pitch_y_mm", "Pitch Y mm"),
            ("matrix.lift_mm", "Lift mm"),
            ("matrix.post_trigger_dwell_ms", "Post Trigger Dwell ms")
        ]
        for i, spec in enumerate(specs):
            self.add_field(panel, i + 1, spec[0], spec[1])
        return panel

    def build_park_section(self):
        panel = self.section("Park Location")
        enabled = JCheckBox("Enabled")
        self.checks["park_location.enabled"] = enabled
        panel.add(enabled, self.gbc(0, 0, 0.0))
        specs = [
            ("park_location.x", "X mm"),
            ("park_location.y", "Y mm"),
            ("park_location.z", "Z mm"),
            ("park_location.rotation", "R deg")
        ]
        for i, spec in enumerate(specs):
            self.add_field(panel, i + 1, spec[0], spec[1])
        return panel

    def get_nested_value(self, cfg, dotted):
        data = cfg
        for part in dotted.split("."):
            if not isinstance(data, dict):
                return None
            data = data.get(part)
        return data

    def set_nested_value(self, cfg, dotted, value):
        parts = dotted.split(".")
        data = cfg
        for part in parts[:-1]:
            if part not in data or not isinstance(data[part], dict):
                data[part] = {}
            data = data[part]
        data[parts[-1]] = value

    def load_into_ui(self, cfg):
        for key in self.combos:
            self.combos[key].setSelectedItem(str(cfg.get(key, "")))
        for key in self.fields:
            value = self.get_nested_value(cfg, key)
            self.fields[key].setText("" if value is None else str(value))
        for key in self.checks:
            value = self.get_nested_value(cfg, key)
            self.checks[key].setSelected(bool(value))

    def parse_float(self, key):
        text = self.fields[key].getText().strip()
        if text == "":
            raise Exception("%s is required" % key)
        try:
            return float(text)
        except:
            raise Exception("%s must be a number" % key)

    def parse_int(self, key):
        text = self.fields[key].getText().strip()
        if text == "":
            raise Exception("%s is required" % key)
        try:
            return int(text)
        except:
            raise Exception("%s must be an integer" % key)

    def collect_config(self, validate):
        cfg = adhesive_calibration.deep_merge(adhesive_calibration.DEFAULT_CONFIG, self.cfg)
        cfg["nozzle_name"] = combo_text(self.combos["nozzle_name"])
        cfg["actuator_name"] = combo_text(self.combos["actuator_name"])

        int_keys = ["matrix.rows", "matrix.columns"]
        for key in self.fields:
            if key in int_keys:
                value = self.parse_int(key)
            else:
                value = self.parse_float(key)
            self.set_nested_value(cfg, key, value)
        for key in self.checks:
            self.set_nested_value(cfg, key, bool(self.checks[key].isSelected()))

        if validate:
            adhesive_calibration.validate_config(cfg)
            if str(cfg.get("nozzle_name", "")).strip() == "":
                raise Exception("Nozzle is required")
            if str(cfg.get("actuator_name", "")).strip() == "":
                raise Exception("Dispense actuator is required")
        return adhesive_calibration.public_clean_config(cfg)

    def refresh_selectors(self):
        try:
            set_combo_items(self.combos["nozzle_name"], available_nozzle_names(),
                combo_text(self.combos["nozzle_name"]))
            set_combo_items(self.combos["actuator_name"], available_actuator_names(),
                combo_text(self.combos["actuator_name"]))
            self.append_log("Refreshed OpenPnP object lists")
        except Exception, e:
            self.show_error("Refresh lists failed", e)

    def set_state(self, state):
        self.state = state
        self.status_area.setText(state)
        self.update_buttons()

    def update_buttons(self):
        for button in self.action_buttons:
            button.setEnabled(not self.worker_running)

    def append_log(self, msg):
        text = str(msg)
        gui_self = self

        class AppendTask(Runnable):
            def run(task_self):
                gui_self.log_area.append(text + "\n")
                gui_self.log_area.setCaretPosition(gui_self.log_area.getDocument().getLength())

        try:
            SwingUtilities.invokeLater(AppendTask())
        except:
            self.log_area.append(text + "\n")

    def show_error(self, title, err):
        self.append_log("%s: %s" % (title, err))
        JOptionPane.showMessageDialog(None, str(err), title, JOptionPane.ERROR_MESSAGE)

    def load_action(self):
        try:
            self.cfg = load_config_file()
            self.saved_snapshot = compact_json(adhesive_calibration.public_clean_config(self.cfg))
            self.load_into_ui(self.cfg)
            self.set_state(IDLE)
            self.append_log("Loaded %s" % CONFIG_PATH)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Load failed", e)

    def save_config_action(self):
        try:
            cfg = self.collect_config(True)
            path = save_config_file(cfg)
            self.cfg = adhesive_calibration.load_config(path)
            self.saved_snapshot = compact_json(adhesive_calibration.public_clean_config(self.cfg))
            self.load_into_ui(self.cfg)
            self.set_state(SAVED)
            self.append_log("Saved %s" % path)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Save failed", e)

    def set_origin_fields(self, loc):
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        self.fields["origin_location.x"].setText("%.4f" % loc.getX())
        self.fields["origin_location.y"].setText("%.4f" % loc.getY())
        self.fields["origin_location.z"].setText("%.4f" % loc.getZ())
        self.fields["origin_location.rotation"].setText("%.4f" % loc.getRotation())

    def capture_origin_action(self, source):
        try:
            cfg = self.collect_config(False)
            machine_obj = get_machine()
            if source == "camera":
                movable = find_top_camera(machine_obj)
            else:
                movable = adhesive_calibration.find_nozzle(cfg, None, machine_obj)
            loc = movable.getLocation().convertToUnits(LengthUnit.Millimeters)
            self.set_origin_fields(loc)
            self.append_log("Captured origin from %s DRO" % source)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Capture origin failed", e)

    def go_origin_action(self):
        try:
            cfg = self.collect_config(True)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Move setup failed", e)
            return
        self.run_worker("Go To Origin Safe Z", lambda: self.do_go_origin(cfg), self.go_done)

    def do_go_origin(self, cfg):
        machine_obj = require_machine_execute()
        gui_self = self

        class Task(Callable):
            def call(task_self):
                nozzle = adhesive_calibration.find_nozzle(cfg, None, machine_obj)
                target = adhesive_calibration.move_to_origin_safe_z(nozzle, cfg, gui_self.append_log)
                return {"ok": True, "target": adhesive_calibration.loc_to_dict(target)}

        return machine_obj.execute(Task())

    def go_done(self, result):
        self.set_state(READY)
        self.result_area.setText("Moved to origin safe Z")
        self.append_log("Move finished")

    def run_calibration_action(self):
        try:
            cfg = self.collect_config(True)
            save_config_file(cfg)
            self.cfg = cfg
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Run setup failed", e)
            return
        self.run_worker("Adhesive Calibration", lambda: self.do_run_calibration(cfg), self.run_done)

    def do_run_calibration(self, cfg):
        machine_obj = require_machine_execute()
        gui_self = self

        class Task(Callable):
            def call(task_self):
                backend = reload_backend()
                return backend.run_calibration(cfg, machine_obj, gui_self.append_log)

        return machine_obj.execute(Task())

    def run_done(self, result):
        if result is not None and result.get("ok", False):
            self.set_state(READY)
        else:
            self.set_state(ERROR)
        self.result_area.setText(self.format_result(result))
        self.append_log("Calibration finished")

    def format_result(self, result):
        if result is None:
            return "No result"
        lines = []
        lines.append("ok: %s" % result.get("ok"))
        lines.append("message: %s" % result.get("message", ""))
        lines.append("line dots: %s" % result.get("line_dots", 0))
        lines.append("matrix dots: %s" % result.get("matrix_dots", 0))
        lines.append("total dots: %s" % result.get("total_dots", 0))
        lines.append("nozzle: %s" % result.get("nozzle_name", ""))
        lines.append("actuator: %s" % result.get("actuator_name", ""))
        return "\n".join(lines)

    def run_worker(self, name, work, done):
        if self.worker_running:
            return
        self.worker_running = True
        self.set_state(RUNNING)
        self.append_log("Starting %s" % name)
        gui_self = self

        class Worker(SwingWorker):
            def doInBackground(worker_self):
                return work()

            def done(worker_self):
                gui_self.worker_running = False
                try:
                    done(worker_self.get())
                except Exception, e:
                    gui_self.set_state(ERROR)
                    gui_self.show_error("%s failed" % name, e)
                gui_self.update_buttons()

        Worker().execute()


def start():
    gui_obj = AdhesiveCalibrationGui()
    gui_obj.build()
    return gui_obj


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
