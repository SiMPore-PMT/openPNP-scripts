# lumen_calibration_gui.py

import imp
import json
import os
import sys

from java.awt import BorderLayout, Dimension, FlowLayout, GridBagConstraints, GridBagLayout, Insets, Image
from java.io import File
from java.lang import Runnable
from java.util.concurrent import Callable
from javax.imageio import ImageIO
from javax.swing import (
    BorderFactory, Box, BoxLayout, ImageIcon, JButton, JComboBox, JFrame, JLabel,
    JOptionPane, JPanel, JScrollPane, JSplitPane, JTextArea, JTextField,
    SwingUtilities, SwingWorker
)
from javax.swing.event import DocumentListener
from org.openpnp.model import Configuration, LengthUnit, Location
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils


CONFIG_PATH = "/home/engineering-simpore/.openpnp2/scripts/lumen_calibration_gui_config.json"
SCRIPT_DIR = "/home/engineering-simpore/.openpnp2/scripts"
HOMING_DIR = os.path.join(SCRIPT_DIR, "Events", "Homing")

if HOMING_DIR not in sys.path:
    sys.path.append(HOMING_DIR)

head_solver = imp.load_source(
    "lumen_head_offset_solver",
    os.path.join(HOMING_DIR, "lumen_head_offset_solver.py")
)
tip_metrology = imp.load_source(
    "lumen_tip_metrology",
    os.path.join(HOMING_DIR, "lumen_tip_metrology.py")
)


IDLE = "IDLE"
HEAD_OFFSET_MEASURED_ONLY = "HEAD_OFFSET_MEASURED_ONLY"
FULL_RUN_APPLIED_IN_MEMORY = "FULL_RUN_APPLIED_IN_MEMORY"
SAVED = "SAVED"
REVERTED_HEAD_OFFSET_ONLY_RECAL_REQUIRED = "REVERTED_HEAD_OFFSET_ONLY_RECAL_REQUIRED"
ERROR = "ERROR"


DEFAULT_CONFIG = {
    "nozzle_name": "",
    "part_id": "Calibration_Square",
    "result_stage": "results",
    "bottom_square_stage": "squareResults",
    "storage_location": {"x": 330.0, "y": 396.6, "z": 3.0, "rotation": 0.0},
    "calibration_location": {"x": 306.7, "y": 359.0, "z": 3.0, "rotation": 0.0},
    "part_height_mm": 2.0,
    "use_openpnp_part_height": True,
    "artifact_pick_angle_sign": -1.0,
    "artifact_pick_angle_offset_deg": 0.0,
    "focal_z_mm": None,
    "search_radius_mm": 5.0,
    "search_step_mm": 1.0,
    "angles_deg": [-150.0, -90.0, -30.0, 30.0, 90.0, 150.0],
    "bottom_square_zero_deg": 0.0,
    "head_offset_sign": 1.0,
    "square_angle_sign": 1.0,
    "settle_ms": 250,
    "square_settle_ms": 250,
    "square_sample_step_mm": 0.25,
    "enable_true_nozzle_rotation_zero": True,
    "rotation_axis_name": "a",
    "zeroed_square_verify_max_error_deg": 0.75,
    "zeroed_square_verify_samples": True,
    "vacuum_ms": 150,
    "blowoff_ms": 120,
    "max_delta_xy_mm": 1.5,
    "max_square_error_deg": 5.0,
    "expected_big_diameter_px": None,
    "expected_small_diameter_px": None,
    "diameter_tolerance_px": 0.0,
    "debug_image_path": ""
}


def get_machine():
    try:
        return machine
    except:
        return Configuration.get().getMachine()


def deep_copy(value):
    return json.loads(json.dumps(value, sort_keys=True))


def deep_merge(defaults, loaded):
    out = deep_copy(defaults)
    for key, value in loaded.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            nested = out[key]
            nested.update(value)
            out[key] = nested
        else:
            out[key] = value
    return out


def compact_json(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"))


def loc_text(loc):
    if loc is None:
        return ""
    return "X %.4f  Y %.4f  Z %.4f  R %.4f" % (
        float(loc.get("x", 0.0)),
        float(loc.get("y", 0.0)),
        float(loc.get("z", 0.0)),
        float(loc.get("rotation", loc.get("r", 0.0)))
    )


def xy_text(data):
    if data is None:
        return ""
    return "X %.4f  Y %.4f" % (float(data.get("x", 0.0)), float(data.get("y", 0.0)))


def maybe_float(value):
    text = str(value).strip()
    if text == "" or text.lower() == "none":
        return None
    return float(text)


def parse_bool(value):
    text = str(value).strip().lower()
    if text in ["true", "1", "yes", "y", "on"]:
        return True
    if text in ["false", "0", "no", "n", "off"]:
        return False
    raise Exception("Expected boolean true/false, got: %s" % value)


def load_config_file():
    if not os.path.exists(CONFIG_PATH):
        return deep_copy(DEFAULT_CONFIG)
    f = open(CONFIG_PATH, "r")
    try:
        loaded = json.loads(f.read())
    finally:
        f.close()
    return deep_merge(DEFAULT_CONFIG, loaded)


def save_config_file(cfg):
    clean = {}
    for key in DEFAULT_CONFIG.keys():
        clean[key] = cfg.get(key, DEFAULT_CONFIG[key])
    f = open(CONFIG_PATH, "w")
    try:
        f.write(json.dumps(clean, sort_keys=True, indent=2))
    finally:
        f.close()


def java_items(model):
    out = []
    if model is None:
        return out
    try:
        for item in model:
            out.append(item)
        return out
    except:
        pass
    try:
        for i in range(model.size()):
            out.append(model.get(i))
    except:
        pass
    return out


def find_nozzle(name):
    machine_obj = get_machine()
    head = machine_obj.getDefaultHead()
    if head is None:
        raise Exception("No default head")
    wanted = str(name or "").strip()
    if wanted:
        try:
            nozzle = head.getNozzleByName(wanted)
            if nozzle is not None:
                return nozzle
        except:
            pass
        try:
            for nozzle in java_items(head.getNozzles()):
                if nozzle.getName() == wanted:
                    return nozzle
        except:
            pass
        raise Exception("Nozzle not found: %s" % wanted)
    nozzle = head.getDefaultNozzle()
    if nozzle is None:
        raise Exception("No default nozzle")
    return nozzle


def get_available_parts():
    names = []
    try:
        for part in java_items(Configuration.get().getParts()):
            names.append(str(part.getId()))
    except:
        pass
    names.sort()
    return names


def get_available_nozzles():
    names = []
    try:
        head = get_machine().getDefaultHead()
        for nozzle in java_items(head.getNozzles()):
            names.append(str(nozzle.getName()))
    except:
        pass
    names.sort()
    return names


def is_down_camera(camera):
    if camera is None:
        return False
    try:
        return camera.getLooking() == SpiCamera.Looking.Down
    except:
        return False


def get_down_camera(machine_obj=None):
    if machine_obj is None:
        machine_obj = get_machine()
    try:
        head = machine_obj.getDefaultHead()
        camera = head.getDefaultCamera()
        if camera is not None:
            return camera
    except:
        pass
    try:
        for camera in java_items(machine_obj.getCameras()):
            if is_down_camera(camera):
                return camera
    except:
        pass
    raise Exception("No top/down camera available")


def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))


def wait_still(movable):
    movable.waitForCompletion(CompletionType.WaitForStillstand)


def move_safe(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)


def require_machine_execute():
    machine_obj = get_machine()
    if not hasattr(machine_obj, "execute"):
        raise Exception("OpenPnP machine task execution is not available")
    return machine_obj


class FieldChangeListener(DocumentListener):
    def __init__(self, callback):
        self.callback = callback

    def insertUpdate(self, event):
        self.callback()

    def removeUpdate(self, event):
        self.callback()

    def changedUpdate(self, event):
        self.callback()


class CalibrationGui(object):
    def __init__(self):
        self.frame = JFrame("Lumen Calibration")
        self.cfg = load_config_file()
        self.fields = {}
        self.loc_fields = {}
        self.result_area = JTextArea()
        self.pairwise_area = JTextArea()
        self.log_area = JTextArea()
        self.image_label = JLabel("No image")
        self.current_label = JTextArea()
        self.top_status_label = JTextArea()
        self.state_label = JLabel("")
        self.stale_label = JLabel(" ")
        self.part_combo = None
        self.nozzle_combo = None
        self.state = IDLE
        self.worker_running = False
        self.last_full_snapshot = None
        self.last_full_nozzle_name = None
        self.last_full_result = None
        self.backup_head_offsets = None
        self.unsaved_full_run_in_memory = False
        self.full_result_stale = False
        self.loading_ui = False
        self.action_buttons = []
        self.location_action_buttons = []

    def build(self):
        self.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.frame.setMinimumSize(Dimension(1000, 700))

        root = JPanel(BorderLayout(8, 8))
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8))

        left_scroll = JScrollPane(self.build_form())
        left_scroll.setPreferredSize(Dimension(455, 560))
        left_scroll.setMinimumSize(Dimension(430, 360))
        left_scroll.setHorizontalScrollBarPolicy(JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED)
        left_scroll.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED)
        split = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left_scroll, self.build_right())
        split.setResizeWeight(0.46)
        split.setDividerLocation(455)
        root.add(self.build_top_bar(), BorderLayout.NORTH)
        root.add(split, BorderLayout.CENTER)
        root.add(self.build_log_panel(), BorderLayout.SOUTH)

        self.frame.add(root, BorderLayout.CENTER)
        self.load_into_ui(self.cfg)
        self.refresh_current_values()
        self.set_state(IDLE)
        self.frame.pack()
        try:
            self.frame.setLocationRelativeTo(gui)
        except:
            self.frame.setLocationRelativeTo(None)
        self.frame.setVisible(True)

    def build_form(self):
        panel = JPanel()
        panel.setLayout(BoxLayout(panel, BoxLayout.Y_AXIS))

        cfg_panel = self.section("Config")
        self.fields["nozzle_name"] = JTextField(18)
        self.watch_field(self.fields["nozzle_name"])
        self.fields["part_id"] = JTextField(18)
        self.watch_field(self.fields["part_id"])
        cfg_panel.add(self.build_selector_panel(), self.full_width_gbc(0))
        specs = [
            ("result_stage", "Top Results Stage"),
            ("bottom_square_stage", "Bottom Square Raw Stage"),
            ("part_height_mm", "Legacy/Fallback Part Height mm"),
            ("use_openpnp_part_height", "OpenPnP Part Height Runtime Flag"),
            ("artifact_pick_angle_sign", "Artifact Pick Angle Sign"),
            ("artifact_pick_angle_offset_deg", "Artifact Pick Angle Offset deg"),
            ("focal_z_mm", "Focal Z mm"),
            ("search_radius_mm", "Search Radius mm"),
            ("search_step_mm", "Search Step mm"),
            ("angles_deg", "Pairwise Angles deg"),
            ("bottom_square_zero_deg", "Desired Nozzle Square Zero"),
            ("head_offset_sign", "Head Offset Sign"),
            ("square_angle_sign", "Square Angle Sign"),
            ("settle_ms", "Top Settle ms"),
            ("square_settle_ms", "Square Settle ms"),
            ("square_sample_step_mm", "Square Sample Step mm"),
            ("enable_true_nozzle_rotation_zero", "Enable True Nozzle Rotation Zero"),
            ("rotation_axis_name", "Rotation Axis Name"),
            ("zeroed_square_verify_max_error_deg", "Zeroed Square Verify Max Error deg"),
            ("zeroed_square_verify_samples", "Zeroed Square Verify Samples"),
            ("vacuum_ms", "Vacuum ms"),
            ("blowoff_ms", "Blowoff ms"),
            ("max_delta_xy_mm", "Max Delta XY mm"),
            ("max_square_error_deg", "Max Square Error deg"),
            ("expected_big_diameter_px", "Expected Big Diameter px"),
            ("expected_small_diameter_px", "Expected Small Diameter px"),
            ("diameter_tolerance_px", "Diameter Tolerance px"),
            ("debug_image_path", "Debug Image Path"),
        ]
        for i, spec in enumerate(specs):
            key, label = spec
            field = JTextField(24)
            self.fields[key] = field
            self.watch_field(field)
            self.add_row(cfg_panel, i + 1, label, field)
        note = JTextArea("Motion Z uses the selected OpenPnP Part height at runtime. Legacy part_height_mm is not used for motion Z when OpenPnP part height is available.")
        self.configure_status_area(note, 2)
        note.setPreferredSize(Dimension(680, 38))
        cfg_panel.add(note, self.full_width_gbc(len(specs) + 1))
        panel.add(cfg_panel)
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.location_section("Storage Pocket Base Location", "storage_location"))
        panel.add(Box.createVerticalStrut(6))
        panel.add(self.location_section("Open Calibration Area Base Location", "calibration_location"))
        panel.add(Box.createVerticalStrut(6))
        panel.add(Box.createVerticalGlue())
        panel.setPreferredSize(Dimension(760, panel.getPreferredSize().height))
        return panel

    def build_top_bar(self):
        wrapper = JPanel(BorderLayout(6, 4))
        wrapper.setBorder(BorderFactory.createEmptyBorder(0, 0, 4, 0))
        self.configure_status_area(self.top_status_label, 2)
        self.top_status_label.setPreferredSize(Dimension(900, 38))
        wrapper.add(self.top_status_label, BorderLayout.NORTH)
        wrapper.add(self.build_buttons(), BorderLayout.SOUTH)
        return wrapper

    def configure_status_area(self, area, rows):
        area.setEditable(False)
        area.setLineWrap(True)
        area.setWrapStyleWord(True)
        area.setRows(rows)
        area.setOpaque(False)
        area.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 0))

    def build_selector_panel(self):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder("Selectors"))
        self.part_combo = JComboBox()
        self.nozzle_combo = JComboBox()
        self.part_combo.setPreferredSize(Dimension(360, self.part_combo.getPreferredSize().height))
        self.nozzle_combo.setPreferredSize(Dimension(360, self.nozzle_combo.getPreferredSize().height))
        self.refresh_part_combo(None)
        self.refresh_nozzle_combo(None)
        self.part_combo.addActionListener(lambda event: self.on_part_combo_changed())
        self.nozzle_combo.addActionListener(lambda event: self.on_nozzle_combo_changed())

        part_row = JPanel(BorderLayout(4, 0))
        part_row.add(self.part_combo, BorderLayout.CENTER)
        refresh_parts = JButton("Refresh Parts")
        refresh_parts.addActionListener(lambda event: self.refresh_part_combo(self.fields["part_id"].getText().strip()))
        self.location_action_buttons.append(refresh_parts)
        part_row.add(refresh_parts, BorderLayout.EAST)
        self.add_row(panel, 0, "Part", part_row)
        self.add_row(panel, 1, "Part ID Fallback", self.fields["part_id"])

        nozzle_row = JPanel(BorderLayout(4, 0))
        nozzle_row.add(self.nozzle_combo, BorderLayout.CENTER)
        refresh_nozzles = JButton("Refresh Nozzles")
        refresh_nozzles.addActionListener(lambda event: self.refresh_nozzle_combo(self.fields["nozzle_name"].getText().strip()))
        self.location_action_buttons.append(refresh_nozzles)
        nozzle_row.add(refresh_nozzles, BorderLayout.EAST)
        self.add_row(panel, 2, "Nozzle", nozzle_row)
        self.add_row(panel, 3, "Nozzle Fallback", self.fields["nozzle_name"])
        return panel

    def full_width_gbc(self, row):
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.gridwidth = 2
        gbc.insets = Insets(2, 2, 5, 2)
        gbc.fill = GridBagConstraints.HORIZONTAL
        gbc.weightx = 1.0
        return gbc

    def refresh_part_combo(self, selected):
        if self.part_combo is None:
            return
        current = str(selected or "")
        if current == "" and "part_id" in self.fields:
            current = self.fields["part_id"].getText().strip()
        self.part_combo.removeAllItems()
        for part_id in get_available_parts():
            self.part_combo.addItem(part_id)
        if current:
            self.select_combo_value(self.part_combo, current)

    def refresh_nozzle_combo(self, selected):
        if self.nozzle_combo is None:
            return
        current = str(selected or "")
        if current == "" and "nozzle_name" in self.fields:
            current = self.fields["nozzle_name"].getText().strip()
        self.nozzle_combo.removeAllItems()
        self.nozzle_combo.addItem("")
        for nozzle_name in get_available_nozzles():
            self.nozzle_combo.addItem(nozzle_name)
        self.select_combo_value(self.nozzle_combo, current)

    def select_combo_value(self, combo, value):
        wanted = str(value or "")
        for i in range(combo.getItemCount()):
            if str(combo.getItemAt(i)) == wanted:
                combo.setSelectedIndex(i)
                return
        if wanted:
            combo.addItem(wanted)
            combo.setSelectedItem(wanted)

    def on_part_combo_changed(self):
        if self.loading_ui or self.part_combo is None:
            return
        item = self.part_combo.getSelectedItem()
        if item is not None:
            self.fields["part_id"].setText(str(item))
        self.refresh_current_values()
        self.on_field_changed()

    def on_nozzle_combo_changed(self):
        if self.loading_ui or self.nozzle_combo is None:
            return
        item = self.nozzle_combo.getSelectedItem()
        self.fields["nozzle_name"].setText(str(item or ""))
        self.refresh_current_values()
        self.on_field_changed()

    def refresh_selectors_and_current(self):
        self.refresh_part_combo(self.fields["part_id"].getText().strip())
        self.refresh_nozzle_combo(self.fields["nozzle_name"].getText().strip())
        self.refresh_current_values()

    def build_right(self):
        panel = JPanel(BorderLayout(6, 6))

        top = JPanel(GridBagLayout())
        top.setBorder(BorderFactory.createTitledBorder("Current Values"))
        self.configure_status_area(self.current_label, 3)
        self.current_label.setPreferredSize(Dimension(500, 58))
        self.add_row(top, 0, "Selected Nozzle / Head Offsets", self.current_label)
        self.add_row(top, 1, "State", self.state_label)
        self.add_row(top, 2, "Full Run Snapshot", self.stale_label)
        panel.add(top, BorderLayout.NORTH)

        center = JPanel(BorderLayout(6, 6))
        self.result_area.setEditable(False)
        self.pairwise_area.setEditable(False)
        self.result_area.setRows(12)
        self.pairwise_area.setRows(12)
        result_split = JSplitPane(
            JSplitPane.VERTICAL_SPLIT,
            JScrollPane(self.result_area),
            JScrollPane(self.pairwise_area)
        )
        result_split.setResizeWeight(0.45)
        center.add(result_split, BorderLayout.CENTER)

        image_panel = JPanel(BorderLayout())
        image_panel.setBorder(BorderFactory.createTitledBorder("Latest Debug Image"))
        self.image_label.setHorizontalAlignment(JLabel.CENTER)
        self.image_label.setPreferredSize(Dimension(420, 180))
        image_panel.add(self.image_label, BorderLayout.CENTER)
        center.add(image_panel, BorderLayout.SOUTH)
        panel.add(center, BorderLayout.CENTER)
        return panel

    def build_log_panel(self):
        panel = JPanel(BorderLayout())
        panel.setBorder(BorderFactory.createTitledBorder("Log"))
        self.log_area.setEditable(False)
        self.log_area.setRows(7)
        panel.add(JScrollPane(self.log_area), BorderLayout.CENTER)
        return panel

    def build_buttons(self):
        panel = JPanel(FlowLayout(FlowLayout.LEFT, 6, 2))
        self.load_btn = JButton("Load Config")
        self.save_btn = JButton("Save Config")
        self.measure_btn = JButton("Preview Head Offset Only")
        self.full_btn = JButton("START Full Calibration")
        self.apply_btn = JButton("Apply Accepted Values")
        self.revert_btn = JButton("Revert To Backed-Up Values")
        self.refresh_btn = JButton("Refresh Current Values")

        self.load_btn.addActionListener(lambda event: self.load_action())
        self.save_btn.addActionListener(lambda event: self.save_action())
        self.measure_btn.addActionListener(lambda event: self.measure_action())
        self.full_btn.addActionListener(lambda event: self.full_run_action())
        self.apply_btn.addActionListener(lambda event: self.apply_action())
        self.revert_btn.addActionListener(lambda event: self.revert_action())
        self.refresh_btn.addActionListener(lambda event: self.refresh_selectors_and_current())

        self.action_buttons = [
            self.load_btn, self.save_btn, self.measure_btn, self.full_btn,
            self.apply_btn, self.revert_btn, self.refresh_btn
        ]
        for button in self.action_buttons:
            panel.add(button)
        return panel

    def section(self, title):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder(title))
        panel.setAlignmentX(JPanel.LEFT_ALIGNMENT)
        panel.setMaximumSize(Dimension(760, 32767))
        panel.setMinimumSize(Dimension(720, panel.getMinimumSize().height))
        return panel

    def add_row(self, panel, row, label, component):
        label_obj = JLabel(label)
        label_obj.setPreferredSize(Dimension(210, label_obj.getPreferredSize().height))
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.insets = Insets(2, 2, 2, 8)
        gbc.anchor = GridBagConstraints.WEST
        gbc.weightx = 0.0
        panel.add(label_obj, gbc)

        gbc = GridBagConstraints()
        gbc.gridx = 1
        gbc.gridy = row
        gbc.insets = Insets(2, 2, 2, 2)
        gbc.anchor = GridBagConstraints.WEST
        gbc.fill = GridBagConstraints.HORIZONTAL
        gbc.weightx = 1.0
        panel.add(component, gbc)

    def location_section(self, title, key):
        panel = self.section(title)
        self.loc_fields[key] = {}
        note = JTextArea("Z is base pocket/area Z. Motion Z uses the selected OpenPnP Part height at runtime.")
        self.configure_status_area(note, 2)
        note.setPreferredSize(Dimension(680, 34))
        panel.add(note, self.full_width_gbc(0))
        labels = [("x", "X"), ("y", "Y"), ("z", "Z"), ("rotation", "R")]
        for i, spec in enumerate(labels):
            axis, label = spec
            field = JTextField(10)
            self.loc_fields[key][axis] = field
            self.watch_field(field)
            self.add_row(panel, i + 1, label, field)
        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 5, 0))
        capture_camera = JButton("Capture Camera DRO")
        capture_nozzle = JButton("Capture Nozzle DRO")
        go_camera = JButton("Go Camera")
        go_nozzle = JButton("Go Nozzle")
        capture_camera.addActionListener(lambda event, loc_key=key: self.capture_location(loc_key, "camera"))
        capture_nozzle.addActionListener(lambda event, loc_key=key: self.capture_location(loc_key, "nozzle"))
        go_camera.addActionListener(lambda event, loc_key=key: self.go_location(loc_key, "camera"))
        go_nozzle.addActionListener(lambda event, loc_key=key: self.go_location(loc_key, "nozzle"))
        self.location_action_buttons.extend([capture_camera, capture_nozzle, go_camera, go_nozzle])
        buttons.add(capture_camera)
        buttons.add(capture_nozzle)
        buttons.add(go_camera)
        buttons.add(go_nozzle)
        panel.add(buttons, self.full_width_gbc(len(labels) + 1))
        return panel

    def watch_field(self, field):
        field.getDocument().addDocumentListener(FieldChangeListener(self.on_field_changed))

    def on_field_changed(self):
        if self.loading_ui:
            return
        if self.state == FULL_RUN_APPLIED_IN_MEMORY and self.last_full_snapshot is not None:
            try:
                current = self.snapshot_config(self.collect_config())
                self.full_result_stale = (compact_json(current) != compact_json(self.last_full_snapshot))
            except:
                self.full_result_stale = True
            self.update_buttons()
            if self.full_result_stale:
                self.stale_label.setText("Stale: config changed since full run")
            else:
                self.stale_label.setText("Matches full-run snapshot")

    def set_state(self, state):
        self.state = state
        self.state_label.setText(state)
        self.refresh_current_values()
        self.update_buttons()

    def update_buttons(self):
        all_buttons = self.action_buttons + self.location_action_buttons
        if self.worker_running:
            for button in all_buttons:
                button.setEnabled(False)
            return

        has_backup = self.backup_head_offsets is not None
        for button in all_buttons:
            button.setEnabled(False)

        if self.state == IDLE:
            enabled = [self.load_btn, self.save_btn, self.refresh_btn, self.measure_btn, self.full_btn]
            if has_backup:
                enabled.append(self.revert_btn)
        elif self.state == HEAD_OFFSET_MEASURED_ONLY:
            enabled = [self.load_btn, self.save_btn, self.refresh_btn, self.measure_btn, self.full_btn]
            if has_backup:
                enabled.append(self.revert_btn)
        elif self.state == FULL_RUN_APPLIED_IN_MEMORY:
            enabled = [self.load_btn, self.save_btn, self.refresh_btn, self.revert_btn]
            if not self.full_result_stale:
                enabled.append(self.apply_btn)
        elif self.state == SAVED:
            enabled = [self.load_btn, self.save_btn, self.refresh_btn, self.measure_btn, self.full_btn]
            if has_backup:
                enabled.append(self.revert_btn)
        elif self.state == REVERTED_HEAD_OFFSET_ONLY_RECAL_REQUIRED:
            enabled = [self.load_btn, self.save_btn, self.refresh_btn, self.measure_btn, self.full_btn]
            if has_backup:
                enabled.append(self.revert_btn)
        else:
            enabled = [self.load_btn, self.save_btn, self.refresh_btn, self.measure_btn, self.full_btn]
            if has_backup:
                enabled.append(self.revert_btn)

        for button in enabled:
            button.setEnabled(True)
        for button in self.location_action_buttons:
            button.setEnabled(True)

    def append_log(self, msg):
        if not SwingUtilities.isEventDispatchThread():
            text = str(msg)

            class AppendTask(Runnable):
                def run(task_self):
                    self.append_log(text)

            SwingUtilities.invokeLater(AppendTask())
            return
        self.log_area.append(str(msg) + "\n")
        self.log_area.setCaretPosition(self.log_area.getDocument().getLength())

    def show_error(self, title, err):
        self.append_log("%s: %s" % (title, err))
        JOptionPane.showMessageDialog(self.frame, str(err), title, JOptionPane.ERROR_MESSAGE)

    def load_into_ui(self, cfg):
        self.loading_ui = True
        try:
            for key, field in self.fields.items():
                value = cfg.get(key, DEFAULT_CONFIG.get(key, ""))
                if isinstance(value, list):
                    field.setText(",".join([str(x) for x in value]))
                elif value is None:
                    field.setText("")
                else:
                    field.setText(str(value))
            for key, fields in self.loc_fields.items():
                loc = cfg.get(key, DEFAULT_CONFIG[key])
                for axis, field in fields.items():
                    field.setText(str(loc.get(axis, 0.0)))
            self.refresh_part_combo(cfg.get("part_id", ""))
            self.refresh_nozzle_combo(cfg.get("nozzle_name", ""))
        finally:
            self.loading_ui = False
        self.full_result_stale = False
        self.stale_label.setText(" ")
        self.update_image()

    def collect_config(self):
        cfg = deep_copy(DEFAULT_CONFIG)
        cfg["nozzle_name"] = self.selected_text_or_field(self.nozzle_combo, "nozzle_name", True)
        cfg["part_id"] = self.selected_text_or_field(self.part_combo, "part_id", False)
        if cfg["part_id"] == "":
            raise Exception("No artifact part selected")
        cfg["result_stage"] = self.fields["result_stage"].getText().strip()
        cfg["bottom_square_stage"] = self.fields["bottom_square_stage"].getText().strip()
        cfg["part_height_mm"] = float(self.fields["part_height_mm"].getText().strip())
        cfg["use_openpnp_part_height"] = parse_bool(
            self.fields["use_openpnp_part_height"].getText())
        cfg["artifact_pick_angle_sign"] = float(
            self.fields["artifact_pick_angle_sign"].getText().strip())
        cfg["artifact_pick_angle_offset_deg"] = float(
            self.fields["artifact_pick_angle_offset_deg"].getText().strip())
        cfg["focal_z_mm"] = maybe_float(self.fields["focal_z_mm"].getText())
        cfg["search_radius_mm"] = float(self.fields["search_radius_mm"].getText().strip())
        cfg["search_step_mm"] = float(self.fields["search_step_mm"].getText().strip())
        cfg["angles_deg"] = self.parse_angles(self.fields["angles_deg"].getText())
        cfg["bottom_square_zero_deg"] = float(self.fields["bottom_square_zero_deg"].getText().strip())
        cfg["head_offset_sign"] = float(self.fields["head_offset_sign"].getText().strip())
        cfg["square_angle_sign"] = float(self.fields["square_angle_sign"].getText().strip())
        cfg["settle_ms"] = int(float(self.fields["settle_ms"].getText().strip()))
        cfg["square_settle_ms"] = int(float(self.fields["square_settle_ms"].getText().strip()))
        cfg["square_sample_step_mm"] = float(self.fields["square_sample_step_mm"].getText().strip())
        cfg["enable_true_nozzle_rotation_zero"] = parse_bool(
            self.fields["enable_true_nozzle_rotation_zero"].getText())
        cfg["rotation_axis_name"] = self.fields["rotation_axis_name"].getText().strip()
        cfg["zeroed_square_verify_max_error_deg"] = float(
            self.fields["zeroed_square_verify_max_error_deg"].getText().strip())
        cfg["zeroed_square_verify_samples"] = parse_bool(
            self.fields["zeroed_square_verify_samples"].getText())
        cfg["vacuum_ms"] = int(float(self.fields["vacuum_ms"].getText().strip()))
        cfg["blowoff_ms"] = int(float(self.fields["blowoff_ms"].getText().strip()))
        cfg["max_delta_xy_mm"] = float(self.fields["max_delta_xy_mm"].getText().strip())
        cfg["max_square_error_deg"] = float(self.fields["max_square_error_deg"].getText().strip())
        cfg["expected_big_diameter_px"] = maybe_float(self.fields["expected_big_diameter_px"].getText())
        cfg["expected_small_diameter_px"] = maybe_float(self.fields["expected_small_diameter_px"].getText())
        cfg["diameter_tolerance_px"] = float(self.fields["diameter_tolerance_px"].getText().strip() or "0")
        cfg["debug_image_path"] = self.fields["debug_image_path"].getText().strip()
        cfg["storage_location"] = self.collect_location("storage_location")
        cfg["calibration_location"] = self.collect_location("calibration_location")
        return cfg

    def selected_text_or_field(self, combo, field_key, allow_empty_combo):
        field_value = self.fields[field_key].getText().strip()
        combo_value = ""
        if combo is not None:
            item = combo.getSelectedItem()
            if item is not None:
                combo_value = str(item).strip()
        if field_value != "":
            return field_value
        if combo_value != "":
            return combo_value
        if allow_empty_combo:
            return ""
        return ""

    def collect_location(self, key):
        fields = self.loc_fields[key]
        return {
            "x": float(fields["x"].getText().strip()),
            "y": float(fields["y"].getText().strip()),
            "z": float(fields["z"].getText().strip()),
            "rotation": float(fields["rotation"].getText().strip())
        }

    def parse_angles(self, text):
        out = []
        for item in str(text).replace(";", ",").split(","):
            item = item.strip()
            if item:
                out.append(float(item))
        if len(out) == 0:
            raise Exception("At least one pairwise angle is required")
        return out

    def snapshot_config(self, cfg):
        return deep_copy(cfg)

    def selected_nozzle_name(self, cfg=None):
        if cfg is None:
            cfg = self.collect_config()
        nozzle = find_nozzle(cfg.get("nozzle_name", ""))
        return nozzle.getName()

    def load_action(self):
        if self.unsaved_full_run_in_memory:
            self.show_error("Load blocked", "Apply or Revert the in-memory calibration before loading config.")
            return
        try:
            self.cfg = load_config_file()
            self.load_into_ui(self.cfg)
            self.refresh_current_values()
            self.set_state(IDLE)
            self.append_log("Loaded %s" % CONFIG_PATH)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Load failed", e)

    def save_action(self):
        if self.unsaved_full_run_in_memory:
            self.show_error("Save blocked", "Apply or Revert the in-memory calibration before saving config.")
            return
        try:
            cfg = self.collect_config()
            save_config_file(cfg)
            self.cfg = cfg
            self.append_log("Saved %s" % CONFIG_PATH)
            self.refresh_current_values()
            if self.state == ERROR:
                self.set_state(IDLE)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Save failed", e)

    def set_location_fields(self, key, loc):
        loc = loc.convertToUnits(LengthUnit.Millimeters)
        fields = self.loc_fields[key]
        fields["x"].setText("%.4f" % loc.getX())
        fields["y"].setText("%.4f" % loc.getY())
        fields["z"].setText("%.4f" % loc.getZ())
        fields["rotation"].setText("%.4f" % loc.getRotation())

    def capture_location(self, key, source):
        try:
            if source == "camera":
                movable = get_down_camera(get_machine())
            else:
                cfg = self.collect_config()
                movable = find_nozzle(cfg.get("nozzle_name", ""))
            loc = movable.getLocation().convertToUnits(LengthUnit.Millimeters)
            self.set_location_fields(key, loc)
            self.append_log("Captured %s from %s DRO" % (key, source))
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Capture failed", e)

    def go_location(self, key, target_type):
        try:
            loc = self.collect_location(key)
            nozzle_name = ""
            if target_type != "camera":
                cfg = self.collect_config()
                nozzle_name = self.selected_nozzle_name(cfg)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Move setup failed", e)
            return
        self.run_machine_worker(
            "Go %s" % target_type,
            lambda: self.do_go_location(loc, target_type, nozzle_name),
            self.go_location_done
        )

    def do_go_location(self, loc, target_type, nozzle_name):
        machine_obj = require_machine_execute()

        class Task(Callable):
            def call(task_self):
                if target_type == "camera":
                    movable = get_down_camera(machine_obj)
                else:
                    movable = find_nozzle(nozzle_name)
                move_safe(movable, mm_loc(loc["x"], loc["y"], loc["z"], loc["rotation"]))
                return {"ok": True, "target": target_type}

        return machine_obj.execute(Task())

    def go_location_done(self, result):
        self.refresh_current_values()
        self.append_log("Move finished: %s" % safe_get(result, "target"))

    def measure_action(self):
        try:
            cfg = self.collect_config()
            nozzle_name = self.selected_nozzle_name(cfg)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Measure setup failed", e)
            return

        self.run_machine_worker(
            "Preview Head Offset Only",
            lambda: self.do_measure(cfg, nozzle_name),
            self.measure_done
        )

    def do_measure(self, cfg, nozzle_name):
        machine_obj = require_machine_execute()
        nozzle = find_nozzle(nozzle_name)

        class Task(Callable):
            def call(task_self):
                return head_solver.solve_head_offset(
                    cfg,
                    nozzle=nozzle,
                    camera=head_solver.get_default_camera(machine_obj),
                    machine_obj=machine_obj,
                    logger=self.append_log
                )

        return machine_obj.execute(Task())

    def measure_done(self, result):
        self.update_result(result, None)
        if result is not None and result.get("ok", False):
            self.set_state(HEAD_OFFSET_MEASURED_ONLY)
            self.append_log("Preview Head Offset Only finished")
        else:
            self.set_state(ERROR)
            self.append_log("Preview Head Offset Only failed: %s" % safe_get(result, "message"))
        self.update_image()

    def full_run_action(self):
        try:
            cfg = self.collect_config()
            snapshot = self.snapshot_config(cfg)
            nozzle_name = self.selected_nozzle_name(cfg)
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Full calibration setup failed", e)
            return

        self.run_machine_worker(
            "START Full Calibration",
            lambda: self.do_full_run(cfg, snapshot, nozzle_name),
            self.full_run_done
        )

    def do_full_run(self, cfg, snapshot, nozzle_name):
        machine_obj = require_machine_execute()
        nozzle = find_nozzle(nozzle_name)

        class Task(Callable):
            def call(task_self):
                return tip_metrology.run_full_sequence(
                    cfg, nozzle=nozzle, machine_obj=machine_obj, logger=self.append_log)

        result = machine_obj.execute(Task())
        return {"result": result, "snapshot": snapshot, "nozzle_name": nozzle_name}

    def full_run_done(self, payload):
        result = None
        if payload is not None:
            result = payload.get("result")
        self.update_result(result, result)
        self.capture_full_run_backup(result)
        if result is not None and result.get("ok", False):
            self.last_full_snapshot = payload.get("snapshot")
            self.last_full_nozzle_name = payload.get("nozzle_name")
            self.last_full_result = result
            self.unsaved_full_run_in_memory = True
            self.full_result_stale = False
            self.stale_label.setText("Matches full-run snapshot")
            self.refresh_current_values()
            self.set_state(FULL_RUN_APPLIED_IN_MEMORY)
            self.append_log("Run Full Calibration finished; machine state is applied in memory")
        else:
            if result is not None and (result.get("head_offsets") or {}).get("applied_head_offsets_mm") is not None:
                self.unsaved_full_run_in_memory = True
            self.set_state(ERROR)
            self.append_log("Run Full Calibration failed: %s" % safe_get(result, "message"))
        self.update_image()

    def capture_full_run_backup(self, result):
        if result is None:
            return
        self.last_full_result = result
        backup = result.get("backup_head_offsets_mm")
        if backup is None:
            offsets = result.get("head_offsets") or {}
            backup = offsets.get("backup_head_offsets_mm")
        if backup is not None:
            self.backup_head_offsets = backup

    def apply_action(self):
        if self.state != FULL_RUN_APPLIED_IN_MEMORY:
            return
        try:
            current_cfg = self.snapshot_config(self.collect_config())
            current_nozzle = self.selected_nozzle_name(current_cfg)
            if current_nozzle != self.last_full_nozzle_name:
                raise Exception("Selected nozzle changed: current=%s full_run=%s" %
                                (current_nozzle, self.last_full_nozzle_name))
            if compact_json(current_cfg) != compact_json(self.last_full_snapshot):
                raise Exception("Config changed since full calibration; save blocked")
        except Exception, e:
            self.full_result_stale = True
            self.stale_label.setText("Stale: %s" % e)
            self.set_state(ERROR)
            self.show_error("Apply blocked", e)
            return

        self.run_simple_worker("Apply Accepted Values", self.do_apply_save, self.apply_done)

    def do_apply_save(self):
        Configuration.get().save()
        return {"ok": True}

    def apply_done(self, result):
        self.unsaved_full_run_in_memory = False
        self.refresh_current_values()
        self.set_state(SAVED)
        self.append_log("Applied accepted values by saving OpenPnP configuration")

    def revert_action(self):
        if self.backup_head_offsets is None:
            self.show_error("Revert blocked", "No backed-up head offsets are available.")
            return
        try:
            nozzle_name = self.last_full_nozzle_name or self.selected_nozzle_name()
        except Exception, e:
            self.set_state(ERROR)
            self.show_error("Revert setup failed", e)
            return

        self.run_machine_worker(
            "Revert To Backed-Up Values",
            lambda: self.do_revert(nozzle_name),
            self.revert_done
        )

    def do_revert(self, nozzle_name):
        machine_obj = require_machine_execute()
        nozzle = find_nozzle(nozzle_name)
        backup = self.backup_head_offsets

        class Task(Callable):
            def call(task_self):
                return tip_metrology.restore_head_offsets(nozzle, backup)

        return machine_obj.execute(Task())

    def revert_done(self, result):
        self.update_result(result, self.last_full_result)
        self.unsaved_full_run_in_memory = False
        self.full_result_stale = True
        self.stale_label.setText("Reverted; nozzle-tip recalibration required")
        self.refresh_current_values()
        self.set_state(REVERTED_HEAD_OFFSET_ONLY_RECAL_REQUIRED)
        self.append_log("Revert restored old head offsets in memory only. To persist the revert, run required nozzle-tip recalibration and then save from OpenPnP or rerun full calibration.")

    def run_machine_worker(self, name, work, done):
        if self.worker_running:
            return
        self.worker_running = True
        self.update_buttons()
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

    def run_simple_worker(self, name, work, done):
        if self.worker_running:
            return
        self.worker_running = True
        self.update_buttons()
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

    def refresh_current_values(self):
        try:
            cfg = self.collect_config()
            nozzle = find_nozzle(cfg.get("nozzle_name", ""))
            loc = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)
            text = "Nozzle: %s  Part: %s  Head Offsets: X %.4f  Y %.4f  Z %.4f  R %.4f  Config: %s  State: %s" % (
                nozzle.getName(), cfg.get("part_id", ""), loc.getX(), loc.getY(),
                loc.getZ(), loc.getRotation(), CONFIG_PATH, self.state)
            self.current_label.setText(text)
            self.top_status_label.setText(text)
        except Exception, e:
            self.current_label.setText("Unavailable: %s" % e)
            self.top_status_label.setText("Unavailable: %s  Config: %s  State: %s" %
                                          (e, CONFIG_PATH, self.state))

    def update_result(self, result, full_result):
        if result is None:
            self.result_area.setText("")
            self.pairwise_area.setText("")
            return
        solver = result
        if full_result is not None:
            solver = (full_result.get("head_offset_solver") or {})
        square = {}
        artifact_return = {}
        if full_result is not None:
            square = full_result.get("square") or {}
            artifact_return = full_result.get("artifact_return") or {}

        lines = []
        lines.append("ok: %s" % safe_get(result, "ok"))
        lines.append("message: %s" % safe_get(result, "message"))
        lines.append("old head offsets: %s" % loc_text(solver.get("old_head_offsets_mm")))
        lines.append("proposed head offsets: %s" % loc_text(solver.get("proposed_head_offsets_mm")))
        applied = (full_result.get("head_offsets") or {}).get("applied_head_offsets_mm") if full_result else None
        if applied is not None:
            lines.append("applied head offsets: %s" % loc_text(applied))
        lines.append("delta XY: %s" % xy_text(solver.get("delta_xy_mm")))
        lines.append("square raw angle deg: %s" % safe_get(square, "raw_angle_deg"))
        lines.append("square normalized error deg: %s" % safe_get(square, "normalized_error_deg"))
        lines.append("final artifact angle on return: %s" %
                     safe_get(artifact_return, "artifact_angle_before_return_deg"))
        self.result_area.setText("\n".join(lines))

        records = solver.get("per_angle_records") or []
        out = []
        for rec in records:
            delta = rec.get("delta_mm") or {}
            meas = rec.get("measurement") or {}
            out.append("angle %s place %s delta %s artifact_angle %s" % (
                rec.get("angle_deg"),
                rec.get("place_angle_deg"),
                xy_text(delta),
                meas.get("artifact_angle_deg")
            ))
        self.pairwise_area.setText("\n".join(out))

    def update_image(self):
        try:
            path = self.fields["debug_image_path"].getText().strip()
            if not path or not os.path.exists(path):
                self.image_label.setIcon(None)
                self.image_label.setText("No image")
                return
            img = ImageIO.read(File(path))
            if img is None:
                self.image_label.setIcon(None)
                self.image_label.setText("No image")
                return
            scaled = img.getScaledInstance(420, 180, Image.SCALE_SMOOTH)
            self.image_label.setText("")
            self.image_label.setIcon(ImageIcon(scaled))
        except:
            self.image_label.setIcon(None)
            self.image_label.setText("Image unavailable")


def safe_get(data, key):
    try:
        if data is None:
            return ""
        value = data.get(key)
        if value is None:
            return ""
        return value
    except:
        return ""


def start():
    gui_obj = CalibrationGui()
    gui_obj.build()
    return gui_obj


class StartRunnable(Runnable):
    def run(self):
        start()


try:
    SwingUtilities.invokeLater(StartRunnable())
except:
    start()
