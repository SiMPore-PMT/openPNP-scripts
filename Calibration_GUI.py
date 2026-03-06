# Jython 2.7 / OpenPnP Swing GUI for pick_cameraAlign_Calibration.py
#
# Layout map:
#   left:  scrollable configuration form
#   right: last captured vision image, run controls, compact result fields
#   bottom: progress log
#
# Usage:
#   Open OpenPnP Scripts, select pick_cameraAlign_Calibration_GUI.py,
#   and run it to edit config or start a dry-run calibration.
#   Homing can keep importing/calling pick_cameraAlign_Calibration.run(False/True)
#   directly; the runtime script remains the non-GUI integration point.

import imp
import json
import os
import sys
import time

from java.awt import (
    BasicStroke, BorderLayout, Color, Dimension, FlowLayout, Font,
    GridBagConstraints, GridBagLayout, Insets, Image, RenderingHints
)
from java.awt.image import BufferedImage
from java.io import File
from java.lang import Runnable
from java.util.concurrent import Callable
from javax.imageio import ImageIO
from javax.swing import (
    BorderFactory, Box, BoxLayout, ImageIcon, JButton, JCheckBox, JComboBox,
    JFrame, JLabel, JOptionPane, JPanel, JScrollPane, JSplitPane, JTextArea,
    JTextField, SwingUtilities, SwingWorker
)

from org.openpnp.model import Configuration, LengthUnit
from org.openpnp.spi import Camera as SpiCamera


def scripts_root():
    try:
        return scripting.getScriptsDirectory().toString()
    except:
        pass
    try:
        return os.path.dirname(os.path.abspath(__file__))
    except:
        return os.getcwd()


SCRIPT_DIR = scripts_root()
HOMING_DIR = os.path.join(SCRIPT_DIR, "Events", "Homing")
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)
if HOMING_DIR not in sys.path:
    sys.path.append(HOMING_DIR)

RUNTIME_PATH = os.path.join(HOMING_DIR, "pick_cameraAlign_Calibration.py")


def load_runtime():
    return imp.load_source("pick_cameraAlign_Calibration", RUNTIME_PATH)


runtime = load_runtime()


def get_machine():
    try:
        return machine
    except:
        return Configuration.get().getMachine()


def get_config():
    try:
        return config
    except:
        return Configuration.get()


def as_list(java_collection):
    out = []
    try:
        for item in java_collection:
            out.append(item)
        return out
    except:
        pass
    try:
        for i in range(java_collection.size()):
            out.append(java_collection.get(i))
    except:
        pass
    return out


def part_labels():
    labels = []
    try:
        for part in get_config().getParts():
            pid = part.getId()
            name = part.getName()
            if name and name != pid:
                labels.append("%s  (%s)" % (pid, name))
            else:
                labels.append(str(pid))
    except:
        pass
    return sorted(labels)


def part_id_from_label(label):
    if label is None:
        return ""
    text = str(label)
    if "  (" in text:
        return text.split("  (", 1)[0]
    return text


def nozzle_names():
    names = []
    try:
        head = get_machine().getDefaultHead()
        for nozzle in as_list(head.getNozzles()):
            names.append(nozzle.getName())
    except:
        pass
    return sorted(names)


def camera_names():
    names = []
    seen = {}
    try:
        head = get_machine().getDefaultHead()
        for cam in as_list(head.getCameras()):
            seen[cam.getName()] = True
    except:
        pass
    try:
        for cam in as_list(get_machine().getCameras()):
            seen[cam.getName()] = True
    except:
        pass
    for name in seen.keys():
        names.append(name)
    return sorted(names)


def select_combo_item(combo, value):
    wanted = str(value or "")
    for i in range(combo.getItemCount()):
        item = combo.getItemAt(i)
        if str(item) == wanted or part_id_from_label(item) == wanted:
            combo.setSelectedIndex(i)
            return
    if wanted:
        combo.addItem(wanted)
        combo.setSelectedItem(wanted)


def find_nozzle_by_name(name):
    head = get_machine().getDefaultHead()
    try:
        nozzle = head.getNozzleByName(name)
        if nozzle is not None:
            return nozzle
    except:
        pass
    return head.getDefaultNozzle()


def find_camera_by_name(name):
    head = get_machine().getDefaultHead()
    machine_obj = get_machine()
    try:
        for cam in as_list(head.getCameras()):
            if cam.getName() == name:
                return cam
    except:
        pass
    try:
        for cam in as_list(machine_obj.getCameras()):
            if cam.getName() == name:
                return cam
    except:
        pass
    try:
        return head.getDefaultCamera()
    except:
        return None


def panel_title(text):
    label = JLabel(text)
    label.setForeground(Color(45, 45, 45))
    return label


OVERLAY_NUMERIC_KEYS = ["square_width", "circle_width", "circle_radius", "cross_size", "key_scale"]
OVERLAY_COLOR_KEYS = ["actual_color", "expected_color", "circle_color", "orientation_color"]


class CancelToken(object):
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True

    def isCancelled(self):
        return self.cancelled


class CalibrationGui(object):
    def __init__(self):
        self.frame = JFrame("Pick Camera Align Calibration")
        self.cfg = runtime.load_config()
        self.fields = {}
        self.loc_fields = {}
        self.loc_rotations = {}
        self.result_fields = {}
        self.overlay_fields = {}
        self.safety_status_labels = {}
        self.error_label = JLabel(" ")
        self.pick_z_warning = JLabel(" ")
        self.camera_image = JLabel("No image")
        self.camera_status = JLabel("Camera idle")
        self.log_area = JTextArea()
        self.last_camera_image = None
        self.last_overlay = None
        self.last_result = None
        self.worker = None
        self.cancel_token = None
        self.is_running = False
        self.move_buttons = []
        self.busy_buttons = []

    def build(self):
        self.frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE)
        self.frame.setMinimumSize(Dimension(1120, 780))
        self.frame.getContentPane().setLayout(BorderLayout(10, 10))

        root = JPanel(BorderLayout(10, 10))
        root.setBorder(BorderFactory.createEmptyBorder(10, 10, 10, 10))

        left_scroll = JScrollPane(self.build_config_panel())
        right = self.build_right_panel()
        split = JSplitPane(JSplitPane.HORIZONTAL_SPLIT, left_scroll, right)
        split.setResizeWeight(0.48)
        split.setDividerLocation(560)

        root.add(split, BorderLayout.CENTER)
        root.add(self.build_log_panel(), BorderLayout.SOUTH)
        self.frame.add(root, BorderLayout.CENTER)

        self.load_into_ui(self.cfg)
        self.frame.pack()
        try:
            self.frame.setLocationRelativeTo(gui)
        except:
            self.frame.setLocationRelativeTo(None)
        self.frame.setVisible(True)
        self.refresh_camera(True)

    def build_config_panel(self):
        outer = JPanel()
        outer.setLayout(BoxLayout(outer, BoxLayout.Y_AXIS))
        outer.setBorder(BorderFactory.createEmptyBorder(2, 2, 2, 8))

        self.part_combo = JComboBox(part_labels())
        self.nozzle_combo = JComboBox(nozzle_names())
        self.camera_combo = JComboBox(camera_names())
        self.part_combo.addActionListener(lambda event: self.update_safety_ui())

        selectors = self.section_panel("Configuration")
        self.add_row(selectors, 0, "Part", self.part_combo)
        self.add_row(selectors, 1, "Nozzle", self.nozzle_combo)
        self.add_row(selectors, 2, "Top Camera", self.camera_combo)
        self.cap_section_width(selectors, 430)
        outer.add(selectors)
        outer.add(Box.createVerticalStrut(8))

        numeric = self.section_panel("Run Parameters")
        for idx, spec in enumerate([
            ("safe_travel_z", "Safe travel Z", "mm"),
            ("test_iterations", "Test iterations", "cycles"),
            ("verification_iterations", "Verification iterations", "cycles"),
            ("max_retries_per_step", "Max retries per step", ""),
            ("circle_detection_min_count", "Circle min count", ""),
        ]):
            key, label, unit = spec
            field = JTextField(10)
            self.fields[key] = field
            self.add_row(numeric, idx, label, self.with_unit(field, unit))
        self.cap_section_width(numeric, 430)
        outer.add(numeric)
        outer.add(Box.createVerticalStrut(8))

        safety = self.build_safety_panel()
        self.cap_section_width(safety, 430)
        outer.add(safety)
        outer.add(Box.createVerticalStrut(8))

        overlay = self.build_overlay_style_panel()
        self.cap_section_width(overlay, 430)
        outer.add(overlay)
        outer.add(Box.createVerticalStrut(8))

        spiral = self.section_panel("Spiral Search")
        self.spiral_enabled = JCheckBox("Enabled")
        self.add_row(spiral, 0, "Search", self.spiral_enabled)
        for idx, spec in enumerate([
            ("start_radius_mm", "Start radius", "mm"),
            ("radius_step_mm", "Radius step", "mm"),
            ("max_radius_mm", "Max radius", "mm"),
            ("angle_step_deg", "Angle step", "deg"),
            ("settle_ms", "Settle", "ms"),
        ]):
            key, label, unit = spec
            field = JTextField(10)
            self.fields["spiral_search.%s" % key] = field
            self.add_row(spiral, idx + 1, label, self.with_unit(field, unit))
        self.cap_section_width(spiral, 430)
        outer.add(spiral)
        outer.add(Box.createVerticalStrut(8))

        outer.add(self.location_section("Die Storage", "die_storage_location_xyz"))
        outer.add(Box.createVerticalStrut(8))
        outer.add(self.location_section("Calibration Work", "cal_work_location_xyz"))
        outer.add(Box.createVerticalStrut(8))

        buttons = JPanel(FlowLayout(FlowLayout.RIGHT, 8, 0))
        load_btn = JButton("Load Config")
        save_btn = JButton("Save Config")
        load_btn.addActionListener(lambda event: self.load_config_action())
        save_btn.addActionListener(lambda event: self.save_config_action())
        buttons.add(load_btn)
        buttons.add(save_btn)
        buttons.setAlignmentX(JPanel.CENTER_ALIGNMENT)
        outer.add(buttons)

        self.error_label.setForeground(Color(170, 30, 30))
        self.error_label.setAlignmentX(JPanel.CENTER_ALIGNMENT)
        outer.add(self.error_label)
        outer.add(Box.createVerticalGlue())
        return outer

    def cap_section_width(self, panel, width):
        size = panel.getPreferredSize()
        panel.setMaximumSize(Dimension(32767, int(size.height)))
        panel.setPreferredSize(Dimension(int(width), int(size.height)))
        panel.setAlignmentX(JPanel.CENTER_ALIGNMENT)

    def build_safety_panel(self):
        wrapper = JPanel(BorderLayout(4, 4))
        wrapper.setBorder(BorderFactory.createTitledBorder("Safety & Vision Lock"))

        self.safety_toggle = JCheckBox("Show safety settings", True)
        wrapper.add(self.safety_toggle, BorderLayout.NORTH)

        self.safety_body = JPanel(GridBagLayout())
        self.require_visual_lock = JCheckBox("Require visual lock before pick")
        self.verify_lock_after_centering = JCheckBox("Verify lock after centering")
        self.pick_surface_z_label = JLabel("0.0000 mm")

        self.add_row(self.safety_body, 0, "Visual lock", self.require_visual_lock)
        self.add_row(self.safety_body, 1, "Verify centered lock", self.verify_lock_after_centering)
        self.add_row(self.safety_body, 2, "Computed pick surface Z", self.pick_surface_z_label)
        self.pick_z_warning.setForeground(Color(190, 90, 0))
        self.add_row(self.safety_body, 3, "", self.pick_z_warning)

        for idx, spec in enumerate([
            ("max_pick_descent_mm", "Max pick descent", "mm"),
            ("z_clearance_before_pick_mm", "Z clearance before pick", "mm"),
            ("cal_camera_z_offset", "Cal camera Z offset", "mm"),
        ]):
            key, label, unit = spec
            field = JTextField(10)
            self.fields[key] = field
            row = idx + 4
            self.add_row(self.safety_body, row, label, self.with_unit(field, unit))

        helper = JLabel("Relative pick Z and visual lock are recommended for crash prevention.")
        helper.setForeground(Color(90, 90, 90))
        self.add_row(self.safety_body, 10, "", helper)
        self.safety_body.add(self.build_safety_status_panel(), self.full_width_gbc(11))

        wrapper.add(self.safety_body, BorderLayout.CENTER)

        self.safety_toggle.addActionListener(lambda event: self.toggle_safety_panel())
        self.require_visual_lock.addActionListener(lambda event: self.update_safety_ui())
        return wrapper

    def full_width_gbc(self, row):
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.gridwidth = 2
        gbc.insets = Insets(6, 2, 3, 2)
        gbc.weightx = 1.0
        gbc.fill = GridBagConstraints.HORIZONTAL
        return gbc

    def build_safety_status_panel(self):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder("Safety Status"))
        specs = [
            ("visual_lock", "Visual lock required?"),
            ("max_descent", "Max descent > 0?"),
            ("safe_travel", "Safe travel Z above locations?"),
        ]
        for idx, spec in enumerate(specs):
            key, label = spec
            status = JLabel("WARN")
            self.safety_status_labels[key] = status
            self.add_row(panel, idx, label, status)
        return panel

    def toggle_safety_panel(self):
        self.safety_body.setVisible(self.safety_toggle.isSelected())
        self.frame.pack()

    def section_panel(self, title):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createCompoundBorder(
            BorderFactory.createTitledBorder(title),
            BorderFactory.createEmptyBorder(6, 8, 8, 8)
        ))
        panel.setAlignmentX(JPanel.CENTER_ALIGNMENT)
        return panel

    def add_row(self, panel, row, label, component):
        gbc = GridBagConstraints()
        gbc.gridx = 0
        gbc.gridy = row
        gbc.insets = Insets(3, 2, 3, 8)
        gbc.anchor = GridBagConstraints.WEST
        panel.add(JLabel(label), gbc)

        gbc = GridBagConstraints()
        gbc.gridx = 1
        gbc.gridy = row
        gbc.insets = Insets(3, 2, 3, 2)
        gbc.anchor = GridBagConstraints.WEST
        gbc.weightx = 1.0
        gbc.fill = GridBagConstraints.HORIZONTAL
        panel.add(component, gbc)

    def with_unit(self, field, unit):
        panel = JPanel(BorderLayout(6, 0))
        panel.add(field, BorderLayout.CENTER)
        panel.add(JLabel(unit), BorderLayout.EAST)
        return panel

    def icon_button(self, codepoint, tooltip):
        try:
            text = unichr(codepoint)
        except:
            text = str(codepoint)
        button = JButton(text)
        button.setToolTipText(tooltip)
        button.setPreferredSize(Dimension(42, 28))
        return button

    def location_section(self, title, key):
        panel = self.section_panel(title)
        fields = {}
        self.loc_fields[key] = fields
        row_panel = JPanel(GridBagLayout())
        labels = [("x", "X"), ("y", "Y"), ("z", "Z")]
        for i, item in enumerate(labels):
            axis, label = item
            gbc = GridBagConstraints()
            gbc.gridx = i * 2
            gbc.gridy = 0
            gbc.insets = Insets(0, 0, 0, 4)
            row_panel.add(JLabel(label), gbc)
            field = JTextField(11)
            field.setMinimumSize(Dimension(130, field.getPreferredSize().height))
            field.setPreferredSize(Dimension(145, field.getPreferredSize().height))
            fields[axis] = field
            gbc = GridBagConstraints()
            gbc.gridx = i * 2 + 1
            gbc.gridy = 0
            gbc.insets = Insets(0, 0, 0, 8)
            gbc.anchor = GridBagConstraints.WEST
            gbc.weightx = 1.0
            gbc.fill = GridBagConstraints.HORIZONTAL
            row_panel.add(field, gbc)
        self.add_row(panel, 0, "Location", row_panel)
        capture = JButton("Capture Current DRO")
        capture.addActionListener(lambda event, loc_key=key: self.capture_location(loc_key))
        self.add_row(panel, 1, "", capture)
        move_panel = JPanel(FlowLayout(FlowLayout.LEFT, 6, 0))
        cam_btn = self.icon_button(0x25C9, "Move camera over %s at safe Z" % title)
        nozzle_btn = self.icon_button(0x2295, "Move nozzle over %s at safe Z" % title)
        cam_btn.addActionListener(lambda event, loc_key=key: self.run_move_tool(loc_key, "camera"))
        nozzle_btn.addActionListener(lambda event, loc_key=key: self.run_move_tool(loc_key, "nozzle"))
        move_panel.add(cam_btn)
        move_panel.add(nozzle_btn)
        self.move_buttons.append(cam_btn)
        self.move_buttons.append(nozzle_btn)
        self.add_row(panel, 2, "Move over", move_panel)
        self.cap_section_width(panel, 430)
        return panel

    def build_right_panel(self):
        right = JPanel()
        right.setLayout(BoxLayout(right, BoxLayout.Y_AXIS))

        cam_panel = JPanel(BorderLayout(6, 6))
        cam_panel.setBorder(BorderFactory.createTitledBorder("Last Captured Vision Image"))
        self.camera_image.setHorizontalAlignment(JLabel.CENTER)
        self.camera_image.setPreferredSize(Dimension(500, 280))
        cam_panel.add(JScrollPane(self.camera_image), BorderLayout.CENTER)
        cam_bar = JPanel(BorderLayout(6, 0))
        refresh_btn = JButton("Capture")
        refresh_btn.addActionListener(lambda event: self.refresh_camera(True))
        cam_bar.add(self.camera_status, BorderLayout.CENTER)
        cam_controls = JPanel(FlowLayout(FlowLayout.RIGHT, 6, 0))
        cam_controls.add(refresh_btn)
        cam_bar.add(cam_controls, BorderLayout.EAST)
        cam_panel.add(cam_bar, BorderLayout.SOUTH)
        right.add(cam_panel)
        right.add(Box.createVerticalStrut(8))

        exec_panel = JPanel(BorderLayout(8, 8))
        exec_panel.setBorder(BorderFactory.createTitledBorder("Calibration Execution"))
        buttons = JPanel(FlowLayout(FlowLayout.LEFT, 8, 0))
        self.run_btn = JButton("Run Calibration (Dry Run)")
        self.test_storage_btn = JButton("Test Storage Vision Lock")
        self.test_cal_btn = JButton("Test Cal Vision Lock")
        self.apply_btn = JButton("Apply Offsets to Machine")
        self.confirm_no_shift_btn = JButton("Show No Shift")
        self.confirm_shift_btn = JButton("Show Shift")
        self.abort_btn = JButton("Abort")
        self.apply_btn.setEnabled(False)
        self.confirm_no_shift_btn.setVisible(False)
        self.confirm_no_shift_btn.setEnabled(False)
        self.confirm_shift_btn.setVisible(False)
        self.confirm_shift_btn.setEnabled(False)
        self.abort_btn.setEnabled(False)
        self.run_btn.addActionListener(lambda event: self.run_dry())
        self.test_storage_btn.addActionListener(lambda event: self.run_vision_test("storage"))
        self.test_cal_btn.addActionListener(lambda event: self.run_vision_test("cal"))
        self.apply_btn.addActionListener(lambda event: self.apply_offsets())
        self.confirm_no_shift_btn.addActionListener(lambda event: self.run_confirm_shift(False))
        self.confirm_shift_btn.addActionListener(lambda event: self.run_confirm_shift(True))
        self.abort_btn.addActionListener(lambda event: self.abort_run())
        buttons.add(self.test_storage_btn)
        buttons.add(self.test_cal_btn)
        buttons.add(self.run_btn)
        buttons.add(self.apply_btn)
        buttons.add(self.abort_btn)
        self.busy_buttons = [self.run_btn, self.test_storage_btn, self.test_cal_btn,
                             self.apply_btn, self.confirm_no_shift_btn,
                             self.confirm_shift_btn] + self.move_buttons
        exec_panel.add(buttons, BorderLayout.NORTH)
        exec_panel.add(self.build_results_panel(), BorderLayout.CENTER)
        confirm_panel = JPanel(FlowLayout(FlowLayout.RIGHT, 8, 0))
        confirm_panel.setBorder(BorderFactory.createTitledBorder("Confirm"))
        confirm_panel.add(self.confirm_no_shift_btn)
        confirm_panel.add(self.confirm_shift_btn)
        exec_panel.add(confirm_panel, BorderLayout.SOUTH)
        right.add(exec_panel)
        right.add(Box.createVerticalGlue())
        return right

    def build_overlay_style_panel(self):
        panel = JPanel(GridBagLayout())
        panel.setBorder(BorderFactory.createTitledBorder("Overlay Style"))
        specs = [
            ("square_width", "Square line", "8"),
            ("circle_width", "Circle line", "6"),
            ("circle_radius", "Fallback circle radius", "14"),
            ("cross_size", "Center cross", "18"),
            ("key_scale", "Key size", "1.4"),
            ("actual_color", "Actual color", "#24B35F"),
            ("expected_color", "Expected color", "#FFB020"),
            ("circle_color", "Circle color", "#2A8CFF"),
            ("orientation_color", "Orientation color", "#F04C4C"),
        ]
        for idx, spec in enumerate(specs):
            key, label, default = spec
            field = JTextField(default, 8)
            field.addActionListener(lambda event: self.redraw_last_overlay())
            self.overlay_fields[key] = field
            self.add_row(panel, idx, label, field)
        redraw = JButton("Redraw Overlay")
        redraw.addActionListener(lambda event: self.redraw_last_overlay())
        self.add_row(panel, len(specs), "", redraw)
        return panel

    def build_results_panel(self):
        panel = JPanel(GridBagLayout())
        specs = [
            ("delta_x", "Delta X", "mm"),
            ("delta_y", "Delta Y", "mm"),
            ("delta_theta", "Delta theta", "deg"),
            ("confidence", "Repeatability RMS", "mm"),
            ("verify", "Verification", ""),
            ("circle_count", "Circle count", ""),
            ("center_x", "Center X error", "mm"),
            ("center_y", "Center Y error", "mm"),
            ("rotation", "Rotation", "deg"),
            ("search_dx", "Search dX", "mm"),
            ("search_dy", "Search dY", "mm"),
        ]
        for idx, spec in enumerate(specs):
            key, label, unit = spec
            field = JTextField(12)
            field.setEditable(False)
            self.result_fields[key] = field
            self.add_row(panel, idx, label, self.with_unit(field, unit))
        return panel

    def build_log_panel(self):
        panel = JPanel(BorderLayout(6, 6))
        panel.setBorder(BorderFactory.createTitledBorder("Progress Log"))
        self.log_area.setEditable(False)
        self.log_area.setRows(9)
        panel.add(JScrollPane(self.log_area), BorderLayout.CENTER)
        return panel

    def load_into_ui(self, cfg):
        select_combo_item(self.part_combo, cfg.get("part_id_or_name", ""))
        select_combo_item(self.nozzle_combo, cfg.get("nozzle_name", ""))
        select_combo_item(self.camera_combo, cfg.get("camera_name", ""))
        for key in ["safe_travel_z", "test_iterations",
                    "verification_iterations", "max_retries_per_step", "circle_detection_min_count"]:
            self.fields[key].setText(str(cfg.get(key, "")))
        self.require_visual_lock.setSelected(bool(cfg.get("require_visual_lock_before_pick", True)))
        self.verify_lock_after_centering.setSelected(bool(cfg.get("verify_lock_after_centering", True)))
        for key in ["max_pick_descent_mm", "z_clearance_before_pick_mm",
                    "cal_camera_z_offset"]:
            self.fields[key].setText(str(cfg.get(key, runtime.DEFAULT_CONFIG.get(key, ""))))
        spiral = cfg.get("spiral_search", {})
        self.spiral_enabled.setSelected(bool(spiral.get("enabled", True)))
        for key in ["start_radius_mm", "radius_step_mm", "max_radius_mm", "angle_step_deg", "settle_ms"]:
            self.fields["spiral_search.%s" % key].setText(str(spiral.get(key, "")))
        self.set_location_fields("die_storage_location_xyz", cfg.get("die_storage_location_xyz", {}))
        self.set_location_fields("cal_work_location_xyz", cfg.get("cal_work_location_xyz", {}))
        self.load_overlay_style(cfg.get("overlay_style", {}))
        self.error_label.setText(" ")
        self.update_safety_ui()

    def load_overlay_style(self, style):
        defaults = runtime.DEFAULT_CONFIG.get("overlay_style", {})
        merged = dict(defaults)
        try:
            merged.update(style)
        except:
            pass
        for key in OVERLAY_NUMERIC_KEYS + OVERLAY_COLOR_KEYS:
            field = self.overlay_fields.get(key)
            if field is not None:
                field.setText(str(merged.get(key, defaults.get(key, ""))))

    def set_location_fields(self, key, data):
        fields = self.loc_fields.get(key, {})
        for axis in ["x", "y", "z"]:
            fields[axis].setText(str(data.get(axis, 0.0)))
        self.loc_rotations[key] = float(data.get("rotation", 0.0))

    def location_from_fields(self, key):
        fields = self.loc_fields[key]
        return {
            "x": float(fields["x"].getText().strip()),
            "y": float(fields["y"].getText().strip()),
            "z": float(fields["z"].getText().strip()),
            "rotation": float(self.loc_rotations.get(key, 0.0))
        }

    def collect_config(self):
        cfg = runtime.deep_update(runtime.DEFAULT_CONFIG, self.cfg)
        cfg["part_id_or_name"] = part_id_from_label(self.part_combo.getSelectedItem())
        cfg["nozzle_name"] = str(self.nozzle_combo.getSelectedItem() or "")
        cfg["camera_name"] = str(self.camera_combo.getSelectedItem() or "")
        cfg["safe_travel_z"] = float(self.fields["safe_travel_z"].getText().strip())
        for deprecated_key in ["storage_base_z_mm", "part_height_z_mm"]:
            if deprecated_key in cfg:
                del cfg[deprecated_key]
        cfg["require_visual_lock_before_pick"] = self.require_visual_lock.isSelected()
        cfg["verify_lock_after_centering"] = self.verify_lock_after_centering.isSelected()
        cfg["max_pick_descent_mm"] = float(self.fields["max_pick_descent_mm"].getText().strip())
        cfg["z_clearance_before_pick_mm"] = float(self.fields["z_clearance_before_pick_mm"].getText().strip())
        cfg["cal_camera_z_offset"] = float(self.fields["cal_camera_z_offset"].getText().strip())
        for key in ["test_iterations", "verification_iterations", "max_retries_per_step", "circle_detection_min_count"]:
            cfg[key] = int(float(self.fields[key].getText().strip()))
        cfg["spiral_search"] = dict(cfg.get("spiral_search", {}))
        cfg["spiral_search"]["enabled"] = self.spiral_enabled.isSelected()
        for key in ["start_radius_mm", "radius_step_mm", "max_radius_mm", "angle_step_deg"]:
            cfg["spiral_search"][key] = float(self.fields["spiral_search.%s" % key].getText().strip())
        cfg["spiral_search"]["settle_ms"] = int(float(self.fields["spiral_search.settle_ms"].getText().strip()))
        cfg["die_storage_location_xyz"] = self.location_from_fields("die_storage_location_xyz")
        if "die_storage_end_location_xyz" in cfg:
            del cfg["die_storage_end_location_xyz"]
        cfg["cal_work_location_xyz"] = self.location_from_fields("cal_work_location_xyz")
        cfg["overlay_style"] = self.overlay_style_from_fields()
        self.apply_derived_runtime_fields(cfg)
        runtime.validate_config(cfg)
        self.validate_gui_config(cfg)
        self.update_safety_ui(cfg)
        return cfg

    def overlay_style_from_fields(self):
        style = dict(runtime.DEFAULT_CONFIG.get("overlay_style", {}))
        for key in OVERLAY_NUMERIC_KEYS:
            field = self.overlay_fields.get(key)
            if field is not None:
                style[key] = float(field.getText().strip())
        for key in OVERLAY_COLOR_KEYS:
            field = self.overlay_fields.get(key)
            if field is not None:
                text = field.getText().strip()
                if text and not text.startswith("#"):
                    text = "#" + text
                style[key] = text.upper()
        return style

    def selected_part_height_mm(self):
        part_id = part_id_from_label(self.part_combo.getSelectedItem())
        try:
            part = get_config().getPart(part_id)
        except:
            part = None
        if part is None:
            return 0.0
        return runtime.infer_part_height_mm(part)

    def apply_derived_runtime_fields(self, cfg):
        cfg["_storage_base_z_mm"] = float(cfg.get("die_storage_location_xyz", {}).get("z", 0.0))
        cfg["_part_height_z_mm"] = self.selected_part_height_mm()
        return cfg

    def validate_gui_config(self, cfg):
        errors = []
        if float(cfg["max_pick_descent_mm"]) <= 0.0:
            errors.append("Max pick descent must be greater than 0.")
        if float(cfg["z_clearance_before_pick_mm"]) < 0.0:
            errors.append("Z clearance before pick must be 0 or greater.")
        storage_surface_z = runtime.storage_pick_surface_z_mm(cfg)
        cal_surface_z = runtime.pick_surface_z_mm(runtime.loc_from_xyz(cfg["cal_work_location_xyz"]), cfg)
        if float(cfg["safe_travel_z"]) <= storage_surface_z:
            errors.append("Safe travel Z must be above computed storage pick surface Z.")
        if float(cfg["safe_travel_z"]) <= cal_surface_z:
            errors.append("Safe travel Z must be above computed calibration pick surface Z.")
        if float(cfg["safe_travel_z"]) <= float(cfg["die_storage_location_xyz"]["z"]):
            errors.append("Safe travel Z must be above Die Storage Z.")
        if float(cfg["safe_travel_z"]) <= float(cfg["cal_work_location_xyz"]["z"]):
            errors.append("Safe travel Z must be above Calibration Work Z.")
        if int(cfg["circle_detection_min_count"]) != 4:
            errors.append("Circle detection minimum count must be 4.")
        if int(cfg["test_iterations"]) < 1:
            errors.append("Test iterations must be at least 1.")
        if int(cfg["verification_iterations"]) < 1:
            errors.append("Verification iterations must be at least 1.")
        style = cfg.get("overlay_style", {})
        for key in OVERLAY_NUMERIC_KEYS:
            try:
                if float(style.get(key, 0.0)) <= 0.0:
                    errors.append("Overlay %s must be greater than 0." % key)
            except:
                errors.append("Overlay %s must be numeric." % key)
        for key in OVERLAY_COLOR_KEYS:
            try:
                text = str(style.get(key, "")).strip()
                if text.startswith("#"):
                    text = text[1:]
                int(text, 16)
                if len(text) != 6:
                    errors.append("Overlay %s must be a 6-digit hex color." % key)
            except:
                errors.append("Overlay %s must be a hex color." % key)
        if errors:
            raise Exception("\n".join(errors))

    def confirm_visual_lock_disabled(self, cfg):
        if bool(cfg.get("require_visual_lock_before_pick", True)):
            return True
        answer = JOptionPane.showConfirmDialog(
            self.frame,
            "Visual lock disabled can cause blind picks. Continue?",
            "Safety Warning",
            JOptionPane.YES_NO_OPTION
        )
        return answer == JOptionPane.YES_OPTION

    def set_status_label(self, key, ok, ok_text, warn_text):
        label = self.safety_status_labels.get(key)
        if label is None:
            return
        if ok:
            label.setText("PASS - " + ok_text)
            label.setForeground(Color(20, 130, 60))
        else:
            label.setText("WARN - " + warn_text)
            label.setForeground(Color(190, 90, 0))

    def update_safety_ui(self, cfg=None):
        self.pick_z_warning.setText("Pick surface = location Z + selected part height.")
        if cfg is None:
            try:
                cfg = self.collect_config_no_validate()
            except:
                cfg = None
        if cfg is None:
            return
        self.apply_derived_runtime_fields(cfg)
        pick_surface_z = runtime.storage_pick_surface_z_mm(cfg)
        cal_surface_z = runtime.pick_surface_z_mm(runtime.loc_from_xyz(cfg["cal_work_location_xyz"]), cfg)
        self.pick_surface_z_label.setText("%.4f mm (part %.4f)" %
                                          (pick_surface_z, runtime.part_height_z_mm(cfg)))
        storage_z = float(cfg["die_storage_location_xyz"]["z"])
        cal_z = float(cfg["cal_work_location_xyz"]["z"])
        safe_z = float(cfg["safe_travel_z"])
        self.set_status_label("visual_lock", bool(cfg.get("require_visual_lock_before_pick", True)),
                              "enabled", "disabled")
        self.set_status_label("max_descent", float(cfg.get("max_pick_descent_mm", 0.0)) > 0.0,
                              "> 0", "<= 0")
        self.set_status_label("safe_travel", safe_z > pick_surface_z and safe_z > cal_surface_z and safe_z > storage_z and safe_z > cal_z,
                              "above storage/cal pick surfaces", "not above storage/cal pick surfaces")

    def collect_config_no_validate(self):
        cfg = runtime.deep_update(runtime.DEFAULT_CONFIG, self.cfg)
        cfg["safe_travel_z"] = float(self.fields["safe_travel_z"].getText().strip())
        cfg["require_visual_lock_before_pick"] = self.require_visual_lock.isSelected()
        cfg["max_pick_descent_mm"] = float(self.fields["max_pick_descent_mm"].getText().strip())
        cfg["die_storage_location_xyz"] = self.location_from_fields("die_storage_location_xyz")
        cfg["cal_work_location_xyz"] = self.location_from_fields("cal_work_location_xyz")
        cfg["overlay_style"] = self.overlay_style_from_fields()
        self.apply_derived_runtime_fields(cfg)
        return cfg

    def load_config_action(self):
        try:
            self.cfg = runtime.load_config()
            self.load_into_ui(self.cfg)
            self.append_log("UI", "Loaded %s" % runtime.config_path())
        except Exception, e:
            self.show_error("Load failed", e)

    def save_config_action(self):
        try:
            cfg = self.collect_config()
            if not self.confirm_visual_lock_disabled(cfg):
                self.append_log("GUI][Safety", "Save cancelled because visual lock is disabled")
                return
            runtime.save_config(cfg)
            self.cfg = cfg
            self.error_label.setText("Saved %s" % runtime.config_path())
            self.append_log("UI", "Saved configuration")
        except Exception, e:
            self.error_label.setText(str(e))
            self.show_error("Save failed", e)

    def capture_location(self, key):
        try:
            source = "nozzle DRO"
            if key in ["die_storage_location_xyz", "cal_work_location_xyz"]:
                camera = find_camera_by_name(str(self.camera_combo.getSelectedItem() or ""))
                if camera is None:
                    raise Exception("No top camera selected.")
                loc = camera.getLocation().convertToUnits(LengthUnit.Millimeters)
                source = "camera DRO"
            else:
                nozzle = find_nozzle_by_name(str(self.nozzle_combo.getSelectedItem() or ""))
                loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
            fields = self.loc_fields[key]
            fields["x"].setText("%.4f" % loc.getX())
            fields["y"].setText("%.4f" % loc.getY())
            fields["z"].setText("%.4f" % loc.getZ())
            self.loc_rotations[key] = float(loc.getRotation())
            self.append_log("UI", "Captured %s from %s" % (key, source))
            self.update_safety_ui()
        except Exception, e:
            self.show_error("Capture failed", e)

    def refresh_camera(self, settle=True):
        try:
            cam = find_camera_by_name(str(self.camera_combo.getSelectedItem() or ""))
            if cam is None:
                raise Exception("No top camera selected.")
            image = None
            if bool(settle):
                try:
                    image = cam.settleAndCapture()
                except:
                    image = cam.capture()
            else:
                image = cam.capture()
            self.last_camera_image = image
            self.last_overlay = None
            self.camera_image.setText("")
            self.camera_image.setIcon(ImageIcon(self.scaled_camera_image(image)))
            self.camera_status.setText("%s captured %s" % (cam.getName(), time.strftime("%H:%M:%S")))
        except Exception, e:
            self.camera_status.setText("Camera refresh failed: %s" % e)

    def update_overlay_from_result(self, result):
        overlay = None
        try:
            overlay = result.get("overlay", None)
        except:
            overlay = None
        if overlay is None:
            return
        path = overlay.get("image_path", None)
        if path is None or not os.path.exists(path):
            return
        try:
            image = ImageIO.read(File(path))
            self.last_camera_image = image
            self.last_overlay = overlay
            painted = self.overlay_image(image, overlay)
            self.camera_image.setText("")
            self.camera_image.setIcon(ImageIcon(self.scaled_camera_image(painted)))
            self.camera_status.setText("Last captured vision frame %s" % time.strftime("%H:%M:%S"))
        except Exception, e:
            self.camera_status.setText("Overlay failed: %s" % e)

    def update_overlay_from_latest_file(self):
        try:
            path = runtime.last_overlay_path()
        except:
            path = os.path.join(HOMING_DIR, "pick_cameraAlign_Calibration_last_overlay.json")
        if not os.path.exists(path):
            return
        try:
            f = open(path, "r")
            try:
                overlay = json.loads(f.read())
            finally:
                f.close()
            self.update_overlay_from_result({"overlay": overlay})
        except Exception, e:
            self.camera_status.setText("Latest overlay load failed: %s" % e)

    def redraw_last_overlay(self):
        if self.last_overlay is not None:
            self.update_overlay_from_result({"overlay": self.last_overlay})

    def overlay_float(self, key, default_value):
        try:
            return float(self.overlay_fields[key].getText().strip())
        except:
            return float(default_value)

    def overlay_color(self, key, default_color):
        try:
            text = self.overlay_fields[key].getText().strip()
            if text.startswith("#"):
                text = text[1:]
            value = int(text, 16)
            return Color((value >> 16) & 255, (value >> 8) & 255, value & 255)
        except:
            return default_color

    def preview_scale_for(self, image):
        try:
            max_w = self.camera_image.getWidth()
            max_h = self.camera_image.getHeight()
            if max_w < 80:
                max_w = 500
            if max_h < 80:
                max_h = 280
            scale = min(float(max_w) / float(image.getWidth()), float(max_h) / float(image.getHeight()))
            if scale > 1.0:
                scale = 1.0
            return scale
        except:
            return 1.0

    def overlay_image(self, image, overlay):
        w = int(image.getWidth())
        h = int(image.getHeight())
        out = BufferedImage(w, h, BufferedImage.TYPE_INT_RGB)
        g = out.createGraphics()
        try:
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON)
            g.drawImage(image, 0, 0, None)
            actual = overlay.get("actual_corners_px", [])
            actual_center = overlay.get("actual_center_px", None)
            expected_center = overlay.get("expected_center_px", None)
            preview_scale = self.preview_scale_for(image)
            if preview_scale <= 0.0:
                preview_scale = 1.0
            square_width = self.overlay_float("square_width", 8.0) / preview_scale
            circle_width = self.overlay_float("circle_width", 6.0) / preview_scale
            circle_radius = self.overlay_float("circle_radius", 14.0) / preview_scale
            cross_size = self.overlay_float("cross_size", 18.0) / preview_scale
            key_scale = self.overlay_float("key_scale", 1.4) / preview_scale
            actual_color = self.overlay_color("actual_color", Color(36, 179, 95))
            expected_color = self.overlay_color("expected_color", Color(255, 176, 32))
            circle_color = self.overlay_color("circle_color", Color(42, 140, 255))
            orientation_color = self.overlay_color("orientation_color", Color(240, 76, 76))

            self.draw_square(g, actual, actual_color, square_width)
            self.draw_circles(g, overlay.get("detected_circles_px", actual), circle_color,
                              circle_width, circle_radius)
            orientation = overlay.get("orientation_circle_px", None)
            if orientation is not None:
                self.draw_circles(g, [orientation], orientation_color, circle_width, circle_radius)
            if actual_center is not None and expected_center is not None and len(actual) >= 4:
                expected = []
                dx = float(expected_center.get("x", 0.0)) - float(actual_center.get("x", 0.0))
                dy = float(expected_center.get("y", 0.0)) - float(actual_center.get("y", 0.0))
                for p in actual:
                    expected.append({"x": float(p.get("x", 0.0)) + dx, "y": float(p.get("y", 0.0)) + dy})
                self.draw_square(g, expected, expected_color, square_width)
                self.draw_cross(g, expected_center, expected_color, cross_size, circle_width)
                self.draw_cross(g, actual_center, actual_color, cross_size, circle_width)

            key_font = max(10, int(round(13.0 * key_scale)))
            key_pad = max(8, int(round(8.0 * key_scale)))
            key_w = max(190, int(round(246.0 * key_scale)))
            key_h = max(56, int(round(68.0 * key_scale)))
            g.setFont(Font("SansSerif", Font.BOLD, key_font))
            g.setColor(Color(0, 0, 0, 150))
            g.fillRect(key_pad, key_pad, key_w, key_h)
            g.setColor(expected_color)
            g.drawString("Expected square", key_pad + int(10 * key_scale), key_pad + int(20 * key_scale))
            g.setColor(actual_color)
            g.drawString("Actual square / circles", key_pad + int(10 * key_scale), key_pad + int(40 * key_scale))
            g.setColor(orientation_color)
            g.drawString("Orientation circle", key_pad + int(10 * key_scale), key_pad + int(60 * key_scale))
        finally:
            g.dispose()
        return out

    def draw_square(self, g, points, color, width):
        if points is None or len(points) < 4:
            return
        g.setColor(color)
        g.setStroke(BasicStroke(max(1.0, float(width))))
        for i in range(4):
            a = points[i]
            b = points[(i + 1) % 4]
            g.drawLine(int(round(float(a.get("x", 0.0)))), int(round(float(a.get("y", 0.0)))),
                       int(round(float(b.get("x", 0.0)))), int(round(float(b.get("y", 0.0)))))

    def draw_circles(self, g, points, color, width, radius_override):
        if points is None:
            return
        g.setColor(color)
        g.setStroke(BasicStroke(max(1.0, float(width))))
        for p in points:
            x = float(p.get("x", 0.0))
            y = float(p.get("y", 0.0))
            try:
                r = float(p.get("radius", 0.0))
            except:
                r = 0.0
            if r <= 0.0:
                r = float(radius_override)
            if r < 4.0:
                r = 9.0
            g.drawOval(int(round(x - r)), int(round(y - r)), int(round(2.0 * r)), int(round(2.0 * r)))

    def draw_cross(self, g, point, color, size, width):
        if point is None:
            return
        x = int(round(float(point.get("x", 0.0))))
        y = int(round(float(point.get("y", 0.0))))
        s = int(round(float(size)))
        g.setColor(color)
        g.setStroke(BasicStroke(max(1.0, float(width))))
        g.drawLine(x - s, y, x + s, y)
        g.drawLine(x, y - s, x, y + s)

    def scaled_camera_image(self, image):
        try:
            max_w = self.camera_image.getWidth()
            max_h = self.camera_image.getHeight()
            if max_w < 80:
                max_w = 500
            if max_h < 80:
                max_h = 280
            iw = float(image.getWidth())
            ih = float(image.getHeight())
            scale = min(float(max_w) / iw, float(max_h) / ih)
            if scale > 1.0:
                scale = 1.0
            w = max(1, int(iw * scale))
            h = max(1, int(ih * scale))
            return image.getScaledInstance(w, h, Image.SCALE_SMOOTH)
        except:
            return image

    def progress_callback(self, tag, msg, line):
        class AppendLater(Runnable):
            def run(inner_self):
                self.append_log(tag, msg)
                if tag in ["Vision", "Spiral", "Compute", "Verify", "VisionLock"]:
                    self.camera_status.setText(msg)
                if tag == "VisionLock" and "acquired" in str(msg).lower():
                    self.update_overlay_from_latest_file()
        SwingUtilities.invokeLater(AppendLater())

    def append_log(self, tag, msg):
        self.log_area.append("[%s] %s\n" % (tag, msg))
        self.log_area.setCaretPosition(self.log_area.getDocument().getLength())

    def set_running(self, running):
        self.is_running = bool(running)
        for button in self.busy_buttons:
            button.setEnabled(not running)
        self.abort_btn.setEnabled(running)
        has_result = self.last_result is not None and not self.last_result.get("error")
        self.apply_btn.setEnabled((not running) and has_result)
        self.confirm_no_shift_btn.setVisible(has_result)
        self.confirm_no_shift_btn.setEnabled((not running) and has_result)
        self.confirm_shift_btn.setVisible(has_result)
        self.confirm_shift_btn.setEnabled((not running) and has_result)

    def run_dry(self):
        try:
            cfg = self.collect_config()
            if not self.confirm_visual_lock_disabled(cfg):
                self.append_log("GUI][Safety", "Run cancelled because visual lock is disabled")
                return
            runtime.save_config(cfg)
            self.cfg = cfg
        except Exception, e:
            self.error_label.setText(str(e))
            self.show_error("Validation failed", e)
            return

        self.log_area.setText("")
        self.append_log("GUI][Run", "Starting dry-run calibration")
        self.last_result = None
        self.update_results(None)
        self.cancel_token = CancelToken()
        self.set_running(True)

        gui_self = self

        class DryRunWorker(SwingWorker):
            def doInBackground(worker_self):
                rt = load_runtime()
                class MachineTask(Callable):
                    def call(task_self):
                        return rt.run(apply_offsets=False, progress_callback=gui_self.progress_callback,
                                      cancel_token=gui_self.cancel_token)
                return get_machine().execute(MachineTask())

            def done(worker_self):
                try:
                    result = worker_self.get()
                    gui_self.last_result = result
                    gui_self.update_results(result)
                    gui_self.update_overlay_from_result(result)
                    gui_self.append_log("Done", "Dry run finished")
                except Exception, e:
                    gui_self.last_result = None
                    gui_self.update_results({"error": str(e)})
                    if gui_self.cancel_token is not None and gui_self.cancel_token.isCancelled():
                        gui_self.append_log("Abort", "Calibration aborted")
                    else:
                        gui_self.append_log("Error", str(e))
                        gui_self.show_error("Calibration failed", e)
                gui_self.set_running(False)

        self.worker = DryRunWorker()
        self.worker.execute()

    def run_vision_test(self, location_key):
        try:
            cfg = self.collect_config()
            runtime.save_config(cfg)
            self.cfg = cfg
        except Exception, e:
            self.error_label.setText(str(e))
            self.show_error("Validation failed", e)
            return

        self.append_log("GUI][VisionTest", "Starting %s vision lock test" % location_key)
        self.cancel_token = CancelToken()
        self.set_running(True)
        gui_self = self

        class VisionTestWorker(SwingWorker):
            def doInBackground(worker_self):
                rt = load_runtime()
                class MachineTask(Callable):
                    def call(task_self):
                        return rt.test_vision_lock(location_key, progress_callback=gui_self.progress_callback,
                                                   cancel_token=gui_self.cancel_token)
                return get_machine().execute(MachineTask())

            def done(worker_self):
                try:
                    result = worker_self.get()
                    gui_self.update_results(result)
                    gui_self.update_overlay_from_result(result)
                    gui_self.append_log("GUI][VisionTest", "%s vision lock test finished" % location_key)
                except Exception, e:
                    gui_self.update_results({"error": str(e)})
                    if gui_self.cancel_token is not None and gui_self.cancel_token.isCancelled():
                        gui_self.append_log("Abort", "Vision lock test aborted")
                    else:
                        gui_self.append_log("Error", str(e))
                        gui_self.show_error("Vision lock test failed", e)
                gui_self.set_running(False)

        self.worker = VisionTestWorker()
        self.worker.execute()

    def run_move_tool(self, location_key, tool_kind):
        try:
            cfg = self.collect_config()
            runtime.save_config(cfg)
            self.cfg = cfg
        except Exception, e:
            self.error_label.setText(str(e))
            self.show_error("Validation failed", e)
            return

        self.append_log("GUI][Move", "Moving %s over %s" % (tool_kind, location_key))
        self.cancel_token = CancelToken()
        self.set_running(True)
        gui_self = self

        class MoveToolWorker(SwingWorker):
            def doInBackground(worker_self):
                rt = load_runtime()
                class MachineTask(Callable):
                    def call(task_self):
                        return rt.move_tool_over_location(location_key, tool_kind,
                                                          progress_callback=gui_self.progress_callback,
                                                          cancel_token=gui_self.cancel_token)
                return get_machine().execute(MachineTask())

            def done(worker_self):
                try:
                    result = worker_self.get()
                    gui_self.append_log("GUI][Move", "%s over %s at X=%.4f Y=%.4f Z=%.4f" %
                                        (tool_kind, location_key, float(result.get("x_mm", 0.0)),
                                         float(result.get("y_mm", 0.0)), float(result.get("z_mm", 0.0))))
                except Exception, e:
                    if gui_self.cancel_token is not None and gui_self.cancel_token.isCancelled():
                        gui_self.append_log("Abort", "Move aborted")
                    else:
                        gui_self.append_log("Error", str(e))
                        gui_self.show_error("Move failed", e)
                gui_self.set_running(False)

        self.worker = MoveToolWorker()
        self.worker.execute()

    def run_confirm_shift(self, show_shift):
        if self.last_result is None or self.last_result.get("error"):
            return
        try:
            cfg = self.collect_config()
            runtime.save_config(cfg)
            self.cfg = cfg
        except Exception, e:
            self.error_label.setText(str(e))
            self.show_error("Validation failed", e)
            return

        mode_text = "show shift" if bool(show_shift) else "show no shift"
        self.append_log("GUI][Confirm", "Starting %s confirm test" % mode_text)
        self.cancel_token = CancelToken()
        self.set_running(True)
        gui_self = self

        class ConfirmShiftWorker(SwingWorker):
            def doInBackground(worker_self):
                rt = load_runtime()
                class MachineTask(Callable):
                    def call(task_self):
                        return rt.confirm_shift(gui_self.last_result, bool(show_shift),
                                                progress_callback=gui_self.progress_callback,
                                                cancel_token=gui_self.cancel_token)
                return get_machine().execute(MachineTask())

            def done(worker_self):
                try:
                    result = worker_self.get()
                    gui_self.update_results(result)
                    gui_self.update_overlay_from_result(result)
                    gui_self.append_log("GUI][Confirm", "Confirm %s target X=%.4f Y=%.4f R=%.4f" %
                                        (str(result.get("mode", mode_text)),
                                         float(result.get("target_x_mm", 0.0)),
                                         float(result.get("target_y_mm", 0.0)),
                                         float(result.get("target_rotation_deg", 0.0))))
                except Exception, e:
                    if gui_self.cancel_token is not None and gui_self.cancel_token.isCancelled():
                        gui_self.append_log("Abort", "Confirm shift aborted")
                    else:
                        gui_self.append_log("Error", str(e))
                        gui_self.show_error("Confirm shift failed", e)
                gui_self.set_running(False)

        self.worker = ConfirmShiftWorker()
        self.worker.execute()

    def apply_offsets(self):
        if self.last_result is None:
            return
        answer = JOptionPane.showConfirmDialog(
            self.frame,
            "Apply the computed offsets to the machine configuration?",
            "Confirm Apply",
            JOptionPane.YES_NO_OPTION
        )
        if answer != JOptionPane.YES_OPTION:
            return
        try:
            self.set_running(True)
            self.cancel_token = CancelToken()
            result = runtime.apply_result(self.last_result, progress_callback=self.progress_callback,
                                          cancel_token=self.cancel_token)
            self.last_result = result
            self.update_results(result)
            self.append_log("Apply", "Offsets applied")
            JOptionPane.showMessageDialog(self.frame, "Offsets applied.", "Apply complete",
                                          JOptionPane.INFORMATION_MESSAGE)
        except Exception, e:
            self.show_error("Apply failed", e)
        self.set_running(False)

    def abort_run(self):
        try:
            if self.cancel_token is not None:
                self.cancel_token.cancel()
            if self.worker is not None:
                self.worker.cancel(True)
            self.append_log("Abort", "Abort requested")
        except Exception, e:
            self.append_log("Abort", "Abort request failed: %s" % e)

    def update_results(self, result):
        for key in self.result_fields.keys():
            self.result_fields[key].setText("")
        if result is None:
            if self.last_result is None:
                self.confirm_no_shift_btn.setVisible(False)
                self.confirm_no_shift_btn.setEnabled(False)
                self.confirm_shift_btn.setVisible(False)
                self.confirm_shift_btn.setEnabled(False)
            return
        if result.get("error"):
            self.result_fields["verify"].setText("ERROR")
            return
        if result.get("type") == "vision_lock":
            self.result_fields["circle_count"].setText(str(result.get("circle_count", "")))
            self.result_fields["center_x"].setText("%.5f" % float(result.get("center_x_mm", 0.0)))
            self.result_fields["center_y"].setText("%.5f" % float(result.get("center_y_mm", 0.0)))
            self.result_fields["rotation"].setText("%.5f" % float(result.get("rotation_deg", 0.0)))
            self.result_fields["search_dx"].setText("%.5f" % float(result.get("search_dx_mm", 0.0)))
            self.result_fields["search_dy"].setText("%.5f" % float(result.get("search_dy_mm", 0.0)))
            self.result_fields["verify"].setText("VISION LOCK")
            return
        if result.get("type") == "confirm_shift":
            self.result_fields["circle_count"].setText(str(result.get("circle_count", "")))
            self.result_fields["center_x"].setText("%.5f" % float(result.get("target_x_mm", 0.0)))
            self.result_fields["center_y"].setText("%.5f" % float(result.get("target_y_mm", 0.0)))
            self.result_fields["rotation"].setText("%.5f" % float(result.get("target_rotation_deg", 0.0)))
            self.result_fields["search_dx"].setText("%.5f" % float(result.get("correction_x_mm", 0.0)))
            self.result_fields["search_dy"].setText("%.5f" % float(result.get("correction_y_mm", 0.0)))
            self.result_fields["verify"].setText(str(result.get("mode", "CONFIRM")).upper())
            return
        computed = result.get("computed", {})
        verification = result.get("verification", {})
        self.result_fields["delta_x"].setText("%.5f" % float(computed.get("offset_x_mm", 0.0)))
        self.result_fields["delta_y"].setText("%.5f" % float(computed.get("offset_y_mm", 0.0)))
        self.result_fields["delta_theta"].setText("%.5f" % float(computed.get("rotation_error_deg", 0.0)))
        self.result_fields["confidence"].setText("%.5f" % float(computed.get("repeatability_rms_mm", 0.0)))
        max_apply = float(self.cfg.get("max_apply_mm", runtime.DEFAULT_CONFIG.get("max_apply_mm", 0.5)))
        mag = float(verification.get("offset_mag_mm", computed.get("offset_mag_mm", 0.0)))
        verdict = "PASS" if mag <= max_apply else "CHECK"
        self.result_fields["verify"].setText("%s  mag=%.5f mm" % (verdict, mag))

    def show_error(self, title, error):
        JOptionPane.showMessageDialog(self.frame, str(error), title, JOptionPane.ERROR_MESSAGE)


def create_ui():
    CalibrationGui().build()


SwingUtilities.invokeLater(create_ui)