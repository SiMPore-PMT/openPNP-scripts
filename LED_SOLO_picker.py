# LED_SOLO_picker.py
# OpenPnP / Jython Swing LED segment picker for Lumen PnP
#
# Lets you toggle/color 8 individual LED positions using commands like:
#   M150 I5 P255 R255 U0 B0
#
# LED indexes are zero-based: I0 through I7.
#
# UI:
# - Graphical ring with 8 clickable square LED segments
# - JColorChooser for assigning the active color
# - Vertical intensity slider (P value 0-255)
# - Clicking a square toggles it on/off
# - "Set Selected Color" assigns the current chooser color to the selected/on LEDs
# - "All Off" turns all segments off
# - "Set" keeps the current LED state and exits
# - Close/X reverts all LEDs off and exits

from javax.swing import (
    JFrame, JPanel, JTextField, JLabel, JOptionPane,
    BoxLayout, JButton, JColorChooser, JSlider, JCheckBox
)
from javax.swing import SwingUtilities
from javax.swing.event import ChangeListener
from java.awt import Color, Dimension, BorderLayout, Graphics, Graphics2D, BasicStroke
from java.awt.event import WindowAdapter, MouseAdapter
from org.openpnp.model import Configuration

# In OpenPnP's scripting environment, 'machine' and 'gui' are usually injected.
# If not, fall back to Configuration.
try:
    machine
except NameError:
    machine = Configuration.get().getMachine()

try:
    gui
except NameError:
    gui = None

# --- Config: change actuator name here if needed ---
ACTUATOR_NAME = "LED Color Driver"   # must match your actuator name in Machine Setup
LED_COUNT = 8
DEFAULT_INTENSITY = 255

# Off color command. Intensity stays at the slider value so turning one segment off
# does not unexpectedly change the brightness of other enabled segments.
OFF_R = 0
OFF_G = 0
OFF_B = 0

led = machine.getActuatorByName(ACTUATOR_NAME)

if led is None:
    JOptionPane.showMessageDialog(
        None,
        "Actuator '%s' not found.\nCheck the name in Machine Setup \u2192 Actuators." % ACTUATOR_NAME,
        "LED SOLO Picker",
        JOptionPane.ERROR_MESSAGE
    )
else:
    def clamp_0_255(v):
        if v < 0:
            return 0
        if v > 255:
            return 255
        return int(v)

    def make_actuator_value(index, color, intensity):
        """Build the value inserted into the actuator's M150 command."""
        p = clamp_0_255(intensity)
        r = clamp_0_255(color.getRed())
        g = clamp_0_255(color.getGreen())
        b = clamp_0_255(color.getBlue())
        return "I%d P%d R%d U%d B%d" % (index, p, r, g, b)

    def make_cmd(index, color, intensity):
        """Build the complete command shown in the existing command display."""
        return "M150 " + make_actuator_value(index, color, intensity)

    def actuate_index(index, color, intensity):
        cmd = make_cmd(index, color, intensity)
        try:
            # Machine Setup defines this actuator as "M150 {StringValue}".
            led.actuate(make_actuator_value(index, color, intensity))
        except Exception as e:
            print("Error sending command to actuator: %s" % e)
        return cmd

    def create_ui():
        # State
        state = {
            "saved": False,
            "selected": 0,
            "intensity": DEFAULT_INTENSITY,
            "live_update": False
        }

        enabled = [False] * LED_COUNT
        colors = [Color.BLACK] * LED_COUNT

        # Main window
        frame = JFrame("LED SOLO Picker")
        frame.getContentPane().setLayout(BorderLayout())
        frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE)

        # --- Color chooser ---
        chooser = JColorChooser(Color.RED)
        chooser.setPreviewPanel(JPanel())

        # --- Intensity slider ---
        intensity_panel = JPanel()
        intensity_panel.setLayout(BoxLayout(intensity_panel, BoxLayout.Y_AXIS))
        intensity_label = JLabel("Intensity: %d" % DEFAULT_INTENSITY)
        intensity_slider = JSlider(JSlider.VERTICAL, 0, 255, DEFAULT_INTENSITY)
        intensity_slider.setMajorTickSpacing(64)
        intensity_slider.setMinorTickSpacing(16)
        intensity_slider.setPaintTicks(True)
        intensity_slider.setPaintLabels(True)
        intensity_slider.setPreferredSize(Dimension(70, 220))
        intensity_panel.add(intensity_label)
        intensity_panel.add(intensity_slider)

        # --- Command display ---
        cmd_panel = JPanel()
        cmd_label = JLabel("Last command:")
        cmd_field = JTextField(30)
        cmd_field.setEditable(False)
        cmd_panel.add(cmd_label)
        cmd_panel.add(cmd_field)

        selected_label = JLabel("Selected LED: I0")
        selected_preview = JPanel()
        selected_preview.setPreferredSize(Dimension(50, 25))
        selected_preview.setBackground(Color.RED)

        live_checkbox = JCheckBox("Live update selected LED while picking color")
        live_checkbox.setSelected(False)

        # --- LED ring panel ---
        class LedRingPanel(JPanel):
            def __init__(self):
                JPanel.__init__(self)
                self.setPreferredSize(Dimension(300, 300))
                # Approx square centers around a ring, index 0 at top and then clockwise.
                self.square_size = 42
                self.centers = [
                    (150, 45),   # I0 top
                    (224, 76),   # I1 upper-right
                    (255, 150),  # I2 right
                    (224, 224),  # I3 lower-right
                    (150, 255),  # I4 bottom
                    (76, 224),   # I5 lower-left
                    (45, 150),   # I6 left
                    (76, 76)     # I7 upper-left
                ]

            def paintComponent(self, g):
                self.super__paintComponent(g)
                g2 = g

                # Draw soft ring guide
                # Call methods through their public AWT base classes. Calling them
                # directly on g2 makes Jython reflect on the private
                # sun.java2d.SunGraphics2D implementation, which Java 9+ blocks.
                Graphics.setColor(g2, Color.LIGHT_GRAY)
                try:
                    Graphics2D.setStroke(g2, BasicStroke(3))
                except Exception:
                    pass
                Graphics.drawOval(g2, 65, 65, 170, 170)

                # Center label
                Graphics.setColor(g2, Color.DARK_GRAY)
                Graphics.drawString(g2, "Click LED squares", 100, 145)
                Graphics.drawString(g2, "I0 - I7", 128, 165)

                # Draw LED squares
                for i in range(LED_COUNT):
                    cx, cy = self.centers[i]
                    x = cx - self.square_size // 2
                    y = cy - self.square_size // 2

                    if enabled[i]:
                        Graphics.setColor(g2, colors[i])
                    else:
                        Graphics.setColor(g2, Color(35, 35, 35))
                    Graphics.fillRect(g2, x, y, self.square_size, self.square_size)

                    # Selection border
                    if i == state["selected"]:
                        Graphics.setColor(g2, Color.WHITE)
                        try:
                            Graphics2D.setStroke(g2, BasicStroke(4))
                        except Exception:
                            pass
                        Graphics.drawRect(g2, x - 3, y - 3, self.square_size + 6, self.square_size + 6)

                    # Normal border
                    Graphics.setColor(g2, Color.BLACK)
                    try:
                        Graphics2D.setStroke(g2, BasicStroke(2))
                    except Exception:
                        pass
                    Graphics.drawRect(g2, x, y, self.square_size, self.square_size)

                    # LED index text
                    Graphics.setColor(g2, Color.WHITE if enabled[i] else Color.LIGHT_GRAY)
                    Graphics.drawString(g2, "I%d" % i, x + 12, y + 26)

            def led_at_point(self, px, py):
                for i in range(LED_COUNT):
                    cx, cy = self.centers[i]
                    x = cx - self.square_size // 2
                    y = cy - self.square_size // 2
                    if px >= x and px <= x + self.square_size and py >= y and py <= y + self.square_size:
                        return i
                return -1

        ring = LedRingPanel()

        def refresh_selected_display():
            idx = state["selected"]
            selected_label.setText("Selected LED: I%d%s" % (idx, " ON" if enabled[idx] else " OFF"))
            if enabled[idx]:
                selected_preview.setBackground(colors[idx])
            else:
                selected_preview.setBackground(Color.BLACK)
            selected_preview.repaint()
            ring.repaint()

        def send_led(index):
            if enabled[index]:
                cmd = actuate_index(index, colors[index], state["intensity"])
            else:
                cmd = actuate_index(index, Color(OFF_R, OFF_G, OFF_B), state["intensity"])
            cmd_field.setText(cmd)

        def send_enabled_leds():
            sent = False
            for i in range(LED_COUNT):
                if enabled[i]:
                    send_led(i)
                    sent = True
            if not sent:
                idx = state["selected"]
                cmd_field.setText(make_cmd(idx, Color(OFF_R, OFF_G, OFF_B), state["intensity"]))

        def set_selected_color(color):
            idx = state["selected"]
            colors[idx] = color
            enabled[idx] = True
            send_led(idx)
            refresh_selected_display()

        def set_all_off():
            for i in range(LED_COUNT):
                enabled[i] = False
                colors[i] = Color.BLACK
                cmd = actuate_index(i, Color(OFF_R, OFF_G, OFF_B), state["intensity"])
                cmd_field.setText(cmd)
            refresh_selected_display()

        # Mouse handler for LED squares
        class RingMouseHandler(MouseAdapter):
            def mouseClicked(self, event):
                idx = ring.led_at_point(event.getX(), event.getY())
                if idx < 0:
                    return

                state["selected"] = idx

                # Toggle selected LED.
                if enabled[idx]:
                    enabled[idx] = False
                    send_led(idx)
                else:
                    c = chooser.getColor()
                    if c is None:
                        c = Color.WHITE
                    colors[idx] = c
                    enabled[idx] = True
                    send_led(idx)

                refresh_selected_display()

        ring.addMouseListener(RingMouseHandler())

        # Color chooser listener
        class ColorListener(ChangeListener):
            def stateChanged(self, event):
                c = chooser.getColor()
                if c is None:
                    return
                selected_preview.setBackground(c)
                selected_preview.repaint()

                if state["live_update"]:
                    idx = state["selected"]
                    colors[idx] = c
                    enabled[idx] = True
                    send_led(idx)
                    refresh_selected_display()

        chooser.getSelectionModel().addChangeListener(ColorListener())

        class IntensityListener(ChangeListener):
            def stateChanged(self, event):
                state["intensity"] = intensity_slider.getValue()
                intensity_label.setText("Intensity: %d" % state["intensity"])
                send_enabled_leds()
                refresh_selected_display()

        intensity_slider.addChangeListener(IntensityListener())

        def on_live_toggle(event):
            state["live_update"] = live_checkbox.isSelected()

        live_checkbox.addActionListener(on_live_toggle)

        # Buttons
        def on_set_color(event):
            c = chooser.getColor()
            if c is None:
                c = Color.WHITE
            set_selected_color(c)

        def on_all_off(event):
            set_all_off()

        def on_set(event):
            state["saved"] = True
            frame.dispose()

        def on_close(event):
            # If not saved, revert all individual LEDs off and close.
            if not state["saved"]:
                set_all_off()
            frame.dispose()

        set_color_button = JButton("Set Selected Color")
        set_color_button.addActionListener(on_set_color)

        all_off_button = JButton("All Off")
        all_off_button.addActionListener(on_all_off)

        set_button = JButton("Set")
        set_button.addActionListener(on_set)

        close_button = JButton("Close")
        close_button.addActionListener(on_close)

        class CloseHandler(WindowAdapter):
            def windowClosing(self, e):
                on_close(None)

        frame.addWindowListener(CloseHandler())

        # --- Assemble left panel ---
        left_panel = JPanel()
        left_panel.setLayout(BoxLayout(left_panel, BoxLayout.Y_AXIS))
        left_panel.add(ring)

        selected_panel = JPanel()
        selected_panel.add(selected_label)
        selected_panel.add(JLabel("Preview:"))
        selected_panel.add(selected_preview)
        left_panel.add(selected_panel)
        left_panel.add(live_checkbox)

        west_panel = JPanel()
        west_panel.setLayout(BorderLayout())
        west_panel.add(intensity_panel, BorderLayout.WEST)
        west_panel.add(left_panel, BorderLayout.CENTER)

        # --- Bottom buttons ---
        button_panel = JPanel()
        button_panel.add(set_color_button)
        button_panel.add(all_off_button)
        button_panel.add(set_button)
        button_panel.add(close_button)

        south_panel = JPanel()
        south_panel.setLayout(BoxLayout(south_panel, BoxLayout.Y_AXIS))
        south_panel.add(cmd_panel)
        south_panel.add(button_panel)

        frame.add(west_panel, BorderLayout.WEST)
        frame.add(chooser, BorderLayout.CENTER)
        frame.add(south_panel, BorderLayout.SOUTH)

        refresh_selected_display()

        frame.pack()
        frame.setLocationRelativeTo(gui)
        frame.setVisible(True)

    SwingUtilities.invokeLater(create_ui)
