# LedColorPicker.py
# Embedded color wheel (JColorChooser) with:
# - Vertical intensity slider (P value 0–255) on the left
# - Live LED updates as you pick colors or move the slider
# - Single preview swatch in the bottom bar
# - Command field to copy Pxxx Rxxx Uxxx Bxxx
# - Set button: keep last color/intensity and exit
# - Close/X: revert to white (P255 R255 U255 B255) and exit

from javax.swing import (
    JFrame, JPanel, JTextField, JLabel, JOptionPane,
    BoxLayout, JButton, JColorChooser, JSlider
)
from javax.swing import SwingUtilities
from javax.swing.event import ChangeListener
from java.awt import Color, Dimension, BorderLayout
from java.awt.event import WindowAdapter
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

led = machine.getActuatorByName(ACTUATOR_NAME)

if led is None:
    JOptionPane.showMessageDialog(
        None,
        "Actuator '%s' not found.\nCheck the name in Machine Setup → Actuators." % ACTUATOR_NAME,
        "LED Color Picker",
        JOptionPane.ERROR_MESSAGE
    )
else:
    def create_ui():
        # State: whether user clicked Set
        state = {"saved": False}

        # Main window
        frame = JFrame("LED Color Picker")
        frame.getContentPane().setLayout(BorderLayout())
        frame.setDefaultCloseOperation(JFrame.DO_NOTHING_ON_CLOSE)

        # --- Color chooser (embedded color wheel, center) ---
        chooser = JColorChooser(Color.WHITE)
        # Remove the internal preview so we only have our bottom one
        chooser.setPreviewPanel(JPanel())

        # --- Intensity slider (left, vertical) ---
        intensity_panel = JPanel()
        intensity_panel.setLayout(BoxLayout(intensity_panel, BoxLayout.Y_AXIS))
        intensity_label = JLabel("Intensity")
        # vertical slider, 0–255, default 255
        intensity_slider = JSlider(JSlider.VERTICAL, 0, 255, 255)
        intensity_slider.setMajorTickSpacing(64)
        intensity_slider.setMinorTickSpacing(16)
        intensity_slider.setPaintTicks(True)
        intensity_slider.setPaintLabels(True)
        # Make it tall, not cramped
        intensity_slider.setPreferredSize(Dimension(70, 220))

        intensity_panel.add(intensity_label)
        intensity_panel.add(intensity_slider)

        # --- Command display (copy/paste target) ---
        cmd_panel = JPanel()
        cmd_label = JLabel("Command (copy for pipeline):")
        cmd_field = JTextField(25)
        cmd_panel.add(cmd_label)
        cmd_panel.add(cmd_field)

        # --- Single Preview + buttons (bottom bar) ---
        bottom_panel = JPanel()
        bottom_panel.setLayout(BoxLayout(bottom_panel, BoxLayout.X_AXIS))

        preview_label = JLabel("Preview:")
        color_preview = JPanel()
        color_preview.setPreferredSize(Dimension(50, 25))
        color_preview.setBackground(Color.WHITE)

        # Live-update function
        def update_from_color_and_intensity():
            c = chooser.getColor()
            if c is None:
                return
            r = c.getRed()
            g = c.getGreen()
            b = c.getBlue()
            brightness = intensity_slider.getValue()  # P value from slider

            cmd = "P%d R%d U%d B%d" % (brightness, r, g, b)
            cmd_field.setText(cmd)
            color_preview.setBackground(c)
            color_preview.repaint()

            # Live-send to the actuator
            try:
                led.actuate(cmd)
            except Exception as e:
                # Avoid popup spam; just log to console
                print("Error sending command to actuator: %s" % e)

        # Listener for color changes
        class ColorListener(ChangeListener):
            def stateChanged(self, event):
                update_from_color_and_intensity()

        # Listener for intensity slider changes
        class IntensityListener(ChangeListener):
            def stateChanged(self, event):
                update_from_color_and_intensity()

        chooser.getSelectionModel().addChangeListener(ColorListener())
        intensity_slider.addChangeListener(IntensityListener())

        # Buttons
        def on_set(event):
            # Mark as saved and close; do NOT revert
            state["saved"] = True
            frame.dispose()

        def on_close(event):
            # If not saved, revert to white full on and close
            if not state["saved"]:
                try:
                    led.actuate("P255 R255 U255 B255")
                except Exception as e:
                    print("Error reverting to white: %s" % e)
            frame.dispose()

        set_button = JButton("Set")
        set_button.addActionListener(on_set)

        close_button = JButton("Close")
        close_button.addActionListener(on_close)

        bottom_panel.add(preview_label)
        bottom_panel.add(color_preview)
        bottom_panel.add(set_button)
        bottom_panel.add(close_button)

        # Handle window X the same as Close
        class CloseHandler(WindowAdapter):
            def windowClosing(self, e):
                on_close(None)

        frame.addWindowListener(CloseHandler())

        # Initialize with chooser's default color (white) and full intensity
        update_from_color_and_intensity()

        # --- Assemble layout ---
        frame.add(intensity_panel, BorderLayout.WEST)   # vertical slider on the left
        frame.add(chooser, BorderLayout.CENTER)

        south_panel = JPanel()
        south_panel.setLayout(BoxLayout(south_panel, BoxLayout.Y_AXIS))
        south_panel.add(cmd_panel)
        south_panel.add(bottom_panel)

        frame.add(south_panel, BorderLayout.SOUTH)

        frame.pack()
        frame.setLocationRelativeTo(gui)  # center relative to main window if available
        frame.setVisible(True)

    # Build UI on Swing's event thread
    SwingUtilities.invokeLater(create_ui)

