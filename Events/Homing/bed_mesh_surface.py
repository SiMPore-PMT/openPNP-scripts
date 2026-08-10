# bed_mesh_surface.py
#
# Support module loaded by the neighboring bed_mesh_gui.py.
#
# SVG/PNG renderer for interactive OpenPnP/Jython bed-mesh views. Jython never
# calls Graphics2D directly because OpenPnP's Java module settings hide the
# private sun.java2d.SunGraphics2D implementation from PyReflectedFunction.

from __future__ import print_function

import math
import time

from java.awt import (
    BorderLayout, Color, Dimension, FlowLayout, Font
)
from java.awt.event import ComponentAdapter, MouseAdapter
from java.awt.image import BufferedImage
from java.io import StringReader
from javax.swing import (
    BorderFactory, JButton, DefaultBoundedRangeModel, ImageIcon, JLabel, JPanel,
    JSlider, SwingConstants
)
from javax.swing.event import ChangeListener
from org.apache.batik.transcoder import TranscoderInput, TranscoderOutput
from org.apache.batik.transcoder.image import ImageTranscoder


DEFAULT_YAW = math.radians(45.0)
DEFAULT_PITCH = math.radians(-35.0)
MIN_PITCH = math.radians(-80.0)
MAX_PITCH = math.radians(-10.0)
MIN_ZOOM = 0.35
MAX_ZOOM = 5.0
IMAGE_MIN_WIDTH = 420
IMAGE_MIN_HEIGHT = 300


def new_scale_model():
    """Create a slider model that can be shared by every view in one run."""
    return DefaultBoundedRangeModel(100, 0, 25, 400)


def clamp(value, minimum, maximum):
    return max(float(minimum), min(float(maximum), float(value)))


def panel_points_by_cell(panel_result):
    result = {}
    for point in panel_result.get("points", []):
        result[(int(point.get("row")), int(point.get("column")))] = point
    return result


def triangulate_panel(panel_result):
    """Return faithful grid-cell triangles without filling missing samples."""
    config = panel_result.get("config", {})
    rows = int(config.get("rows", 0))
    columns = int(config.get("columns", 0))
    by_cell = panel_points_by_cell(panel_result)
    triangles = []
    for row in range(1, rows):
        for column in range(1, columns):
            p00 = by_cell.get((row, column))
            p10 = by_cell.get((row, column + 1))
            p01 = by_cell.get((row + 1, column))
            p11 = by_cell.get((row + 1, column + 1))
            if None in [p00, p10, p01, p11]:
                continue
            triangles.append((p00, p10, p11))
            triangles.append((p00, p11, p01))
    return triangles


def _panel_extent(panel_result):
    config = panel_result.get("config", {})
    x0 = float(config.get("lower_left_x_mm", 0.0))
    y0 = float(config.get("lower_left_y_mm", 0.0))
    width = float(config.get("width_mm", 0.0))
    height = float(config.get("height_mm", 0.0))
    return (min(x0, x0 + width), max(x0, x0 + width),
        min(y0, y0 + height), max(y0, y0 + height))


def compute_display_metrics(panel_results):
    """Compute run-wide color bounds and the automatic Z exaggeration."""
    panel_results = list(panel_results or [])
    x_values = []
    y_values = []
    offsets = []
    for panel_result in panel_results:
        x_min, x_max, y_min, y_max = _panel_extent(panel_result)
        x_values.extend([x_min, x_max])
        y_values.extend([y_min, y_max])
        for point in panel_result.get("points", []):
            x_values.append(float(point.get("x_mm", 0.0)))
            y_values.append(float(point.get("y_mm", 0.0)))
            offsets.append(float(point.get("z_offset_mm", 0.0)))
    if not x_values:
        x_values = [0.0, 1.0]
    if not y_values:
        y_values = [0.0, 1.0]
    x_min = min(x_values)
    x_max = max(x_values)
    y_min = min(y_values)
    y_max = max(y_values)
    xy_span = max(x_max - x_min, y_max - y_min, 1.0)
    max_abs_offset = max([abs(value) for value in offsets] or [0.0])
    # A symmetric -max..+max Z range occupies 20% of the X/Y span.
    auto_z_multiplier = (0.10 * xy_span / max_abs_offset
        if max_abs_offset > 0.0 else 1.0)
    return {
        "x_min": x_min,
        "x_max": x_max,
        "y_min": y_min,
        "y_max": y_max,
        "max_abs_offset": max_abs_offset,
        "auto_z_multiplier": auto_z_multiplier,
    }


def offset_color(value, maximum):
    """Blue -> white -> red symmetric color mapping."""
    if float(maximum) <= 0.0:
        return Color(245, 245, 245)
    ratio = clamp(float(value) / float(maximum), -1.0, 1.0)
    if ratio >= 0.0:
        fade = int(round(245.0 * (1.0 - ratio)))
        return Color(245, fade, fade)
    fade = int(round(245.0 * (1.0 + ratio)))
    return Color(fade, fade, 245)


def _shade(color, factor):
    factor = clamp(factor, 0.0, 1.25)
    return Color(
        int(clamp(color.getRed() * factor, 0.0, 255.0)),
        int(clamp(color.getGreen() * factor, 0.0, 255.0)),
        int(clamp(color.getBlue() * factor, 0.0, 255.0)),
        color.getAlpha())


def _color_hex(color):
    return "#%02x%02x%02x" % (
        color.getRed(), color.getGreen(), color.getBlue())


def _xml_text(value):
    return (str(value).replace("&", "&amp;").replace("<", "&lt;")
        .replace(">", "&gt;").replace('"', "&quot;"))


class _ImageCaptureTranscoder(ImageTranscoder):
    """Let Batik draw internally and retain its completed public image."""
    def __init__(self):
        ImageTranscoder.__init__(self)
        self.image = None

    def createImage(self, width, height):
        return BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB)

    def writeImage(self, image, output):
        self.image = image


def _point_xyz(point, z_multiplier):
    return (float(point.get("x_mm", 0.0)),
        float(point.get("y_mm", 0.0)),
        float(point.get("z_offset_mm", 0.0)) * float(z_multiplier))


def _normal_brightness(points):
    a, b, c = points
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length <= 1.0e-12:
        return 0.92
    nx, ny, nz = nx / length, ny / length, nz / length
    light = abs(nx * -0.25 + ny * -0.35 + nz * 0.90)
    return 0.78 + 0.22 * light


class _SurfaceMouseListener(MouseAdapter):
    def __init__(self, owner):
        self.owner = owner
        self.last_x = None
        self.last_y = None

    def mousePressed(self, event):
        self.last_x = event.getX()
        self.last_y = event.getY()

    def mouseReleased(self, event):
        self.last_x = None
        self.last_y = None
        self.owner.render()

    def mouseDragged(self, event):
        if self.last_x is None:
            self.mousePressed(event)
            return
        dx = event.getX() - self.last_x
        dy = event.getY() - self.last_y
        self.last_x = event.getX()
        self.last_y = event.getY()
        self.owner.yaw += float(dx) * 0.010
        self.owner.pitch = clamp(
            self.owner.pitch + float(dy) * 0.010, MIN_PITCH, MAX_PITCH)
        self.owner.render_if_due()

    def mouseWheelMoved(self, event):
        self.owner.zoom = clamp(
            self.owner.zoom * math.pow(1.12, -event.getWheelRotation()),
            MIN_ZOOM, MAX_ZOOM)
        self.owner.render()


class _SurfaceResizeListener(ComponentAdapter):
    def __init__(self, owner):
        self.owner = owner

    def componentResized(self, event):
        self.owner.render()


class _ScaleChangeListener(ChangeListener):
    def __init__(self, owner):
        self.owner = owner

    def stateChanged(self, event):
        self.owner.scale_ratio = float(self.owner.scale_slider.getValue()) / 100.0
        self.owner.update_scale_label()
        self.owner.render()


class BedMeshSurfaceView(object):
    """Interactive surface view backed by a normal Swing component tree."""

    def __init__(self, panel_results=None, shared_metrics=None,
            scale_model=None):
        self.panel_results = list(panel_results or [])
        self.shared_metrics = (shared_metrics or
            compute_display_metrics(self.panel_results))
        self.local_metrics = compute_display_metrics(self.panel_results)
        self.yaw = DEFAULT_YAW
        self.pitch = DEFAULT_PITCH
        self.zoom = 1.0
        self.scale_ratio = 1.0
        self._last_render_time = 0.0

        self.root = JPanel(BorderLayout(5, 5))
        self.image_label = JLabel("", SwingConstants.CENTER)
        self.image_label.setOpaque(True)
        self.image_label.setBackground(Color(28, 31, 36))
        self.image_label.setPreferredSize(Dimension(680, 390))
        self.image_label.setBorder(BorderFactory.createLineBorder(Color(75, 80, 88)))

        toolbar = JPanel(FlowLayout(FlowLayout.LEFT, 7, 2))
        reset = JButton("Reset View")
        reset.addActionListener(lambda event: self.reset_view())
        toolbar.add(reset)
        toolbar.add(JLabel("Vertical warp:"))
        self.scale_slider = JSlider(scale_model or new_scale_model())
        self.scale_slider.setPreferredSize(Dimension(180, 36))
        self.scale_slider.setMajorTickSpacing(75)
        self.scale_slider.setPaintTicks(True)
        self.scale_slider.addChangeListener(_ScaleChangeListener(self))
        toolbar.add(self.scale_slider)
        self.scale_label = JLabel()
        self.scale_label.setFont(self.scale_label.getFont().deriveFont(Font.BOLD))
        toolbar.add(self.scale_label)
        toolbar.add(JLabel("Drag to rotate; wheel to zoom"))
        self.root.add(toolbar, BorderLayout.NORTH)
        self.root.add(self.image_label, BorderLayout.CENTER)

        listener = _SurfaceMouseListener(self)
        self._mouse_listener = listener
        self.image_label.addMouseListener(listener)
        self.image_label.addMouseMotionListener(listener)
        self.image_label.addMouseWheelListener(listener)
        self._resize_listener = _SurfaceResizeListener(self)
        self.image_label.addComponentListener(self._resize_listener)
        self.update_scale_label()
        self.render()

    def get_component(self):
        return self.root

    def set_results(self, panel_results, shared_metrics=None):
        self.panel_results = list(panel_results or [])
        self.shared_metrics = (shared_metrics or
            compute_display_metrics(self.panel_results))
        self.local_metrics = compute_display_metrics(self.panel_results)
        self.update_scale_label()
        self.render()

    def reset_view(self):
        self.yaw = DEFAULT_YAW
        self.pitch = DEFAULT_PITCH
        self.zoom = 1.0
        self.render()

    def render_if_due(self):
        # SVG transcoding is intentionally throttled during a drag. The release
        # event always renders the final camera position.
        if time.time() - self._last_render_time >= 0.045:
            self.render()

    def effective_z_multiplier(self):
        return (float(self.shared_metrics.get("auto_z_multiplier", 1.0)) *
            float(self.scale_ratio))

    def update_scale_label(self):
        self.scale_label.setText("%.1fx (%.2fx auto)" % (
            self.effective_z_multiplier(), self.scale_ratio))

    def _rotate(self, xyz):
        metrics = self.local_metrics
        center_x = (metrics.get("x_min") + metrics.get("x_max")) * 0.5
        center_y = (metrics.get("y_min") + metrics.get("y_max")) * 0.5
        x = xyz[0] - center_x
        y = xyz[1] - center_y
        z = xyz[2]
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        x1 = cos_yaw * x - sin_yaw * y
        y1 = sin_yaw * x + cos_yaw * y
        cos_pitch = math.cos(self.pitch)
        sin_pitch = math.sin(self.pitch)
        view_y = cos_pitch * y1 - sin_pitch * z
        depth = sin_pitch * y1 + cos_pitch * z
        return (x1, view_y, depth)

    def _fit_transform(self, width, height, z_multiplier):
        metrics = self.local_metrics
        z_extent = float(self.shared_metrics.get("max_abs_offset", 0.0)) * z_multiplier
        corners = []
        for x in [metrics.get("x_min"), metrics.get("x_max")]:
            for y in [metrics.get("y_min"), metrics.get("y_max")]:
                for z in [-z_extent, z_extent]:
                    corners.append(self._rotate((x, y, z)))
        min_x = min([point[0] for point in corners])
        max_x = max([point[0] for point in corners])
        min_y = min([point[1] for point in corners])
        max_y = max([point[1] for point in corners])
        drawable_width = max(1.0, float(width) - 110.0)
        drawable_height = max(1.0, float(height) - 105.0)
        scale = min(drawable_width / max(max_x - min_x, 1.0e-9),
            drawable_height / max(max_y - min_y, 1.0e-9)) * self.zoom
        center_view_x = (min_x + max_x) * 0.5
        center_view_y = (min_y + max_y) * 0.5
        center_screen_x = float(width) * 0.5
        center_screen_y = float(height) * 0.5 - 8.0
        return (scale, center_view_x, center_view_y,
            center_screen_x, center_screen_y)

    def _project(self, xyz, transform):
        rotated = self._rotate(xyz)
        scale, center_x, center_y, screen_x, screen_y = transform
        return (int(round(screen_x + (rotated[0] - center_x) * scale)),
            int(round(screen_y - (rotated[1] - center_y) * scale)),
            rotated[2])

    def _line_3d_svg(self, first, second, transform, color, width=1.0,
            opacity=1.0):
        a = self._project(first, transform)
        b = self._project(second, transform)
        return ('<line x1="%d" y1="%d" x2="%d" y2="%d" '
            'stroke="%s" stroke-width="%.2f" opacity="%.3f" />' %
            (a[0], a[1], b[0], b[1], color, width, opacity))

    def _reference_plane_svg(self, transform):
        svg = []
        metrics = self.local_metrics
        x_min, x_max = metrics.get("x_min"), metrics.get("x_max")
        y_min, y_max = metrics.get("y_min"), metrics.get("y_max")
        for index in range(6):
            ratio = float(index) / 5.0
            x = x_min + (x_max - x_min) * ratio
            y = y_min + (y_max - y_min) * ratio
            svg.append(self._line_3d_svg((x, y_min, 0.0),
                (x, y_max, 0.0), transform, "#5b626c", 1.0, 0.45))
            svg.append(self._line_3d_svg((x_min, y, 0.0),
                (x_max, y, 0.0), transform, "#5b626c", 1.0, 0.45))
        return svg

    def _surfaces_svg(self, transform, z_multiplier):
        maximum = float(self.shared_metrics.get("max_abs_offset", 0.0))
        faces = []
        for panel_result in self.panel_results:
            for triangle in triangulate_panel(panel_result):
                xyz = [_point_xyz(point, z_multiplier) for point in triangle]
                projected = [self._project(point, transform) for point in xyz]
                depth = sum([point[2] for point in projected]) / 3.0
                offset = sum([float(point.get("z_offset_mm", 0.0))
                    for point in triangle]) / 3.0
                faces.append((depth, projected, xyz, offset))
        faces.sort(key=lambda face: face[0])
        svg = []
        for depth, projected, xyz, offset in faces:
            points = " ".join(["%d,%d" % (point[0], point[1])
                for point in projected])
            color = _shade(offset_color(offset, maximum),
                _normal_brightness(xyz))
            svg.append('<polygon points="%s" fill="%s" '
                'stroke="#24272c" stroke-width="1" />' %
                (points, _color_hex(color)))
        return svg

    def _panels_and_points_svg(self, transform, z_multiplier):
        maximum = float(self.shared_metrics.get("max_abs_offset", 0.0))
        svg = []
        for panel_result in self.panel_results:
            x_min, x_max, y_min, y_max = _panel_extent(panel_result)
            outline = [(x_min, y_min, 0.0), (x_max, y_min, 0.0),
                (x_max, y_max, 0.0), (x_min, y_max, 0.0)]
            for index in range(4):
                svg.append(self._line_3d_svg(outline[index],
                    outline[(index + 1) % 4], transform,
                    "#d2d7de", 1.8, 0.80))
            for point in panel_result.get("points", []):
                projected = self._project(_point_xyz(point, z_multiplier), transform)
                color = offset_color(
                    float(point.get("z_offset_mm", 0.0)), maximum)
                svg.append('<circle cx="%d" cy="%d" r="4" fill="%s" '
                    'stroke="#191b1f" stroke-width="1" />' %
                    (projected[0], projected[1], _color_hex(color)))
            label_point = self._project((x_min, y_max, 0.0), transform)
            svg.append('<text x="%d" y="%d" fill="#f0f2f5" '
                'font-family="sans-serif" font-size="12" font-weight="bold">%s</text>' %
                (label_point[0] + 5, label_point[1] - 5,
                 _xml_text(panel_result.get("panel_name", "Panel"))))
        return svg

    def _axes_svg(self, transform):
        metrics = self.local_metrics
        x_min, x_max = metrics.get("x_min"), metrics.get("x_max")
        y_min, y_max = metrics.get("y_min"), metrics.get("y_max")
        axis_size = max(x_max - x_min, y_max - y_min, 1.0) * 0.18
        origin = (x_min, y_min, 0.0)
        axes = [
            ((x_min + axis_size, y_min, 0.0), "#eb5c5c", "+X"),
            ((x_min, y_min + axis_size, 0.0), "#64dc87", "+Y"),
            ((x_min, y_min, axis_size * 0.65), "#69a0f5",
                "+Z offset"),
        ]
        svg = []
        for endpoint, color, label in axes:
            svg.append(self._line_3d_svg(
                origin, endpoint, transform, color, 2.2, 1.0))
            projected = self._project(endpoint, transform)
            svg.append('<text x="%d" y="%d" fill="%s" '
                'font-family="sans-serif" font-size="11" font-weight="bold">%s</text>' %
                (projected[0] + 4, projected[1] - 3, color, _xml_text(label)))
        return svg

    def _legend_svg(self, width, height):
        maximum = float(self.shared_metrics.get("max_abs_offset", 0.0))
        legend_width = min(240, max(130, width - 220))
        legend_x = width - legend_width - 22
        legend_y = height - 38
        return [
            '<rect x="%d" y="%d" width="%d" height="12" '
            'fill="url(#offsetGradient)" stroke="#dce0e6" stroke-width="1" />' %
            (legend_x, legend_y, legend_width),
            '<text x="%d" y="%d" fill="#dce0e6" font-family="sans-serif" '
            'font-size="11" text-anchor="start">%+.4f</text>' %
            (legend_x, legend_y + 27, -maximum),
            '<text x="%d" y="%d" fill="#dce0e6" font-family="sans-serif" '
            'font-size="11" text-anchor="middle">0.0000 mm offset</text>' %
            (legend_x + legend_width / 2, legend_y + 27),
            '<text x="%d" y="%d" fill="#dce0e6" font-family="sans-serif" '
            'font-size="11" text-anchor="end">%+.4f</text>' %
            (legend_x + legend_width, legend_y + 27, maximum),
        ]

    def _svg_text(self, width, height):
        z_multiplier = self.effective_z_multiplier()
        transform = self._fit_transform(width, height, z_multiplier)
        svg = [
            '<?xml version="1.0" encoding="UTF-8"?>',
            '<svg xmlns="http://www.w3.org/2000/svg" version="1.1" '
            'width="%d" height="%d" viewBox="0 0 %d %d">' %
            (width, height, width, height),
            '<defs><linearGradient id="offsetGradient" x1="0%" y1="0%" '
            'x2="100%" y2="0%"><stop offset="0%" stop-color="#0000f5" />'
            '<stop offset="50%" stop-color="#f5f5f5" />'
            '<stop offset="100%" stop-color="#f50000" />'
            '</linearGradient></defs>',
            '<rect x="0" y="0" width="%d" height="%d" fill="#1c1f24" />' %
            (width, height),
            '<text x="18" y="19" fill="#e1e4e9" font-family="sans-serif" '
            'font-size="11">Height = z_offset_mm x %.1f (visual exaggeration)</text>' %
            z_multiplier,
        ]
        svg.extend(self._reference_plane_svg(transform))
        svg.extend(self._surfaces_svg(transform, z_multiplier))
        svg.extend(self._panels_and_points_svg(transform, z_multiplier))
        svg.extend(self._axes_svg(transform))
        svg.extend(self._legend_svg(width, height))
        if not any([panel.get("points", []) for panel in self.panel_results]):
            svg.append('<text x="%d" y="%d" fill="#e1e4e9" '
                'font-family="sans-serif" font-size="16" font-weight="bold" '
                'text-anchor="middle">No completed mesh points</text>' %
                (width / 2, height / 2))
        svg.append('</svg>')
        return "".join(svg)

    def render(self):
        width = max(IMAGE_MIN_WIDTH, self.image_label.getWidth())
        height = max(IMAGE_MIN_HEIGHT, self.image_label.getHeight())
        transcoder = _ImageCaptureTranscoder()
        source = TranscoderInput(StringReader(self._svg_text(width, height)))
        transcoder.transcode(source, TranscoderOutput())
        if transcoder.image is None:
            raise RuntimeError("Batik did not produce a bed-mesh image")
        self.image_label.setIcon(ImageIcon(transcoder.image))
        self._last_render_time = time.time()
