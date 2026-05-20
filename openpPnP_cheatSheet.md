# LumenPnP / OpenPnP Motion, Camera, and Actuator Control Reference

This is a minimal API/control surface extracted from the nozzle calibration scripts. It is intended as a starting point for writing a different Jython/OpenPnP script that can move the machine, use the camera, and control the nozzle/vacuum/blowoff hardware.

---

## Required OpenPnP / Java Imports

```python
import time
import traceback

from java.lang import Thread, Throwable
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi import Nozzle as SpiNozzle
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, VisionUtils
```

Optional, only when saving or converting camera/pipeline images:

```python
from java.io import File
from javax.imageio import ImageIO
from org.openpnp.util import OpenCvUtils
```

---

## Get OpenPnP Runtime Objects

### Machine

```python
machine_obj = Configuration.get().getMachine()
```

Or, inside an OpenPnP script context where `machine` exists:

```python
machine_obj = machine
```

### Configuration

```python
config_obj = Configuration.get()
```

Or, inside an OpenPnP script context where `config` exists:

```python
config_obj = config
```

### Head

```python
head = machine_obj.getDefaultHead()
```

### Default Nozzle

```python
nozzle = head.getDefaultNozzle()
```

### Nozzle by Name

```python
nozzle = head.getNozzleByName("Pick Head")
```

### Default Head Camera

```python
camera = head.getDefaultCamera()
```

### Down-Looking / Top Camera

```python
for cam in machine_obj.getCameras():
    if cam.getLooking() == SpiCamera.Looking.Down:
        camera = cam
        break
```

### Part by ID

```python
part = Configuration.get().getPart("PickCameraAlignDie")
```

---

## Build OpenPnP Locations

Use millimeters unless you intentionally need another unit.

```python
def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))
```

Example:

```python
target = mm_loc(40.0, 40.0, 25.0, 0.0)
```

Read current position:

```python
loc = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)

x = loc.getX()
y = loc.getY()
z = loc.getZ()
r = loc.getRotation()
```

---

## Wait for Motion Completion

```python
movable.waitForCompletion(CompletionType.WaitForStillstand)
```

Useful wrapper:

```python
def wait_still(movable):
    try:
        movable.waitForCompletion(CompletionType.WaitForStillstand)
    except:
        pass
```

---

## Motion Control

### Safe-Z Move Using OpenPnP Motion Planner

This uses OpenPnP’s safe-Z motion behavior and therefore routes through the configured machine driver, including gcodeAsync.

```python
MovableUtils.moveToLocationAtSafeZ(movable, target)
movable.waitForCompletion(CompletionType.WaitForStillstand)
```

Works with movable tools such as:

```python
MovableUtils.moveToLocationAtSafeZ(nozzle, target)
MovableUtils.moveToLocationAtSafeZ(camera, target)
```

### Direct Tool Move

This sends a direct move to the movable. In the calibration scripts this is used for controlled split moves.

```python
movable.moveTo(target)
movable.waitForCompletion(CompletionType.WaitForStillstand)
```

Some OpenPnP versions accept a speed argument:

```python
movable.moveTo(target, 1.0)
movable.waitForCompletion(CompletionType.WaitForStillstand)
```

Compatible wrapper:

```python
def move_to(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    try:
        movable.moveTo(target, 1.0)
    except TypeError:
        movable.moveTo(target)
    wait_still(movable)
```

### Move Camera

```python
target = mm_loc(x, y, z, camera.getLocation().getRotation())
MovableUtils.moveToLocationAtSafeZ(camera, target)
wait_still(camera)
```

### Move Nozzle

```python
target = mm_loc(x, y, z, rotation_deg)
MovableUtils.moveToLocationAtSafeZ(nozzle, target)
wait_still(nozzle)
```

Or direct:

```python
target = mm_loc(x, y, z, rotation_deg)
move_to(nozzle, target)
```

### Rotate Nozzle in Place

```python
current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
target = mm_loc(current.getX(), current.getY(), current.getZ(), rotation_deg)

move_to(nozzle, target)
```

### Lift Nozzle

```python
current = nozzle.getLocation().convertToUnits(LengthUnit.Millimeters)
target = mm_loc(current.getX(), current.getY(), safe_z, current.getRotation())

move_to(nozzle, target)
```

---

## Camera Control

### Capture Image

```python
image = camera.settleAndCapture()
```

If capture fails or you want a delay before capture:

```python
Thread.sleep(250)
image = camera.settleAndCapture()
```

### Save Captured Image

```python
ImageIO.write(image, "png", File("/path/to/capture.png"))
```

### Get Pixel Location in Machine Coordinates

Camera-only transform:

```python
machine_loc = VisionUtils.getPixelLocation(camera, x_px, y_px)
machine_loc = machine_loc.convertToUnits(LengthUnit.Millimeters)
```

Tool-aware transform:

```python
tool_loc = VisionUtils.getPixelLocation(camera, nozzle, x_px, y_px)
tool_loc = tool_loc.convertToUnits(LengthUnit.Millimeters)
```

The calibration code uses both:

```python
camera_only = VisionUtils.getPixelLocation(camera, px, py).convertToUnits(LengthUnit.Millimeters)
tool_aware = VisionUtils.getPixelLocation(camera, nozzle, px, py).convertToUnits(LengthUnit.Millimeters)
```

Use `camera_only` when you want the measured location in the camera frame.

Use `tool_aware` when you want the nozzle target location that accounts for the tool/nozzle offset.

---

## Pipeline / Vision Control

### Get Part Fiducial Pipeline

```python
settings = part.getFiducialVisionSettings()
pipeline = settings.getPipeline()
```

### Set Pipeline Properties

```python
pipeline.setProperty("camera", camera)
pipeline.setProperty("nozzle", nozzle)
pipeline.setProperty("part", part)
```

### Run Pipeline

```python
capture = camera.settleAndCapture()
pipeline.process()
```

### Read a Pipeline Stage Result

```python
result = pipeline.getResult("results")
model = result.getModel()
```

### Convert Java List-Like Model to Python List

```python
def java_list_to_py(model):
    if model is None:
        return []
    if hasattr(model, "size") and hasattr(model, "get"):
        out = []
        for i in range(model.size()):
            out.append(model.get(i))
        return out
    try:
        return list(model)
    except:
        return [model]
```

---

## Actuator Control

### Find Actuator by Name

```python
actuator = head.getActuatorByName("Actuator Name")
```

Fallback options used by the scripts:

```python
actuator = machine_obj.getActuatorByName("Actuator Name")
actuator = machine_obj.getActuator("Actuator Name")
```

Or scan actuators:

```python
for actuator in java_list_to_py(machine_obj.getActuators()):
    if actuator.getName() == "Actuator Name":
        break
```

### Resolve Vacuum Actuator from Nozzle / Head

```python
actuator = nozzle.getVacuumActuator()
```

Fallback used by the scripts:

```python
actuator = nozzle.getVacuumValveActuator()
```

Head-level fallback:

```python
actuator = nozzle.getHead().getVacuumActuator()
actuator = nozzle.getHead().getVacuumValveActuator()
```

### Resolve Blowoff Actuator from Nozzle / Head

```python
actuator = nozzle.getBlowOffActuator()
```

Fallback spelling:

```python
actuator = nozzle.getBlowoffActuator()
```

Head-level fallback:

```python
actuator = nozzle.getHead().getBlowOffActuator()
actuator = nozzle.getHead().getBlowoffActuator()
```

### Set Actuator State

```python
actuator.actuate(True)
actuator.actuate(False)
```

Fallback:

```python
actuator.setActuated(True)
actuator.setActuated(False)
```

Compatible wrapper:

```python
def set_actuator_state(actuator, on_off):
    if hasattr(actuator, "actuate"):
        actuator.actuate(bool(on_off))
    elif hasattr(actuator, "setActuated"):
        actuator.setActuated(bool(on_off))
    else:
        raise Exception("Actuator has no supported state method")
```

### Vacuum On / Off

```python
vacuum = nozzle.getVacuumActuator()
vacuum.actuate(True)    # vacuum on
vacuum.actuate(False)   # vacuum off
```

### Blowoff Pulse

```python
blowoff = nozzle.getBlowOffActuator()

blowoff.actuate(True)
Thread.sleep(120)
blowoff.actuate(False)
```

---

## Pick / Place Control

### High-Level Pick

```python
nozzle.pick(part, None)
```

Fallback if you are manually controlling vacuum:

```python
nozzle.setPart(part)
```

### High-Level Place / Release Fallback

The scripts mostly release by controlling vacuum/blowoff directly, then clearing nozzle part state:

```python
nozzle.setPart(None)
```

### Manual Pick Pattern

```python
vacuum = nozzle.getVacuumActuator()
vacuum.actuate(True)

Thread.sleep(150)

try:
    nozzle.pick(part, None)
except Throwable:
    nozzle.setPart(part)
```

### Manual Place Pattern

```python
vacuum = nozzle.getVacuumActuator()
vacuum.actuate(False)

blowoff = nozzle.getBlowOffActuator()
if blowoff is not None:
    blowoff.actuate(True)
    Thread.sleep(120)
    blowoff.actuate(False)

nozzle.setPart(None)
```

### Part-On Sensor Check

```python
if nozzle.isPartOnEnabled(SpiNozzle.PartOnStep.AfterPick):
    if not nozzle.isPartOn():
        raise Exception("Part-on sensor did not detect a part after pick.")
```

### Part-Off Sensor Check

```python
if nozzle.isPartOffEnabled(SpiNozzle.PartOffStep.AfterPlace):
    if not nozzle.isPartOff():
        raise Exception("Part-off sensor did not verify release after place.")
```

---

## Direct Machine / Nozzle Configuration Writes

### Read Nozzle Head Offsets

```python
offsets = nozzle.getHeadOffsets().convertToUnits(LengthUnit.Millimeters)

x = offsets.getX()
y = offsets.getY()
z = offsets.getZ()
r = offsets.getRotation()
```

### Write Nozzle Head Offsets

```python
new_offsets = Location(
    LengthUnit.Millimeters,
    new_x,
    new_y,
    old_offsets.getZ(),
    old_offsets.getRotation()
)

nozzle.setHeadOffsets(new_offsets)
```

---

## Minimal Reusable Control Skeleton

```python
from java.lang import Thread, Throwable
from org.openpnp.model import Configuration, Location, LengthUnit
from org.openpnp.spi import Camera as SpiCamera
from org.openpnp.spi import Nozzle as SpiNozzle
from org.openpnp.spi.MotionPlanner import CompletionType
from org.openpnp.util import MovableUtils, VisionUtils

def mm_loc(x, y, z, r):
    return Location(LengthUnit.Millimeters, float(x), float(y), float(z), float(r))

def wait_still(movable):
    try:
        movable.waitForCompletion(CompletionType.WaitForStillstand)
    except:
        pass

def move_safe(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    MovableUtils.moveToLocationAtSafeZ(movable, target)
    wait_still(movable)

def move_direct(movable, target):
    target = target.convertToUnits(LengthUnit.Millimeters)
    try:
        movable.moveTo(target, 1.0)
    except TypeError:
        movable.moveTo(target)
    wait_still(movable)

def set_actuator_state(actuator, on_off):
    if actuator is None:
        return False
    if hasattr(actuator, "actuate"):
        actuator.actuate(bool(on_off))
    elif hasattr(actuator, "setActuated"):
        actuator.setActuated(bool(on_off))
    else:
        raise Exception("Actuator has no supported state method")
    return True

machine_obj = Configuration.get().getMachine()
head = machine_obj.getDefaultHead()
nozzle = head.getDefaultNozzle()
camera = head.getDefaultCamera()

part = Configuration.get().getPart("PickCameraAlignDie")

# Move camera
move_safe(camera, mm_loc(40, 40, 25, camera.getLocation().getRotation()))

# Capture
image = camera.settleAndCapture()

# Convert a detected pixel into a nozzle target
tool_loc = VisionUtils.getPixelLocation(camera, nozzle, 320, 240)
tool_loc = tool_loc.convertToUnits(LengthUnit.Millimeters)

# Move nozzle to that tool-aware target
move_safe(nozzle, mm_loc(tool_loc.getX(), tool_loc.getY(), 25, 0))

# Vacuum / pick
vacuum = nozzle.getVacuumActuator()
set_actuator_state(vacuum, True)
Thread.sleep(150)

try:
    nozzle.pick(part, None)
except Throwable:
    nozzle.setPart(part)

# Place / release
set_actuator_state(vacuum, False)

try:
    blowoff = nozzle.getBlowOffActuator()
except:
    blowoff = None

if blowoff is not None:
    set_actuator_state(blowoff, True)
    Thread.sleep(120)
    set_actuator_state(blowoff, False)

nozzle.setPart(None)
```

---

## Extra Available Calls Seen in the Codebase

These are not required for basic motion/control, but are useful when building more advanced scripts.

```python
object.getName()
object.getId()

machine_obj.getCameras()
machine_obj.getActuators()
machine_obj.getDefaultHead()

head.getDefaultNozzle()
head.getNozzleByName(name)
head.getDefaultCamera()
head.getCameras()
head.getActuatorByName(name)

camera.getLooking()
camera.getLocation()
camera.settleAndCapture()

nozzle.getLocation()
nozzle.getHead()
nozzle.getHeadOffsets()
nozzle.setHeadOffsets(location)
nozzle.getVacuumActuator()
nozzle.getVacuumValveActuator()
nozzle.getBlowOffActuator()
nozzle.getBlowoffActuator()
nozzle.pick(part, None)
nozzle.setPart(part_or_none)
nozzle.isPartOnEnabled(SpiNozzle.PartOnStep.AfterPick)
nozzle.isPartOn()
nozzle.isPartOffEnabled(SpiNozzle.PartOffStep.AfterPlace)
nozzle.isPartOff()

movable.moveTo(location)
movable.moveTo(location, speed)
movable.waitForCompletion(CompletionType.WaitForStillstand)

MovableUtils.moveToLocationAtSafeZ(movable, location)

VisionUtils.getPixelLocation(camera, x_px, y_px)
VisionUtils.getPixelLocation(camera, nozzle, x_px, y_px)

actuator.actuate(True_or_False)
actuator.setActuated(True_or_False)

part.getFiducialVisionSettings()
settings.getPipeline()
pipeline.setProperty(name, value)
pipeline.process()
pipeline.getResult(stage_name)
pipeline.getWorkingImage()
result.getModel()
```
