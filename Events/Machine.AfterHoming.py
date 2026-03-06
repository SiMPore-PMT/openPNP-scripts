# Machine.AfterHoming.py
#
# Keep this event hook small. The actual tip-squaring workflow lives under
# Events/Homing so it can be edited and tested independently.

import imp
import os


def scripts_root():
    try:
        return scripting.getScriptsDirectory().toString()
    except:
        pass
    try:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    except:
        return os.getcwd()


SCRIPT_DIR = scripts_root()
SQUARING_PATH = os.path.join(SCRIPT_DIR, "Events", "Homing", "pickHead_squaring.py")

imp.load_source("pickHead_squaring", SQUARING_PATH)
