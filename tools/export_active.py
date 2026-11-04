"""Developer entry point to run from Fusion's Python console."""
import importlib
from pathlib import Path
import sys
import traceback

REPOSITORY = Path(__file__).resolve().parents[1]
if str(REPOSITORY) not in sys.path:
    sys.path.insert(0, str(REPOSITORY))
try:
    for name in ('transforms', 'joint_frames', 'rigid_groups', 'xml_writer', 'fusion_reader', 'export_paths', 'FusionToSimscape'):
        module = importlib.import_module('FusionToSimscape.' + name)
        importlib.reload(module)
    module.choose_and_export()
except Exception:
    (REPOSITORY / 'exports' / 'fusion_error.log').write_text(traceback.format_exc())
    print(traceback.format_exc())
