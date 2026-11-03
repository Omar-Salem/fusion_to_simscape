"""Filesystem locations for model export packages."""
from pathlib import Path
import re


def model_export_folder(parent, model_name):
    """Keep model names readable while preventing path separators or traversal."""
    folder_name = re.sub(r'[<>:"/\\|?*\x00-\x1f]', '_', model_name).strip().rstrip('.')
    return Path(parent) / (folder_name or 'Model')
