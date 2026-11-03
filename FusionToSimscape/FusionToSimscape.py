"""Fusion entry point."""
from pathlib import Path
import json
import shutil
import traceback
import adsk.core

COMMAND_ID = 'FusionToSimscapeExport'
PANEL_ID = 'SolidScriptsAddinsPanel'
HANDLERS = []


def export_active(output):
    import adsk.fusion
    from .fusion_reader import FusionReader
    from .xml_writer import write
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None:
        raise ValueError('Open a Fusion design before exporting.')
    snapshot = FusionReader(design, output).read()
    destination = Path(output)
    (destination / 'assembly.json').write_text(json.dumps(snapshot, indent=2))
    write(snapshot, destination / 'assembly.xml')
    templates = Path(__file__).resolve().parent
    for template in templates.glob('*.m'):
        shutil.copy2(template, destination / template.name)
    shutil.copytree(templates / 'private', destination / 'private', dirs_exist_ok=True)
    print(f"Exported {len(snapshot['bodies'])} solids and {len(snapshot['joints'])} joints to {destination}")
    return snapshot


def choose_and_export():
    try:
        dialog = adsk.core.Application.get().userInterface.createFolderDialog()
        dialog.title = 'Choose Simscape Multibody export folder'
        if dialog.showDialog() != adsk.core.DialogResults.DialogOK:
            return
        export_active(dialog.folder)
        adsk.core.Application.get().userInterface.messageBox('Export complete: ' + dialog.folder)
    except Exception:
        error = traceback.format_exc()
        adsk.core.Application.get().userInterface.messageBox(error)


class ExecuteHandler(adsk.core.CommandEventHandler):
    def notify(self, args):
        choose_and_export()


class CreatedHandler(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        handler = ExecuteHandler()
        args.command.execute.add(handler)
        HANDLERS.append(handler)


def run(context):
    ui = adsk.core.Application.get().userInterface
    definition = ui.commandDefinitions.itemById(COMMAND_ID)
    if definition is None:
        definition = ui.commandDefinitions.addButtonDefinition(
            COMMAND_ID, 'Export to Simscape Multibody',
            'Export visible solids, mass properties, and assembly joints.')
    handler = CreatedHandler()
    definition.commandCreated.add(handler)
    HANDLERS.append(handler)
    panel = ui.allToolbarPanels.itemById(PANEL_ID)
    if panel is None:
        raise RuntimeError('Fusion Scripts and Add-ins toolbar panel not found.')
    if panel.controls.itemById(COMMAND_ID) is None:
        panel.controls.addCommand(definition)


def stop(context):
    ui = adsk.core.Application.get().userInterface
    panel = ui.allToolbarPanels.itemById(PANEL_ID)
    if panel:
        control = panel.controls.itemById(COMMAND_ID)
        if control:
            control.deleteMe()
    definition = ui.commandDefinitions.itemById(COMMAND_ID)
    if definition:
        definition.deleteMe()
    HANDLERS.clear()
