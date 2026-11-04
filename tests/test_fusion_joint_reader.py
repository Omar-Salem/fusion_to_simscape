"""Exercise the Fusion API adapter without requiring the native application."""
import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from FusionToSimscape.transforms import identity


class ApiJoint:
    """Minimal surface of Fusion Joint used by the reader."""
    def createForAssemblyContext(self, context):
        return self.proxy


class Matrix:
    def __init__(self, values):
        self.values = values

    def asArray(self):
        return self.values


def reader_module():
    adsk = ModuleType('adsk')
    adsk.core = ModuleType('adsk.core')
    adsk.fusion = ModuleType('adsk.fusion')
    adsk.fusion.JointTypes = SimpleNamespace(RigidJointType=0, RevoluteJointType=1, SliderJointType=2)
    adsk.fusion.Joint = ApiJoint
    adsk.fusion.JointDirections = SimpleNamespace(CustomJointDirection=3)
    modules = {'adsk': adsk, 'adsk.core': adsk.core, 'adsk.fusion': adsk.fusion}
    with patch.dict(sys.modules, modules):
        return importlib.import_module('FusionToSimscape.fusion_reader')


class FusionJointReaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = reader_module()

    def read_joint(self, joint_type, direction, as_built=False, custom_axis=(1., 2., -3.)):
        proxy_frame = [0., -1., 0., 10., 1., 0., 0., 20., 0., 0., 1., 30., 0., 0., 0., 1.]
        native_frame = identity()
        native = SimpleNamespace() if as_built else ApiJoint()
        native.name = 'Test motion'
        native.isSuppressed = False
        native.occurrenceTwo = SimpleNamespace(fullPathName='Base:1')
        native.occurrenceOne = SimpleNamespace(fullPathName='Moving:1')
        native.jointMotion = SimpleNamespace(jointType=joint_type, rotationAxis=direction,
                                            slideDirection=direction,
                                            rotationAxisVector=Matrix(custom_axis),
                                            slideDirectionVector=Matrix(custom_axis))
        proxy = SimpleNamespace() if as_built else ApiJoint()
        proxy.jointMotion = native.jointMotion
        if as_built:
            native.transform, proxy.transform = Matrix(native_frame), Matrix(proxy_frame)
        else:
            native.geometryTwoTransform = Matrix(native_frame)
            proxy.geometryTwoTransform = Matrix(proxy_frame)
        native.createForAssemblyContext = lambda context: proxy
        component = SimpleNamespace(joints=[] if as_built else [native],
                                    asBuiltJoints=[native] if as_built else [],
                                    rigidGroups=SimpleNamespace(count=0))
        reader = self.module.FusionReader(SimpleNamespace(rootComponent=component), Path('/unused'))
        reader.occurrence_bodies = {'Owner:1+Base:1': ['base'], 'Owner:1+Moving:1': ['moving']}
        reader._connections(component, 'Owner:1', object())
        return reader.joints[0]

    def test_joint_and_as_built_axis_dispatch_and_centimetre_conversion(self):
        for as_built in (False, True):
            for joint_type, kind in ((1, 'Revolute'), (2, 'Prismatic')):
                for direction, expected in enumerate(([0., 1., 0.], [-1., 0., 0.], [0., 0., 1.])):
                    with self.subTest(as_built=as_built, kind=kind, direction=direction):
                        joint = self.read_joint(joint_type, direction, as_built)
                        self.assertEqual(joint['type'], kind)
                        self.assertEqual((joint['base'], joint['follower']), ('base', 'moving'))
                        self.assertEqual(joint['frame'][2:12:4], expected)
                        self.assertEqual(joint['frame'][3:12:4], [.1, .2, .3])

    def test_custom_axes_use_native_vectors_and_solved_proxy_frames(self):
        expected = [-2/math.sqrt(14), 1/math.sqrt(14), -3/math.sqrt(14)]
        for as_built in (False, True):
            for joint_type in (1, 2):
                joint = self.read_joint(joint_type, 3, as_built)
                for actual, target in zip(joint['frame'][2:12:4], expected):
                    self.assertAlmostEqual(actual, target, places=12)

    def test_unsupported_joint_types_still_fail_explicitly(self):
        with self.assertRaisesRegex(ValueError, 'Unsupported joint type'):
            self.read_joint(3, 2)


class FusionRigidGroupReaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = reader_module()

    def test_native_members_are_scoped_to_each_repeated_assembly_instance(self):
        groups = [SimpleNamespace(name='Group', isSuppressed=False,
                                  occurrences=[SimpleNamespace(fullPathName='A:1'),
                                               SimpleNamespace(fullPathName='B:1')])]
        component = SimpleNamespace(joints=[], asBuiltJoints=[],
                                    rigidGroups=SimpleNamespace(count=1, item=lambda index: groups[index]))
        reader = self.module.FusionReader(SimpleNamespace(rootComponent=component), Path('/unused'))
        reader.occurrence_bodies = {'Owner:1+A:1': ['a1', 'a1_second_body'], 'Owner:1+B:1': ['b1'],
                                   'Owner:2+A:1': ['a2'], 'Owner:2+B:1': ['b2']}
        for owner in ('Owner:1', 'Owner:2'):
            reader._connections(component, owner, object())
        self.assertEqual(reader.rigid_groups, [dict(name='Owner:1/Group', members=['a1', 'a1_second_body', 'b1']),
                                              dict(name='Owner:2/Group', members=['a2', 'b2'])])

    def test_suppressed_hidden_and_empty_members_are_handled_explicitly(self):
        groups = [SimpleNamespace(name='Suppressed', isSuppressed=True),
                  SimpleNamespace(name='Visible', isSuppressed=False,
                                  occurrences=[SimpleNamespace(fullPathName=name)
                                               for name in ('A:1', 'Hidden:1', 'Empty:1')])]
        component = SimpleNamespace(joints=[], asBuiltJoints=[],
                                    rigidGroups=SimpleNamespace(count=2, item=lambda index: groups[index]))
        reader = self.module.FusionReader(SimpleNamespace(rootComponent=component), Path('/unused'))
        reader.occurrence_bodies = {'A:1': ['a'], 'Empty:1': [], 'Empty:1+MovingChild:1': ['child'],
                                   'Other:1+Hidden:1': ['unrelated_visible_body']}
        reader._connections(component, '', None)
        self.assertEqual(reader.rigid_groups, [dict(name='Visible', members=['a'])])
        self.assertEqual({record['member'] for record in reader.diagnostics}, {'Hidden:1', 'Empty:1'})
        self.assertNotIn('child', reader.rigid_groups[0]['members'])
