"""验证项目 Gazebo world 的近景相机和运行所需系统。"""

import xml.etree.ElementTree as ET
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


PACKAGE_DIR = Path(__file__).parents[1]
WORLD_PATH = PACKAGE_DIR / 'worlds' / 'linkerhand_demo.sdf'


def _load_launch(filename):
    path = PACKAGE_DIR / 'launch' / filename
    spec = importlib.util.spec_from_file_location(
        filename.replace('.', '_'), path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _launch_default(launch_description, argument_name):
    declaration = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
        and entity.name == argument_name
    )
    return perform_substitutions(
        LaunchContext(), declaration.default_value
    )


def test_demo_world_has_close_up_camera_and_scene_controls():
    world = ET.parse(WORLD_PATH).getroot().find('world')
    assert world is not None

    minimal_scene = world.find(
        "gui/plugin[@filename='MinimalScene']"
    )
    assert minimal_scene is not None
    assert minimal_scene.findtext('camera_pose') == (
        '0.34 0 0.38 0 0.25 3.14159'
    )
    assert minimal_scene.findtext('ambient_light') == '0.25 0.25 0.25'

    scene = world.find('scene')
    assert scene is not None
    assert scene.findtext('ambient') == '0.35 0.35 0.35'

    gui_plugins = {
        plugin.get('filename') for plugin in world.findall('gui/plugin')
    }
    assert {'GzSceneManager', 'InteractiveViewControl', 'WorldControl'} <= (
        gui_plugins
    )


def test_demo_world_keeps_required_gazebo_systems_and_ground():
    world = ET.parse(WORLD_PATH).getroot().find('world')
    plugin_filenames = {
        plugin.get('filename') for plugin in world.findall('plugin')
    }

    assert {
        'ignition-gazebo-physics-system',
        'ignition-gazebo-user-commands-system',
        'ignition-gazebo-scene-broadcaster-system',
    } <= plugin_filenames
    assert world.find("model[@name='ground_plane']") is not None


def test_setup_installs_demo_world():
    setup_source = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')

    assert "'/worlds', glob('worlds/*.sdf')" in setup_source


def test_gazebo_launches_default_to_demo_world():
    for filename in (
        'gazebo_control.launch.py',
        'mediapipe_gazebo.launch.py',
    ):
        default_world = Path(_launch_default(_load_launch(filename), 'world'))

        assert default_world.name == 'linkerhand_demo.sdf'
        assert default_world.is_file()


def test_mediapipe_launch_forwards_world_override():
    launch_source = (
        PACKAGE_DIR / 'launch' / 'mediapipe_gazebo.launch.py'
    ).read_text(encoding='utf-8')

    assert "'world': LaunchConfiguration('world')" in launch_source
