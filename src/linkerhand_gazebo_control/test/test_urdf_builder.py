from pathlib import Path
import xml.etree.ElementTree as ET

from linkerhand_gazebo_control.joints import CONTROLLED_JOINTS
from linkerhand_gazebo_control.urdf_builder import (
    DEFAULT_INERTIAL_SCALE,
    build_controlled_urdf,
)
from linkerhand_retargeting.joints import MIMIC_JOINTS


WORKSPACE_SRC = Path(__file__).resolve().parents[2]


def test_builder_adds_native_controller_and_joint_state_publisher():
    source = (
        WORKSPACE_SRC
        / 'linkerhand_l30_right_description'
        / 'urdf'
        / 'linkerhand_l30_right.urdf'
    )
    result = build_controlled_urdf(
        source,
        'right',
    )
    robot = ET.fromstring(result)

    assert robot.find("link[@name='world']") is not None
    assert robot.findall('link/collision') == []
    assert robot.findall('link/visual')
    for dynamics in robot.findall('joint/dynamics'):
        assert float(dynamics.get('friction')) == 0.0
        assert float(dynamics.get('damping')) > 0.0
    root_joint = robot.find("joint[@name='world_to_base_footprint']")
    assert root_joint is not None
    assert root_joint.find('parent').get('link') == 'world'

    assert robot.find('ros2_control') is None
    for mimic in MIMIC_JOINTS:
        source_joint = robot.find(f"joint[@name='{mimic}']")
        assert source_joint.find('mimic') is None

    native = robot.find(
        "gazebo/plugin[@name='linkerhand_gazebo_plugin::OnlineJointController']"
    )
    assert native is not None
    assert native.get('filename') == 'liblinkerhand_online_joint_controller.so'
    assert native.find('topic').text == '/right/gazebo_joint_trajectory'
    assert float(native.find('position_gain').text) > 0.0
    assert float(native.find('max_velocity').text) > 0.0
    assert [item.text for item in native.findall('joint_name')] == list(
        CONTROLLED_JOINTS
    )

    state_publisher = robot.find(
        "gazebo/plugin[@name='gz::sim::systems::JointStatePublisher']"
    )
    assert state_publisher is not None
    assert state_publisher.find('topic').text == (
        '/right/gazebo_joint_states_raw'
    )


def test_left_builder_corrects_exported_inertial_scale():
    source = (
        WORKSPACE_SRC
        / 'linkerhand_l30_left_description'
        / 'urdf'
        / 'linkerhand_l30_left.urdf'
    )
    source_robot = ET.parse(source).getroot()
    source_mass = float(
        source_robot.find("link[@name='index_proximal']/inertial/mass").get(
            'value'
        )
    )

    result = build_controlled_urdf(
        source,
        'left',
    )
    robot = ET.fromstring(result)
    scaled_mass = float(
        robot.find("link[@name='index_proximal']/inertial/mass").get('value')
    )

    assert scaled_mass == source_mass * DEFAULT_INERTIAL_SCALE['left']


def test_builder_rejects_non_positive_inertial_scale():
    source = (
        WORKSPACE_SRC
        / 'linkerhand_l30_right_description'
        / 'urdf'
        / 'linkerhand_l30_right.urdf'
    )

    try:
        build_controlled_urdf(
            source,
            'right',
            inertial_scale=0.0,
        )
    except ValueError as error:
        assert 'inertial_scale' in str(error)
    else:
        raise AssertionError('零惯量缩放必须被拒绝')
