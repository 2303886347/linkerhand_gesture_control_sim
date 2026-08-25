"""在不修改原始模型文件的前提下生成 Gazebo 受控 URDF。"""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from linkerhand_gazebo_control.joints import CONTROLLED_JOINTS
from linkerhand_retargeting.joints import MIMIC_JOINTS


DEFAULT_INERTIAL_SCALE = {
    'left': 1.0 / 7.6,
    'right': 1.0,
}


def _remove_gazebo_mimic_constraints(robot):
    """Gazebo 中把四指 DIP 改为独立同步关节。"""
    for joint_name in MIMIC_JOINTS:
        joint = robot.find(f"joint[@name='{joint_name}']")
        mimic = joint.find('mimic')
        if mimic is not None:
            joint.remove(mimic)


def _remove_collision_meshes(robot):
    """手势显示模式移除会造成相邻指节互锁的高精度碰撞网格。"""
    for link in robot.findall('link'):
        for collision in list(link.findall('collision')):
            link.remove(collision)


def _normalize_joint_dynamics(robot):
    """保留粘性阻尼，并关闭会让轻量指节产生静态卡滞的库仑摩擦。"""
    for joint in robot.findall('joint'):
        dynamics = joint.find('dynamics')
        if dynamics is not None:
            dynamics.set('friction', '0.0')


def _scale_inertial_properties(robot, scale):
    """按统一比例缩放质量和惯量矩阵，修正模型导出时的密度尺度偏差。"""
    scale = float(scale)
    if scale <= 0.0:
        raise ValueError('inertial_scale 必须大于 0')

    for inertial in robot.findall('link/inertial'):
        mass = inertial.find('mass')
        if mass is not None:
            mass.set('value', str(float(mass.get('value')) * scale))

        inertia = inertial.find('inertia')
        if inertia is not None:
            for component in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
                inertia.set(
                    component,
                    str(float(inertia.get(component)) * scale),
                )


def _add_online_joint_controller(robot, side):
    """添加支持持续在线目标的多关节位置控制插件。"""
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(
        gazebo,
        'plugin',
        filename='liblinkerhand_online_joint_controller.so',
        name='linkerhand_gazebo_plugin::OnlineJointController',
    )
    ET.SubElement(plugin, 'topic').text = f'/{side}/gazebo_joint_trajectory'
    ET.SubElement(plugin, 'position_gain').text = '8.0'
    ET.SubElement(plugin, 'max_velocity').text = '3.0'

    for joint_name in CONTROLLED_JOINTS:
        ET.SubElement(plugin, 'joint_name').text = joint_name


def _add_joint_state_publisher(robot, side):
    """使用 Gazebo 官方只读插件发布实际关节状态。"""
    gazebo = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(
        gazebo,
        'plugin',
        filename='ignition-gazebo-joint-state-publisher-system',
        name='gz::sim::systems::JointStatePublisher',
    )
    ET.SubElement(plugin, 'topic').text = (
        f'/{side}/gazebo_joint_states_raw'
    )


def build_controlled_urdf(
    source_path,
    side,
    inertial_scale=None,
):
    """附加固定世界根、在线控制和只读状态发布插件。"""
    source_path = Path(source_path)
    robot = ET.parse(source_path).getroot()
    side = str(side).strip().lower()
    if side not in {'left', 'right'}:
        raise ValueError(f'不支持的手侧：{side}')
    if inertial_scale is None:
        inertial_scale = DEFAULT_INERTIAL_SCALE[side]

    _remove_gazebo_mimic_constraints(robot)
    _remove_collision_meshes(robot)
    _normalize_joint_dynamics(robot)
    _scale_inertial_properties(robot, inertial_scale)

    if robot.find("link[@name='world']") is None:
        robot.insert(0, ET.Element('link', name='world'))
        world_joint = ET.Element(
            'joint', name='world_to_base_footprint', type='fixed'
        )
        ET.SubElement(world_joint, 'parent', link='world')
        ET.SubElement(world_joint, 'child', link='base_footprint')
        ET.SubElement(world_joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        robot.insert(1, world_joint)

    _add_online_joint_controller(robot, side)
    _add_joint_state_publisher(robot, side)

    # deepcopy 可避免调用方后续持有的 Element 被序列化过程修改。
    return ET.tostring(deepcopy(robot), encoding='unicode')
