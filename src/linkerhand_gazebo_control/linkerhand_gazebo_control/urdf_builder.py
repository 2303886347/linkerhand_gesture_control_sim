"""在不修改原始模型文件的前提下生成 Gazebo 受控 URDF。"""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from linkerhand_model_profiles import load_model_profile


DEFAULT_INERTIAL_SCALE = {
    side: load_model_profile('l30', side).gazebo_inertial_scale
    for side in ('left', 'right')
}


def _remove_gazebo_mimic_constraints(robot, profile):
    """Gazebo 中把四指 DIP 改为独立同步关节。"""
    for joint_name in profile.mimic_joints:
        joint = robot.find(f"joint[@name='{joint_name}']")
        mimic = joint.find('mimic')
        if mimic is not None:
            joint.remove(mimic)


def _remove_collision_meshes(robot):
    """手势显示模式移除会造成相邻指节互锁的高精度碰撞网格。"""
    for link in robot.findall('link'):
        for collision in list(link.findall('collision')):
            link.remove(collision)


def _resolve_package_mesh_uris(robot):
    """把 Gazebo GUI 无法解析的 ROS 包网格 URI 转为绝对文件 URI。"""
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename')
        if not filename or not filename.startswith('package://'):
            continue

        package_path = filename.removeprefix('package://')
        package_name, separator, relative_path = package_path.partition('/')
        if not separator or not package_name or not relative_path:
            raise ValueError(f'无效的 ROS 包资源 URI：{filename}')

        try:
            package_share = Path(get_package_share_directory(package_name))
        except PackageNotFoundError as error:
            raise ValueError(
                f'网格资源所属 ROS 包不存在：{package_name}（URI：{filename}）'
            ) from error

        mesh_path = package_share / relative_path
        if not mesh_path.is_file():
            raise ValueError(f'网格资源文件不存在：{mesh_path}（URI：{filename}）')

        mesh.set('filename', mesh_path.resolve().as_uri())


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


def _add_online_joint_controller(robot, side, profile):
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

    for joint_name in profile.controlled_joints:
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
    model_id='l30',
    profile=None,
):
    """附加固定世界根、在线控制和只读状态发布插件。"""
    source_path = Path(source_path)
    robot = ET.parse(source_path).getroot()
    side = str(side).strip().lower()
    if side not in {'left', 'right'}:
        raise ValueError(f'不支持的手侧：{side}')
    profile = profile or load_model_profile(model_id, side)
    if profile.side != side:
        raise ValueError(f'profile 手侧 {profile.side} 与请求手侧 {side} 不一致')
    if inertial_scale is None:
        inertial_scale = profile.gazebo_inertial_scale

    _resolve_package_mesh_uris(robot)
    _remove_gazebo_mimic_constraints(robot, profile)
    _remove_collision_meshes(robot)
    _normalize_joint_dynamics(robot)
    _scale_inertial_properties(robot, inertial_scale)

    if robot.find("link[@name='world']") is None:
        robot.insert(0, ET.Element('link', name='world'))
        world_joint = ET.Element(
            'joint', name=profile.world_joint_name, type='fixed'
        )
        ET.SubElement(world_joint, 'parent', link='world')
        ET.SubElement(world_joint, 'child', link=profile.root_link)
        ET.SubElement(world_joint, 'origin', xyz='0 0 0', rpy='0 0 0')
        robot.insert(1, world_joint)

    _add_online_joint_controller(robot, side, profile)
    _add_joint_state_publisher(robot, side)

    # deepcopy 可避免调用方后续持有的 Element 被序列化过程修改。
    return ET.tostring(deepcopy(robot), encoding='unicode')
