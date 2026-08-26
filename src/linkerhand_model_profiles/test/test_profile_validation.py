"""验证损坏的型号配置会在节点启动前被明确拒绝。"""

from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

from linkerhand_model_profiles import ProfileError, load_model_profile_from_files


@pytest.fixture
def l30_files(tmp_path):
    profile_share = Path(
        get_package_share_directory('linkerhand_model_profiles')
    )
    description_share = Path(
        get_package_share_directory('linkerhand_l30_left_description')
    )
    model = yaml.safe_load(
        (profile_share / 'config' / 'l30' / 'model.yaml').read_text(
            encoding='utf-8'
        )
    )
    side = yaml.safe_load(
        (profile_share / 'config' / 'l30' / 'left.yaml').read_text(
            encoding='utf-8'
        )
    )
    urdf = ET.parse(
        description_share / 'urdf' / 'linkerhand_l30_left.urdf'
    )

    def write(model_data=None, side_data=None, urdf_tree=None):
        model_path = tmp_path / 'model.yaml'
        side_path = tmp_path / 'left.yaml'
        urdf_path = tmp_path / 'left.urdf'
        model_path.write_text(
            yaml.safe_dump(
                model_data if model_data is not None else model,
                sort_keys=False,
            ),
            encoding='utf-8',
        )
        side_path.write_text(
            yaml.safe_dump(
                side_data if side_data is not None else side,
                sort_keys=False,
            ),
            encoding='utf-8',
        )
        (urdf_tree if urdf_tree is not None else urdf).write(
            urdf_path, encoding='utf-8', xml_declaration=True
        )
        return model_path, side_path, urdf_path

    return model, side, urdf, write


def test_rejects_unsupported_schema(l30_files):
    model, _, _, write = l30_files
    damaged = deepcopy(model)
    damaged['schema_version'] = 999

    with pytest.raises(ProfileError, match='不支持的 profile schema'):
        load_model_profile_from_files(*write(model_data=damaged))


def test_rejects_overlapping_active_and_mimic_joints(l30_files):
    model, _, _, write = l30_files
    damaged = deepcopy(model)
    damaged['active_joints'].append('index_dip')

    with pytest.raises(ProfileError, match='关节集合必须互斥'):
        load_model_profile_from_files(*write(model_data=damaged))


def test_rejects_joint_missing_from_urdf(l30_files):
    _, _, urdf, write = l30_files
    damaged = deepcopy(urdf)
    root = damaged.getroot()
    missing_joint = root.find("joint[@name='index_pip']")
    root.remove(missing_joint)

    with pytest.raises(ProfileError, match='URDF 中不存在关节：index_pip'):
        load_model_profile_from_files(*write(urdf_tree=damaged))


def test_rejects_mimic_source_that_disagrees_with_urdf(l30_files):
    model, _, _, write = l30_files
    damaged = deepcopy(model)
    damaged['mimic_joints']['index_dip']['source'] = 'middle_pip'

    with pytest.raises(ProfileError, match='mimic 源不一致'):
        load_model_profile_from_files(*write(model_data=damaged))


def test_rejects_mapping_output_beyond_urdf_limit(l30_files):
    model, _, _, write = l30_files
    damaged = deepcopy(model)
    damaged['mapping_defaults']['index_pip']['output_max_rad'] = 10.0

    with pytest.raises(ProfileError, match='output_max 超出 URDF 限位'):
        load_model_profile_from_files(*write(model_data=damaged))
