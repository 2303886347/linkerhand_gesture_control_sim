"""解析多型号 RViz 启动所需的 profile、URDF 和重定向配置。"""

from dataclasses import dataclass
from pathlib import Path

from linkerhand_model_profiles import ModelProfile, load_model_profile


@dataclass(frozen=True)
class RvizHandSpec:
    """单侧机械手在 RViz 管线中的确定性运行配置。"""

    model_id: str
    side: str
    profile: ModelProfile
    robot_description: str
    parameters_file: Path | None
    uses_profile_defaults: bool


def _default_parameters_path(retargeting_share, model_id, side):
    """按型号查找标定；L30 继续使用已经公开的兼容文件名。"""
    config_dir = Path(retargeting_share) / 'config'
    filename = (
        f'retargeting_{side}.yaml'
        if model_id == 'l30'
        else f'retargeting_{model_id}_{side}.yaml'
    )
    path = config_dir / filename
    return path if path.is_file() else None


def resolve_rviz_hand_spec(
    model_id,
    side,
    retargeting_share,
    parameters_file='',
):
    """解析单侧配置；缺少默认标定时退回 profile 并由启动层告警。"""
    normalized_model = str(model_id).strip().lower()
    normalized_side = str(side).strip().lower()
    profile = load_model_profile(normalized_model, normalized_side)

    explicit_parameters = str(parameters_file).strip()
    if explicit_parameters:
        selected_parameters = Path(explicit_parameters).expanduser()
        if not selected_parameters.is_file():
            raise FileNotFoundError(
                f'显式指定的重定向配置不存在：{selected_parameters}'
            )
    else:
        selected_parameters = _default_parameters_path(
            retargeting_share,
            normalized_model,
            normalized_side,
        )

    try:
        robot_description = profile.urdf_path.read_text(encoding='utf-8')
    except OSError as error:
        raise FileNotFoundError(
            f'无法读取 {normalized_model}/{normalized_side} URDF：'
            f'{profile.urdf_path}'
        ) from error

    return RvizHandSpec(
        model_id=normalized_model,
        side=normalized_side,
        profile=profile,
        robot_description=robot_description,
        parameters_file=selected_parameters,
        uses_profile_defaults=selected_parameters is None,
    )
