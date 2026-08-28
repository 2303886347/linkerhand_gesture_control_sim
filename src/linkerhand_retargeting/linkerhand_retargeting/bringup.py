"""解析多型号运行管线使用的 profile、标定和 URDF。"""

from dataclasses import dataclass
from pathlib import Path

from linkerhand_model_profiles import ModelProfile, load_model_profile


@dataclass(frozen=True)
class RetargetingSpec:
    """单侧重定向管线的型号与标定选择结果。"""

    model_id: str
    side: str
    profile: ModelProfile
    parameters_file: Path | None
    uses_profile_defaults: bool


@dataclass(frozen=True)
class RvizHandSpec:
    """单侧机械手在 RViz 管线中的确定性运行配置。"""

    retargeting: RetargetingSpec
    robot_description: str

    @property
    def model_id(self):
        return self.retargeting.model_id

    @property
    def side(self):
        return self.retargeting.side

    @property
    def profile(self):
        return self.retargeting.profile

    @property
    def parameters_file(self):
        return self.retargeting.parameters_file

    @property
    def uses_profile_defaults(self):
        return self.retargeting.uses_profile_defaults


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


def resolve_retargeting_spec(
    model_id,
    side,
    retargeting_share,
    parameters_file='',
):
    """解析型号和标定；缺少默认 YAML 时允许使用 profile 默认映射。"""
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

    return RetargetingSpec(
        model_id=normalized_model,
        side=normalized_side,
        profile=profile,
        parameters_file=selected_parameters,
        uses_profile_defaults=selected_parameters is None,
    )


def resolve_rviz_hand_spec(
    model_id,
    side,
    retargeting_share,
    parameters_file='',
):
    """在重定向配置上补充 RViz 所需的完整 URDF 文本。"""
    retargeting = resolve_retargeting_spec(
        model_id,
        side,
        retargeting_share,
        parameters_file,
    )
    try:
        robot_description = retargeting.profile.urdf_path.read_text(
            encoding='utf-8'
        )
    except OSError as error:
        raise FileNotFoundError(
            f'无法读取 {retargeting.model_id}/{retargeting.side} URDF：'
            f'{retargeting.profile.urdf_path}'
        ) from error

    return RvizHandSpec(
        retargeting=retargeting,
        robot_description=robot_description,
    )
