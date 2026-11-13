"""Explicit sensor modes shared by GUI shells and isolated ORB workers."""

STEREO = "stereo"
STEREO_INERTIAL = "stereo_inertial"
HYBRID = "hybrid"
DEFAULT_MODE = STEREO_INERTIAL


def validate_mode(mode):
    if mode not in (STEREO, STEREO_INERTIAL, HYBRID):
        raise ValueError(f"未知定位模式：{mode}")
    return mode


def apply_mode_environment(environment, mode):
    """Return a child-only environment; never alter the GUI's process environment."""
    mode = validate_mode(mode)
    environment = dict(environment)
    environment["EGOHAND_VIO_STEREO_ONLY"] = "1" if mode == STEREO else "0"
    if mode == HYBRID:
        from pathlib import Path
        library=Path(__file__).resolve().parent/'native'/'bin'/'egohand_vio_visual_bootstrap.dll'
        if not library.is_file():
            raise RuntimeError('混合模式专用桥接库尚未构建，不能用普通VIO冒充视觉先行模式')
        environment['EGOHAND_VIO_LIBRARY']=str(library)
    if mode == STEREO:
        environment["EGOHAND_VIO_DISABLE_RECOVERY"] = "1"
    return environment
