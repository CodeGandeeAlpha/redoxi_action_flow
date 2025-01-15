try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

from redoxi_common_py.configs.video_source_from_url import *
import redoxi_common_py.configs.video_source_from_url as videoSrcCfg

__all__ = [
    "VideoReaderWithCropInitConfig",
    "VideoReaderWithCropRuntimeConfig",
    "VideoReaderWithCropNodeConfig",
] + videoSrcCfg.__all__


@define(kw_only=True)
class VideoReaderWithCropInitConfig(videoSrcCfg.VideoSourceFromUrlInitConfig):
    crop_cfg_path: str | None = field(default=None)


@define(kw_only=True)
class VideoReaderWithCropRuntimeConfig(videoSrcCfg.VideoSourceFromUrlRuntimeConfig):
    pass


@define(kw_only=True)
class VideoReaderWithCropNodeConfig(videoSrcCfg.BaseRosNodeConfig):
    init_config: VideoReaderWithCropInitConfig = field(
        factory=VideoReaderWithCropInitConfig
    )
    runtime_config: VideoReaderWithCropRuntimeConfig = field(
        factory=VideoReaderWithCropRuntimeConfig
    )
