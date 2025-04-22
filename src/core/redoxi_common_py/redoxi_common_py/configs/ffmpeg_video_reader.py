try:
    from attrs import define, field, asdict
    import attrs.validators as av
except ImportError:
    from attr import define, field, asdict
    import attr.validators as av

# from redoxi_common_py.configs.base_node import *

from redoxi_common_py.configs.video_reader_base import *
import redoxi_common_py.configs.video_reader_base as videoReaderBaseCfg

from typing import Any, Literal, ClassVar

__all__ = [
    "FFmpegVideoReaderInitConfig",
    "FFmpegVideoReaderRuntimeConfig",
    "FFmpegVideoReaderNodeConfig",
] + videoReaderBaseCfg.__all__


@define(kw_only=True)
class FFmpegVideoReaderInitConfig(videoReaderBaseCfg.VideoReaderBaseInitConfig):
    ffmpeg_path: str = field(default="/usr/bin/ffmpeg")


@define(kw_only=True)
class FFmpegVideoReaderRuntimeConfig(videoReaderBaseCfg.VideoReaderBaseRuntimeConfig):
    ffmpeg_args: list[str] = field()

    # input frame data format
    frame_width: int = field()
    frame_height: int = field()
    frame_channels: int = field()
    frame_encoding: str | None = field(default=None)


@define(kw_only=True)
class FFmpegVideoReaderNodeConfig(videoReaderBaseCfg.BaseRosNodeConfig):
    # init config has no default, must be set explicitly
    init_config: FFmpegVideoReaderInitConfig = field()

    # runtime config follows base default
    runtime_config: FFmpegVideoReaderRuntimeConfig = field(
        factory=FFmpegVideoReaderRuntimeConfig
    )
