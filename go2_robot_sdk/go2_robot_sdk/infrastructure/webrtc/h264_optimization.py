# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""Runtime monkey-patches for aiortc's H264 decoder.

Replaces build-time Docker patches with import-time modifications:

1. **GPU decoder selection** — probes for NVDEC ``h264_cuvid`` at runtime and
   falls back to software ``h264`` transparently.

2. **Encoded-frame tap** — exposes a callback hook that receives raw H264
   access units *before* PyAV's decode.  External consumers (e.g. a GStreamer
   pipeline) can register a tap to consume the raw H264 and optionally skip
   the software decode entirely.

All patches are idempotent; calling them multiple times is safe.
"""

from __future__ import annotations

import logging
from typing import Callable, Optional

logger = logging.getLogger(__name__)

# Type alias for the tap callback.
# Signature: (data: bytes, timestamp: int) -> bool
#   Return True  → skip PyAV decode (caller will decode externally)
#   Return False → let PyAV decode proceed as usual
H264TapCallback = Callable[[bytes, int], bool]

_patched_gpu = False
_patched_tap = False
_encoded_frame_tap: Optional[H264TapCallback] = None


def patch_h264_gpu_decoder() -> None:
    """Monkey-patch aiortc to prefer h264_cuvid (NVDEC) with software fallback.

    Safe to call even if aiortc or PyAV is not installed — logs a warning and
    returns silently.
    """
    global _patched_gpu
    if _patched_gpu:
        return

    try:
        import av  # type: ignore
        import aiortc.codecs.h264 as h264_mod  # type: ignore
    except ImportError:
        logger.debug("aiortc or PyAV not available; skipping GPU decoder patch")
        return

    def _pick_h264_decoder() -> str:
        if "h264_cuvid" not in av.codecs_available:
            return "h264"
        try:
            probe = av.CodecContext.create("h264_cuvid", "r")
            probe.open()
            probe.close()
            return "h264_cuvid"
        except Exception:
            return "h264"

    H264Decoder = getattr(h264_mod, "H264Decoder", None)
    if H264Decoder is None:
        logger.warning("H264Decoder class not found in aiortc.codecs.h264")
        return

    _orig_init = H264Decoder.__init__

    def _patched_init(self: object) -> None:
        _orig_init(self)
        chosen = _pick_h264_decoder()
        if chosen != "h264":
            self.codec = av.CodecContext.create(chosen, "r")  # type: ignore[attr-defined]
            logger.info("H264Decoder using GPU-accelerated codec: %s", chosen)

    H264Decoder.__init__ = _patched_init
    _patched_gpu = True
    logger.info("Installed H264 GPU decoder selection (runtime patch)")


def patch_h264_encoded_frame_tap() -> None:
    """Monkey-patch aiortc's H264Decoder.decode to call the tap before decode.

    The tap callback is managed via :func:`set_encoded_frame_tap` /
    :func:`clear_encoded_frame_tap`.
    """
    global _patched_tap
    if _patched_tap:
        return

    try:
        import aiortc.codecs.h264 as h264_mod  # type: ignore
    except ImportError:
        logger.debug("aiortc not available; skipping tap patch")
        return

    H264Decoder = getattr(h264_mod, "H264Decoder", None)
    if H264Decoder is None:
        logger.warning("H264Decoder class not found in aiortc.codecs.h264")
        return

    _orig_decode = H264Decoder.decode

    def _patched_decode(self: object, encoded_frame: object) -> list:
        tap = _encoded_frame_tap
        if tap is not None:
            data = getattr(encoded_frame, "data", None)
            ts = getattr(encoded_frame, "timestamp", 0)
            if data is not None and tap(data, ts):
                return []
        return _orig_decode(self, encoded_frame)

    H264Decoder.decode = _patched_decode
    _patched_tap = True
    logger.info("Installed H264 encoded-frame tap (runtime patch)")


def install_h264_patches() -> None:
    """Convenience: apply all H264 optimizations in one call."""
    patch_h264_gpu_decoder()
    patch_h264_encoded_frame_tap()


def set_encoded_frame_tap(callback: H264TapCallback) -> None:
    """Register a callback to receive raw H264 access units before decode.

    Only one tap can be active at a time; calling this replaces any existing
    tap.  The tap patch is installed automatically if not already applied.
    """
    global _encoded_frame_tap
    if not _patched_tap:
        patch_h264_encoded_frame_tap()
    _encoded_frame_tap = callback
    logger.info("H264 encoded-frame tap registered")


def clear_encoded_frame_tap() -> None:
    """Remove the currently registered tap callback."""
    global _encoded_frame_tap
    _encoded_frame_tap = None
    logger.debug("H264 encoded-frame tap cleared")
