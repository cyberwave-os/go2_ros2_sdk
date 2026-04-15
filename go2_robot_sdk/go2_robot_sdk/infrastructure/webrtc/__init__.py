"""
WebRTC infrastructure adapters
"""
from .webrtc_adapter import WebRTCAdapter
from .go2_connection import Go2Connection, Go2ConnectionError
from .h264_optimization import (
    install_h264_patches,
    set_encoded_frame_tap,
    clear_encoded_frame_tap,
)
from .http_client import HttpClient, WebRTCHttpError, make_local_request
from .data_decoder import WebRTCDataDecoder, DataDecodingError, deal_array_buffer
from .crypto import CryptoUtils, ValidationCrypto, PathCalculator, EncryptionError

__all__ = [
    'WebRTCAdapter', 'Go2Connection', 'Go2ConnectionError',
    'install_h264_patches', 'set_encoded_frame_tap', 'clear_encoded_frame_tap',
    'HttpClient', 'WebRTCHttpError', 'make_local_request',
    'WebRTCDataDecoder', 'DataDecodingError', 'deal_array_buffer',
    'CryptoUtils', 'ValidationCrypto', 'PathCalculator', 'EncryptionError'
] 