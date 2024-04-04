import numpy as np

def float16_to_8bit_bytes(value):
    if not isinstance(value, np.float16):
        raise TypeError("Input must be a numpy float16.")
    int_representation = np.frombuffer(value.tobytes(), dtype=np.uint16)[0]
    high_byte = int_representation >> 8
    low_byte = int_representation & 0xFF
    return (high_byte, low_byte)

def binary_strings_to_float16(high_byte_str, low_byte_str):
    if len(high_byte_str) != 8 or len(low_byte_str) != 8:
        raise ValueError("Both strings must be 8 bits long.")
    high_byte = int(high_byte_str, 2)
    low_byte = int(low_byte_str, 2)
    int_representation = (high_byte << 8) | low_byte
    float_value = np.frombuffer(np.array([int_representation], dtype=np.uint16).tobytes(), dtype=np.float16)[0]
    return float_value