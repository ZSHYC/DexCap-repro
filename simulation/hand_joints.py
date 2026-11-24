LEAP_SWAP = [1, 0, 2, 3, 5, 4, 6, 7, 9, 8, 10, 11, 12, 13, 14, 15]
LEFT_MIRRORED = [0, 4, 8, 12, 13]


def decode_leap_joints(scaled, left):
    joints = 2.0 * scaled
    if left:
        joints[LEFT_MIRRORED] *= -1
    return joints[LEAP_SWAP]


def encode_leap_joints(joints, left):
    scaled = 0.5 * joints[LEAP_SWAP]
    if left:
        scaled[LEFT_MIRRORED] *= -1
    return scaled
