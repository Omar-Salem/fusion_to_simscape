"""Map Fusion motion directions onto Simscape's joint-frame Z axis."""
import math

from .transforms import inverse


def unit(vector):
    length = math.sqrt(sum(value*value for value in vector))
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError('Joint motion axis must be finite and nonzero.')
    return [value/length for value in vector]


def rotate(frame, vector):
    """Rotate a direction without applying the frame's translation."""
    return [sum(frame[4*row+column]*vector[column] for column in range(3))
            for row in range(3)]


def motion_frame(frame, direction, custom_axis=None, native_frame=None):
    """Preserve the captured origin and put the positive motion axis on Z.

    Standard direction indices are Fusion JointDirections X=0, Y=1, Z=2.
    A custom axis is supplied in the native joint owner's coordinates and is
    transported via native and solved proxy joint frames, not nominal parents.
    """
    if direction in (0, 1, 2):
        # A cyclic column permutation preserves handedness and leaves Z unchanged.
        columns = ((1, 2, 0), (2, 0, 1), (0, 1, 2))[direction]
        result = list(frame)
        for row in range(3):
            for column in range(3):
                result[4*row+column] = frame[4*row+columns[column]]
        return result
    if direction != 3 or custom_axis is None or native_frame is None:
        raise ValueError('Custom motion directions require a native axis and joint frame.')
    axis = unit(rotate(frame, rotate(inverse(native_frame), unit(custom_axis))))
    # Retain the original roll where possible; use Y when X is parallel to Z.
    candidates = ([frame[4*row+column] for row in range(3)] for column in (0, 1))
    for candidate in candidates:
        along = sum(a*b for a, b in zip(candidate, axis))
        projected = [candidate[k]-along*axis[k] for k in range(3)]
        if sum(value*value for value in projected) > 1e-12:
            x_axis = unit(projected)
            break
    else:
        raise ValueError('Joint geometry does not define an orthonormal frame.')
    y_axis = [axis[1]*x_axis[2]-axis[2]*x_axis[1],
              axis[2]*x_axis[0]-axis[0]*x_axis[2],
              axis[0]*x_axis[1]-axis[1]*x_axis[0]]
    result = list(frame)
    for row in range(3):
        result[4*row:4*row+3] = [x_axis[row], y_axis[row], axis[row]]
    return result
