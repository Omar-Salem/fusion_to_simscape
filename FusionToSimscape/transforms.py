"""Rigid transform operations. Matrices are row-major, translations in metres."""
import math


def identity():
    return [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.]


def metres(matrix):
    result = list(matrix)
    for index in (3, 7, 11):
        result[index] *= 0.01
    return result


def multiply(a, b):
    return [sum(a[4*r+k] * b[4*k+c] for k in range(4))
            for r in range(4) for c in range(4)]


def inverse(a):
    result = identity()
    for r in range(3):
        for c in range(3):
            result[4*r+c] = a[4*c+r]
        result[4*r+3] = -sum(a[4*k+r] * a[4*k+3] for k in range(3))
    return result


def point(matrix, xyz):
    return [sum(matrix[4*r+c]*xyz[c] for c in range(3)) + matrix[4*r+3]
            for r in range(3)]


def principal_inertia(moments, axes):
    """Rotate centroidal principal inertias into the component axes (kg m²)."""
    tensor = [[sum(moments[k]*axes[k][r]*axes[k][c] for k in range(3))*1e-4
               for c in range(3)] for r in range(3)]
    # XML uses inertia tensor entries in this exact order, not Fusion's XYZ order.
    return [tensor[0][0], tensor[1][1], tensor[2][2],
            tensor[1][2], tensor[2][0], tensor[0][1]]


def triangle_normal(vertices):
    a = [vertices[1][k]-vertices[0][k] for k in range(3)]
    b = [vertices[2][k]-vertices[0][k] for k in range(3)]
    cross = [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
    length = math.sqrt(sum(v*v for v in cross))
    return [v/length for v in cross] if length else [0., 0., 0.]
