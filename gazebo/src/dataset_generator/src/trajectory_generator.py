import sys
import numpy as np
from math import pi


def parse_args(argv):
    import argparse

    common = argparse.ArgumentParser(description='Trajectory generator.', add_help=False)
    common.add_argument('-s', '--seed', type=int, nargs='?', default=42,
                        help='random seed')
    common.add_argument('-o', '--output', nargs='?',
                        help='output trajectory path')

    parser = argparse.ArgumentParser(parents=[common])
    subparsers = parser.add_subparsers(dest='mode')

    ref_parser = subparsers.add_parser('reference', parents=[common],
                                       help='similar trajectory based on reference')
    ref_parser.add_argument('reference', help='reference trajectory path')
    ref_parser.add_argument('-d', '--delta', type=float, nargs='?', default=1,
                            help='fluctuation radius (max difference of each point with reference)')

    curve_parser = subparsers.add_parser('curve', parents=[common],
                                         help='closed curve trajectory in a given range (annulus)')
    curve_parser.add_argument('r', type=float, help='inner circle radius')
    curve_parser.add_argument('R', type=float, help='outer circle radius')
    curve_parser.add_argument('-c', '--center', type=float, nargs='*', default=[0, 0, 0],
                              help='center 3D coordinates')
    curve_parser.add_argument('-l', '--length', type=int, nargs='?', default=3,
                              help='number of points in trajectory')
    curve_parser.add_argument('-d', '--delta', type=float, nargs='?', default=0,
                              help='angle fluctuation radius (degrees)')

    args = parser.parse_args(args=argv)

    if args.mode == 'curve' and len(args.center) != 3:
        parser.error('Expected 3 center coordinates')

    return args


def print_trajectory(trajectory, file=sys.stdout):
    for point in trajectory:
        print(' '.join(map(str, point)), file=file)


def main(args):
    np.random.seed(args.seed)

    if args.mode == 'reference':
        trajectory = reference_trajectory(args.reference, args.delta)
    else:
        trajectory = curve_trajectory(args.r, args.R, args.center, args.length, args.delta)

    if args.output is None:
        print_trajectory(trajectory)
    else:
        with open(args.output, 'w') as file:
            print_trajectory(trajectory, file)


def reference_trajectory(reference_path, delta):
    """Generate similar trajectory based on reference."""

    trajectory = []
    for line in open(reference_path):
        reference = list(map(float, line.split()))
        point = [x + np.random.uniform(-delta, delta) for x in reference]
        trajectory.append(point)
    return trajectory


def closed_curve(length, num=10):
    """Generate random closed curve and return `length` samples of angle and radius."""

    angle = np.linspace(0, 2 * pi, num=length, endpoint=False)
    amplitude = np.random.rand(num) * np.logspace(-0.5, -2.5, num)
    phase = np.random.rand(num) * 2 * pi
    radius = np.ones_like(angle)
    for n in range(num):
        radius += amplitude[n] * np.sin(n * angle + phase[n])
    return angle, radius


def curve_trajectory(r, R, center, trajectory_length, delta):
    """Generate closed curve trajectory in a given range of distance [r, R] from center."""

    # Generate random closed curve
    angle, radius = closed_curve(trajectory_length)
    _, z = closed_curve(trajectory_length)
    x = radius * np.cos(angle)
    y = radius * np.sin(angle)

    # Scale curve to desired radius
    xyz = np.vstack((x, y, z))
    distance = np.linalg.norm(xyz, axis=0)
    min_distance = np.min(distance)
    max_distance = np.max(distance)
    middle = (min_distance + max_distance) / 2
    scaled_middle = (r + R) / 2
    radius_scale = (R - r) / (max_distance - min_distance)
    scaled_distance = (distance - middle) * radius_scale + scaled_middle
    xyz *= scaled_distance / distance

    # Translate all points to the given center
    xyz += np.array(center)
    x, y, z = np.vsplit(xyz, 3)
    x, y, z = x.flatten(), y.flatten(), z.flatten()

    # import matplotlib.pyplot as plt
    # from mpl_toolkits.mplot3d import Axes3D
    # fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.scatter3D(x, y, z)
    # plt.show()

    # Calculate angles looking in the center with random noise
    roll = np.zeros_like(x) + np.random.uniform(-delta, delta, x.shape)
    yaw = np.degrees(np.arctan2(-y, -x)) + np.random.uniform(-delta, delta, x.shape)
    pitch = np.degrees(np.arcsin(z / scaled_distance)) + np.random.uniform(-delta, delta, x.shape)

    return list(zip(x, y, z, roll, pitch, yaw))


if __name__ == '__main__':
    argv = sys.argv[1:]
    main(parse_args(argv))
