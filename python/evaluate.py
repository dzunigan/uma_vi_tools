# Copyright (C) 2019  Alberto Jaenal <alberto.jaenal at uma.es>

import numpy as np
import argparse
from math import ceil


def decimate_trayectory(sequence, offset, skip, max_length):
    '''
    Decimates the trayectory under the parameters
    :param sequence: evaluation sequence (list)
    :param offset: offset for the evaluation sequence
    :param skip: step to decimate the evaluation sequence (0 steps to 1)
    :param max_length: maximum length of the decimated sequence
    :return: decimated sequence
    '''
    if max_length <= 0:
        max_length = len(sequence)
    if offset >= len(sequence):
        return sequence

    n = min(max_length, ceil((len(sequence) - offset) / (skip + 1)))

    decimated_sequence = list()
    for i in range(0, n):
        decimated_sequence.append(sequence[offset + i * (skip + 1)])

    return decimated_sequence


def obtain_matches(timestamps_gt, timestamps_eval, time_tolerance):
    '''
    Matches two sequences of timestamps
    :param timestamps_gt: timestamps of the ground truth sequence
    :param timestamps_eval: timestamps of the evaluation sequence
    :param time_tolerance: maximum tolerance in which a match is accepted
    :return: the matched evaluation and ground truth sequences timestamps
    '''
    indexes, tgt, teval = list(), list(timestamps_gt), list(timestamps_eval)
    # Compute the possible matches (within the time tolerance) and sort them by their difference
    pre_matches = sorted([(abs(tg - te), tg, te) for tg in timestamps_gt
                          for te in timestamps_eval if abs(tg - te) < time_tolerance])
    # Evaluate the correct matches, by excluding possible repetitions
    for diff, tg, te in pre_matches:
        if (te in teval) and (tg in tgt):
            indexes.append((timestamps_gt.index(tg), timestamps_eval.index(te)))
            tgt.remove(tg)
            teval.remove(te)
    return [list(t) for t in zip(*sorted(indexes))]


def umeyama(poses, poses_gt, scale=False):
    '''
    Compute the Umeyama (rigid) transformation between two trayectories
    :param poses: evaluation positions
    :param poses_gt: ground truth positions
    :param scale: whether to perform the scale (SIM(3))
    :return: the transformation matrix
    '''
    Tout = np.eye(4)
    poses_mean = poses.mean(1, keepdims=True)
    poses_gt_mean = poses_gt.mean(1, keepdims=True)
    poses_demean = poses - poses_mean

    sigma = np.matmul(poses_gt - poses_gt_mean, poses_demean.T) / poses.shape[1]
    poses_var = np.sum(poses_demean * poses_demean, axis=0).mean()

    U, D, V = np.linalg.svd(sigma, full_matrices=True, compute_uv=True)

    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(V) < 0:
        S[2, 2] = -1

    Tout[:3, :3] = np.matmul(np.matmul(U, S), V)
    Tout[:3, 3] = poses_gt_mean.squeeze()

    if scale:
        s = (np.dot(D, S.diagonal())).sum() / poses_var
        Tout[:3, 3] -= s * np.dot(Tout[:3, :3], poses_mean.squeeze())
        Tout[:3, :3] *= s
    else:
        Tout[:3, 3] -= np.dot(Tout[:3, :3], poses_mean.squeeze())

    return Tout


def invert(T, scale=False):
    '''
    Performs the inversion of a transformation matrix
    :param T: transformation matrix
    :param scale: indicate if the matrix is in SE(3) or in SIM(3)
    :return: the inverted matrix
    '''
    Tinv = np.eye(4)
    if scale:
        # SIM(3)
        s = np.power(np.linalg.det(T[:3, :3]), 1 / 3)  # det(T) = s^3
        Tinv[:3, :3] = T[:3, :3].T / s
        Tinv[:3, 3] = - np.dot(Tinv[:3, :3], T[:3, 3]) / s
    else:
        # SE(3)
        Tinv[:3, :3] = T[:3, :3].T
        Tinv[:3, 3] = - np.dot(Tinv[:3, :3], T[:3, 3])
    return Tinv


def obtain_measure(T, scale=False):
    '''
    Obtain the translational norm and the rotation angle from a transformation matrix
    :param T: transformation matrix
    :param scale: indicate if the matrix is in SE(3) or in SIM(3)
    '''
    dist = np.linalg.norm(T[:3, 3])
    if scale:
        s = np.power(np.linalg.det(T[:3, :3]), 1 / 3)  # det(T) = s^3
    else:
        s = 1
    angle = np.arccos(min(1, max(-1, (np.trace(T[:3, :3] / s) - 1) / 2)))
    return dist, angle


def format(number):
    '''
    Output formatting for a 4 decimal precision
    '''
    return '{:.4f}'.format(number)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('gt_file', help='file with the ground truth measures')
    parser.add_argument('evaluation_file', help='file with the evaluated measures')
    parser.add_argument('--scale', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--time_offset',
                        help='time offset (in nanosecods) added to the timestamps of the second file (default: 0 ns)',
                        default=0, type=int)
    parser.add_argument('--max_difference',
                        help='maximally allowed time difference (in nanoseconds) for matching entries (default: 20000000 ns)',
                        default=20000000, type=int)
    parser.add_argument('-o', '--offset', help='offset to decimate the evaluation sequence (default: 0)', default=0,
                        type=int)
    parser.add_argument('-s', '--skip', help='skip to decimate the evaluation sequence, used as skip + 1 (default: 0)',
                        default=0, type=int)
    parser.add_argument('-n', '--num_samples',
                        help='maximum number of samples to decimate the evaluation sequence (default: none)', default=0,
                        type=int)
    parser.add_argument('--plot', help='plot the result to a file (output format: png)', action='store_true')
    args = parser.parse_args()

    # Read and parse the evaluation file
    print()
    print('Parsing formats...')
    if '.csv' in args.evaluation_file:
        print('\t-The evaluation file is in EUROC format (ns)')
        data = np.loadtxt(args.evaluation_file, delimiter=",", comments="#")
        data[:, 0] /= 1E9
    elif '.txt' in args.evaluation_file:
        print('\t-The evaluation file is in TUM format (s)')
        data = np.loadtxt(args.evaluation_file)
    timestamps = data[:, 0] + args.time_offset / 1E9
    posesquat = data[:, 1:].T

    # Decimate the evaluation sequence
    decimate_indexes = decimate_trayectory(range(len(timestamps)), args.offset, args.skip, args.num_samples)
    timestamps = timestamps[decimate_indexes]
    posesquat = posesquat[:, decimate_indexes]

    # Read and parse the ground truth file
    if '.csv' in args.gt_file:
        print('\t-The ground truth file is in EUROC format (ns)')
        data_gt = np.loadtxt(args.gt_file, delimiter=",", comments="#")
        data_gt[:, 0] /= 1E9
    elif '.txt' in args.gt_file:
        print('\t-The ground truth file is in TUM format (s)')
        data_gt = np.loadtxt(args.gt_file)

    # Calculate the breakpoint index, in which the sequence is divided in start and end segments
    scindere = np.argmax(np.diff(data_gt[:, 0])) + 1
    timestamps_start = data_gt[:scindere, 0]
    timestamps_end = data_gt[scindere:, 0]
    posesquat_end = data_gt[scindere:, 1:].T
    posesquat_start = data_gt[:scindere, 1:].T

    # Obtain the matches between the start and end segments and the decimated evaluation segment
    ind_start_gt, ind_start_ev = obtain_matches(list(timestamps_start), list(timestamps), args.max_difference / 1E9)
    ind_end_gt, ind_end_ev = obtain_matches(list(timestamps_end), list(timestamps), args.max_difference / 1E9)

    # Obtain the transformation matrices between the segments through Umeyama
    T_start = umeyama(posesquat[:, ind_start_ev][:3, :], posesquat_start[:3, ind_start_gt], scale=args.scale)
    T_end = umeyama(posesquat[:, ind_end_ev][:3, :], posesquat_end[:3, ind_end_gt], scale=args.scale)

    # Obtain the drift transformation matrix and the measure drifts
    T_drift = np.matmul(T_end, invert(T_start, scale=args.scale))
    drift = obtain_measure(T_drift, scale=args.scale)

    # Obtain the alignment error
    poses = np.ones([4, posesquat.shape[1]])
    poses[:3, :] = posesquat[:3, :]
    align_error = np.linalg.norm(np.matmul(T_start, poses)[:3, :] - np.matmul(T_end, poses)[:3, :], axis=0)

    print('\nResults:')

    if args.scale:
        print('\tScale: ', np.power(np.linalg.det(T_start[:3, :3]), 1 / 3))
        print()

    print('\tStart segments count: ', len(ind_start_ev))
    print('\tEnd segments count: ', len(ind_end_ev))
    print('\tTranslational drift: ', format(drift[0]), 'm')
    print('\tRotational drift: ', format(180.0 / np.pi * drift[1]), 'deg')
    print()
    print('\tTotal segments count: ', len(timestamps))
    print('\tTranslational alignment error RMSE: ', format(np.sqrt(np.mean(align_error ** 2))), 'm')
    print('\tTranslational alignment error mean: ', format(np.mean(align_error)), 'm')
    print('\tTranslational alignment error median: ', format(np.median(align_error)), 'm')
    print('\tTranslational alignment error stdev: ', format(np.std(align_error)), 'm')
    print('\tTranslational alignment error min: ', format(np.min(align_error)), 'm')
    print('\tTranslational alignment error max: ', format(np.max(align_error)), 'm')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig = plt.figure()
        plt.plot(timestamps - timestamps[0], align_error, '-', color="red")
        plt.xlabel('time [s]')
        plt.ylabel('translational error [m]')
        output_name = args.evaluation_file.replace('.txt', '.png') if '.txt' in args.evaluation_file else args.evaluation_file.replace('.csv', '.png')
        print()
        print('Saving error plot in:\n ', output_name)
        plt.savefig(output_name, dpi=300)