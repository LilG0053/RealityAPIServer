# Credit to https://github.com/richardos/icp
import math
import numpy as np
from sklearn.neighbors import NearestNeighbors


def euclidean_distance(point1, point2):
    """
    Euclidean distance between two points.
    :param point1: the first point as a tuple (a_1, a_2, ..., a_n)
    :param point2: the second point as a tuple (b_1, b_2, ..., b_n)
    :return: the Euclidean distance
    """
    a = np.array(point1)
    b = np.array(point2)

    return np.linalg.norm(a - b, ord=2)


def point_based_matching(point_pairs, weights=None):
    """
    This function is based on the paper "Robot Pose Estimation in Unknown Environments by Matching 2D Range Scans"
    by F. Lu and E. Milios.

    :param point_pairs: the matched point pairs [((x1, y1), (x1', y1')), ..., ((xi, yi), (xi', yi')), ...]
    :param weights: one nonnegative weight per pair; defaults to equal weights
    :return: the rotation angle and the 2D translation (x, y) to be applied for matching the given pairs of points
    """

    x_mean = 0
    y_mean = 0
    xp_mean = 0
    yp_mean = 0
    n = len(point_pairs)

    if n == 0:
        return None, None, None

    if weights is None:
        weights = [1.0] * n
    if len(weights) != n or any(not math.isfinite(w) or w < 0 for w in weights):
        raise ValueError('weights must contain one finite, nonnegative value per pair')
    weight_sum = sum(weights)
    if weight_sum <= 1e-12:
        return None, None, None

    for pair, weight in zip(point_pairs, weights):

        (x, y), (xp, yp) = pair

        x_mean += weight * x
        y_mean += weight * y
        xp_mean += weight * xp
        yp_mean += weight * yp

    x_mean /= weight_sum
    y_mean /= weight_sum
    xp_mean /= weight_sum
    yp_mean /= weight_sum

    s_x_xp = 0
    s_y_yp = 0
    s_x_yp = 0
    s_y_xp = 0
    for pair, weight in zip(point_pairs, weights):

        (x, y), (xp, yp) = pair

        s_x_xp += weight * (x - x_mean)*(xp - xp_mean)
        s_y_yp += weight * (y - y_mean)*(yp - yp_mean)
        s_x_yp += weight * (x - x_mean)*(yp - yp_mean)
        s_y_xp += weight * (y - y_mean)*(xp - xp_mean)

    rot_angle = math.atan2(s_x_yp - s_y_xp, s_x_xp + s_y_yp)
    translation_x = xp_mean - (x_mean*math.cos(rot_angle) - y_mean*math.sin(rot_angle))
    translation_y = yp_mean - (x_mean*math.sin(rot_angle) + y_mean*math.cos(rot_angle))

    return rot_angle, translation_x, translation_y

# robust weighting decreases the weights of items that are further away
def robust_weight(distance, kernel_distance):
    scale_squared = kernel_distance ** 2

    return (scale_squared / (scale_squared + distance ** 2)) ** 2

def icp(reference_points, points, max_iterations=100, distance_threshold=0.3, convergence_translation_threshold=1e-3,
        convergence_rotation_threshold=1e-4, point_pairs_threshold=10, kernel_distance=.1, verbose=False):
    """
    An implementation of the Iterative Closest Point algorithm that matches a set of M 2D points to another set
    of N 2D (reference) points.

    :param reference_points: the reference point set as a numpy array (N x 2)
    :param points: the point that should be aligned to the reference_points set as a numpy array (M x 2)
    :param max_iterations: the maximum number of iteration to be executed
    :param distance_threshold: the distance threshold between two points in order to be considered as a pair
    :param convergence_translation_threshold: the threshold for the translation parameters (x and y) for the
                                              transformation to be considered converged
    :param convergence_rotation_threshold: the threshold for the rotation angle (in rad) for the transformation
                                               to be considered converged
    :param point_pairs_threshold: the minimum number of point pairs the should exist
    :param kernel_distance: positive distance scale for robust correspondence weights
    :param verbose: whether to print informative messages about the process (default: False)
    :return: the transformation history as a list of numpy arrays containing the rotation (R) and translation (T)
             transformation in each iteration in the format [R | T] and the aligned points as a numpy array M x 2
    """

    if not math.isfinite(kernel_distance) or kernel_distance <= 0:
        raise ValueError('kernel_distance must be finite and positive')

    transformation_history = []

    # creates NearestNeighbors object search tool: given a new point, informs the closet to reference_points
    nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(reference_points)

    for iter_num in range(max_iterations):
        if verbose:
            print('------ iteration', iter_num, '------')

        closest_point_pairs = []  # list of point correspondences for closest point rule
        weights = []
        
        # distances: distance from the closet point
        # indices: which index from reference_points is the closet
        distances, indices = nbrs.kneighbors(points) 
        for nn_index in range(len(distances)):
            if distances[nn_index][0] < distance_threshold:
                # returns source point, reference point
                closest_point_pairs.append((points[nn_index], reference_points[indices[nn_index][0]]))
                weights.append(robust_weight(distances[nn_index][0], kernel_distance))

        # if only few point pairs, stop process
        if verbose:
            print('number of pairs found:', len(closest_point_pairs))
        if len(closest_point_pairs) < point_pairs_threshold:
            if verbose:
                print('Very few points close to each other.')
            break

        # trim or robustweighting and degeneracy before computing
        

        # compute translation and rotation using point correspondences
        closest_rot_angle, closest_translation_x, closest_translation_y = point_based_matching(closest_point_pairs, weights)
        if closest_rot_angle is not None:
            if verbose:
                print('Rotation:', math.degrees(closest_rot_angle), 'degrees')
                print('Translation:', closest_translation_x, closest_translation_y)
        if closest_rot_angle is None or closest_translation_x is None or closest_translation_y is None:
            if verbose:
                print('No better solution can be found!')
            break

        # transform 'points' (using the calculated rotation and translation)
        c, s = math.cos(closest_rot_angle), math.sin(closest_rot_angle)
        rot = np.array([[c, -s],
                        [s, c]])
        aligned_points = np.dot(points, rot.T)
        aligned_points[:, 0] += closest_translation_x
        aligned_points[:, 1] += closest_translation_y

        # update 'points' for the next iteration
        points = aligned_points

        # update transformation history
        transformation_history.append(np.hstack((rot, np.array([[closest_translation_x], [closest_translation_y]]))))

        # check convergence
        if (abs(closest_rot_angle) < convergence_rotation_threshold) \
                and (abs(closest_translation_x) < convergence_translation_threshold) \
                and (abs(closest_translation_y) < convergence_translation_threshold):
            if verbose:
                print('Converged!')
            break

    return transformation_history, points

# check for fitness (inlier ratio), inlier RSME (of those points within range, what is their distance squared)
# compare the "points" with initial reference
def evaluate_alignment(reference_points, aligned_points, distance_threshold):
    """
    Measure how well an aligned source scan matches a reference scan.

    Fitness is the fraction of aligned source points whose nearest reference
    point is strictly closer than distance_threshold.

    Distances and RMSE use the same units as the input coordinates.
    """
    if len(reference_points) == 0 or len(aligned_points) == 0:
        return {
            "inlier_count": 0,
            "source_count": len(aligned_points),
            "fitness": 0.0,
            "inlier_rmse": None,
        }

    # Recompute correspondences using the FINAL aligned points.
    nbrs = NearestNeighbors(n_neighbors=1, algorithm="kd_tree")
    nbrs.fit(reference_points)

    distances, _ = nbrs.kneighbors(aligned_points)

    # kneighbors returns shape (number_of_source_points, 1).
    # Select the first and only neighbor distance for each point.
    distances = distances[:, 0]

    inlier_mask = distances < distance_threshold
    inlier_distances = distances[inlier_mask]

    inlier_count = len(inlier_distances)
    source_count = len(aligned_points)
    fitness = inlier_count / source_count

    if inlier_count > 0:
        inlier_rmse = np.sqrt(np.mean(inlier_distances ** 2))
    else:
        # There is no meaningful RMSE when there are no accepted pairs.
        inlier_rmse = None\

    # we could also add code for the shape of the graph
    # if the lidar scan gives us one straight line, the RSME is low but we don't know if it accurately tracked movement

    return {
        "inlier_count": inlier_count,
        "source_count": source_count,
        "fitness": fitness,
        "inlier_rmse": inlier_rmse,
    }
