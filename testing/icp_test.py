import math
import numpy as np
import time

from map.icp import icp, evaluate_alignment


# An asymmetric shape gives the matcher features in more than one direction.
REFERENCE_SHAPE = np.array([
    [0.0, 0.0], [0.2, 0.0], [0.4, 0.0], [0.6, 0.0],
    [0.8, 0.0], [1.0, 0.0], [0.0, 0.2], [0.0, 0.4],
    [0.0, 0.6], [0.0, 0.8], [0.2, 0.8], [0.4, 0.8],
    [0.6, 0.8], [0.6, 0.6], [0.6, 0.4],
])

# Change or add a case here to repeat the same experiment quickly.
# Noise is the standard deviation in meters, applied independently to each scan.
CASES = {
    "perfect": {"noise": 0.0, "missing": 0, "extra": 0},
    "noisy": {"noise": 0.01, "missing": 0, "extra": 0},
    "partial_and_extra": {"noise": 0.01, "missing": 2, "extra": 3},
}


def make_scans(case, seed=42):
    """Create independently noisy observations of the same underlying shape."""
    rng = np.random.default_rng(seed)
    angle = math.radians(3.0)
    rotation = np.array([
        [math.cos(angle), -math.sin(angle)],
        [math.sin(angle), math.cos(angle)],
    ])
    translation = np.array([0.08, 0.04])

    reference = REFERENCE_SHAPE + rng.normal(0, case["noise"], REFERENCE_SHAPE.shape)
    moving = REFERENCE_SHAPE @ rotation.T + translation
    moving += rng.normal(0, case["noise"], moving.shape)

    # Remove real observations and add unrelated points in the moving frame.
    if case["missing"]:
        moving = moving[:-case["missing"]]
    if case["extra"]:
        extra_points = np.array([[0.45, 0.15], [0.95, 0.20], [1.20, 0.70]])
        moving = np.vstack((moving, extra_points[:case["extra"]]))

    # This inverse maps the ideal moving scan back into the reference frame.
    expected = np.eye(3)
    expected[:2, :2] = rotation.T
    expected[:2, 2] = -rotation.T @ translation
    return reference, moving, expected


def run_case(name, case):
    reference, moving, expected = make_scans(case)

    start_time = time.perf_counter()
    history, aligned = icp(reference, moving, distance_threshold=0.3,
                           point_pairs_threshold=10, verbose=True)
    print(f"\n{name}: reference={len(reference)}, moving={len(moving)}")
    if not history:
        print("  No transform was produced; cannot evaluate pose error.")
        return

    # see time elapsed
    elapsed_seconds = time.perf_counter() - start_time
    print(f"  ICP time: {elapsed_seconds * 1000:.2f} ms")

    estimated = np.eye(3)
    for step in history:
        step_homogeneous = np.eye(3)
        step_homogeneous[:2, :] = step
        estimated = step_homogeneous @ estimated

    translation_error = np.linalg.norm(estimated[:2, 2] - expected[:2, 2])
    estimated_angle = math.atan2(estimated[1, 0], estimated[0, 0])
    expected_angle = math.atan2(expected[1, 0], expected[0, 0])
    angle_error = math.atan2(math.sin(estimated_angle - expected_angle),
                             math.cos(estimated_angle - expected_angle))
    quality = evaluate_alignment(reference, aligned, 0.3)
    print(f"  Translation error: {translation_error:.4f} m")
    print(f"  Rotation error:    {abs(math.degrees(angle_error)):.3f} degrees")
    print(f"  Final alignment:   {quality}")

if __name__ == "__main__":
    for name, case in CASES.items():
        run_case(name, case)
