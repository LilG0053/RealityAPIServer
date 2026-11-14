from dataclasses import dataclass, field
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from map.icp import icp, evaluate_alignment
import logging

logger = logging.getLogger(__name__)

@dataclass
class PoseNode:
    id: int
    lidar_points: np.ndarray
    robot_transform: np.ndarray

@dataclass
class PoseEdge:
    from_id: int
    to_id: int
    estimated_transform: np.ndarray
    confidence: int
    
class PoseGraph:
    def __init__(self):
        self.nodes = []
        self.edges = []
        # converts from the localizer to updated values in the graph
        self.localizer_to_graph = np.eye(3)

    def add_node(self, lidar_points: np.ndarray, robot_transform: np.ndarray):
        robot_graph_pose = self.localizer_to_graph @ robot_transform
        new_node = PoseNode(id=len(self.nodes), lidar_points=lidar_points, robot_transform=robot_graph_pose)
        self.nodes.append(new_node)

        # if more than one node exists, add an edge to connect new node to graph
        if len(self.nodes) > 1:
            previous_node = self.nodes[new_node.id - 1]
            
            relative_transform = (
                np.linalg.inv(previous_node.robot_transform)
                @ new_node.robot_transform
            )

            self.add_edge(
                from_id=previous_node.id,
                to_id=new_node.id,
                estimated_transform=relative_transform,
                confidence=1,  # Placeholder weight for now
            )

    def add_edge(self, from_id: int, to_id: int, estimated_transform: np.ndarray, confidence: int):
        edge = PoseEdge(from_id=from_id, to_id=to_id, estimated_transform=estimated_transform, confidence=confidence)
        self.edges.append(edge)

    def check_closure(self, node: PoseNode):
        index_list = self.find_candidates(node, 1, 5)

        best_match = self.confirm_candidate(index_list, node.lidar_points, 0.3)
        if (best_match != None):
            logger.debug("Candidates found, performing loop closure")
            # not sure what all these transforms are doing
            corrected_current_pose = best_match["correction"] @ node.robot_transform
            matching_candidate = self.nodes[best_match["index"]].robot_transform

            # transform from the source node to the candidate node
            relative_pose = (
                np.linalg.inv(node.robot_transform)
                @ best_match["correction"]
                @ matching_candidate
            )
            
            self.add_edge(node.id, best_match["index"], relative_pose, 1)
            # call function to add loop-closure
            self.closure_algorithm()
        else:
            logger.debug("No candidates found, not doing loop closure")

    def measured_edges(self,):
        # initial guess, [x1, y1, theta1, ...] of the existing values from the transform
        x0 = []
        for edge in self.edges:
            x = edge.estimated_transform[0, 2]
            y = edge.estimated_transform[1, 2]
            theta = np.arctan2(
                edge.estimated_transform[1, 0],
                edge.estimated_transform[0, 0],
            )
            x0.extend([x, y, theta])

        return x0

    def initial_nodes(self,):
        # initial guess, [x1, y1, theta1, ...] of the existing values from the transform
        x0 = []
        for node in self.nodes:
            x = node.robot_transform[0, 2]
            y = node.robot_transform[1, 2]
            theta = np.arctan2(
                node.robot_transform[1, 0],
                node.robot_transform[0, 0],
            )
            x0.extend([x, y, theta])

        return x0

    def get_predicted_edge_pose(self, pose_values):
        predicted_relative_pose = []
        for edge in self.edges:
            # This assumes node IDs match their indices in self.nodes.
            from_index = edge.from_id * 3
            to_index = edge.to_id * 3

            x_i, y_i, theta_i = pose_values[from_index : from_index + 3]
            x_j, y_j, theta_j = pose_values[to_index : to_index + 3]

            # Difference in server coordinates.
            dx = x_j - x_i
            dy = y_j - y_i

            # Express node j's position in node i's frame.
            predicted_x =  np.cos(theta_i) * dx + np.sin(theta_i) * dy
            predicted_y = -np.sin(theta_i) * dx + np.cos(theta_i) * dy
            predicted_theta = theta_j - theta_i

            # returns the relative x, y, theta change between two nodes
            predicted_relative_pose.extend([
                predicted_x,
                predicted_y,
                predicted_theta,
            ])

        return predicted_relative_pose

    def update_node_transform(self, result):
        for node in self.nodes:
            # don't use the first node because it stays fixed
            if (node.id == 0):
                continue

            # turns the result array [x_1, y_1, theta_1] into a transform
            index = (node.id - 1) * 3
            x, y, theta = result.x[index : index + 3]
            c = np.cos(theta)
            s = np.sin(theta)

            new_transform = np.array([
                [c, -s, x],
                [s,  c, y],
                [0,  0, 1],
            ])

            # updates each nodes transform
            old_transform = node.robot_transform
            node.robot_transform = new_transform

            # measure the distance between that transform
            transform_change = new_transform @ np.linalg.inv(old_transform)

            # need to also change each lidar scans position based on the transform, this needs to connect to the lidar images
            rotation_change = transform_change[:2, :2]
            translation_change = transform_change[:2, 2]

            node.lidar_points = node.lidar_points @ rotation_change.T + translation_change
            
            # need to also send the updated transform back to localizer

    def closure_algorithm(self,):
        # measured_edges is array of transforms between vertices that stays fixed
        meausred_edge_pose = self.measured_edges()
        x0 = self.initial_nodes()

        # pose_values contians the least_squares guess as array [x1, y1, theta1, ...]
        def residuals(pose_values):
            # pose values doesn't contain the first node so we add that
            all_poses = np.concatenate((x0[:3], pose_values))

            predicted_edge_pose = self.get_predicted_edge_pose(all_poses)

            # array of errors from predicted movement between nodes vs. measured movement between nodes
            errors = np.asarray(predicted_edge_pose) - np.asarray(meausred_edge_pose)

            # Wrap every theta difference to [-pi, pi].
            angle_errors = errors[2::3]
            errors[2::3] = np.arctan2(np.sin(angle_errors), np.cos(angle_errors))

            return errors

        # run least squares which uses residual function and takes all node values (remove first three because x0, y0, theta0 is fixed)
        result = least_squares(residuals, x0[3:])

        if result.success:
            self.update_node_transform(result)


    # checking for possible loop closures by looking for poses that were nearby
    def find_candidates(self, node: PoseNode, search_radius: float, exclude_recent: int = 10):
        # gets the robot positions
        if not self.nodes:
            return []
        robot_positions = np.array([
            node.robot_transform[:2, 2]
            for node in self.nodes
        ])

        tree = cKDTree(robot_positions)

        # gets indicies of nodes 
        indicies = tree.query_ball_point(node.robot_transform[:2, 2], search_radius)

        # filter out current node and the most recent "exclude_recent" nodes
        filtered_indices = [
            i for i in indicies
            if i < node.id and node.id - i >= exclude_recent
        ]
        return filtered_indices

    def confirm_candidate(self, indices, reference_points: np.ndarray, threshold: float,
                          *, min_inliers: int = 10, min_fitness: float = 0.3,
                          max_rmse: float = 0.05):
        """Return the best passing candidate, or None when no match passes.

        Scans must be 2D points in the same server-coordinate plane. ICP maps
        the candidate scan (source) onto reference_points (current scan).
        """
        if len(reference_points) == 0:
            return None

        best_match = None
        for index in indices:
            candidate_points = self.nodes[index].lidar_points
            if len(candidate_points) == 0:
                continue

            history, aligned_points = icp(reference_points, candidate_points)
            if not history:
                continue

            metrics = evaluate_alignment(reference_points, aligned_points, threshold)

            if metrics["inlier_count"] < min_inliers:
                continue
            if metrics["fitness"] < min_fitness:
                continue
            if metrics["inlier_rmse"] is None or metrics["inlier_rmse"] > max_rmse:
                continue

            # ICP returns successive transforms; preserve their application order.
            correction = np.eye(3)
            for step in history:
                correction = np.vstack((step, [0, 0, 1])) @ correction

            if best_match is None or metrics["fitness"] > best_match["metrics"]["fitness"]:
                best_match = {"index": index, "metrics": metrics, "correction": correction}

        return best_match


