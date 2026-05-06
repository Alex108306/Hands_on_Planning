import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__("frontier_exploration_node")
        self.declare_parameter('mode', 'sim')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.use_transposed_grid = self.mode == "real"

        # Topics in this project:
        # - /projected_map from grid_mapping
        # - /turtlebot/odom from localization_node
        # - /goal_pose consumed by online_motion_planning_node
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/projected_map", self.map_callback, 10
        )
        self.inflated_map_sub = self.create_subscription(
            OccupancyGrid, "/inflated_map", self.inflated_map_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/turtlebot/odom", self.odom_callback, 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.centroid_markers_pub = self.create_publisher(
            MarkerArray, "/frontier_centroids", 10
        )
        self.timer = self.create_timer(1.0, self.update_goal)

        # Map/robot state
        self.map_msg = None
        self.grid_map = None
        self.inflated_map = None
        self.width = None
        self.height = None
        self.resolution = None
        self.origin = None
        self.odom_frame = "world_enu"
        self.robot_xy = None

        # Goal selection settings
        if self.mode == "real":
            self.min_goal_distance_cells = 20
            self.goal_reached_distance_m = 0.05
            self.goal_timeout_sec = 30
            self.blacklist_duration_sec = 30
            self.blacklist_radius_cells = 6
        else:
            self.min_goal_distance_cells = 20
            self.goal_reached_distance_m = 0.25
            self.goal_timeout_sec = 50
            self.blacklist_duration_sec = 45
            self.blacklist_radius_cells = 6
        self.min_new_goal_separation_cells = 4

        # Occupancy interpretation for this map format (0..100 probabilities)
        self.free_threshold = 35
        self.unknown_low = 45
        self.unknown_high = 55
        self.occupied_threshold = 50
        self.goal_clearance_cells = 2
        self.border_margin_cells = 20
        self.unknown_clearance_radius = 1
        self.inward_goal_offset_cells = 8
        self.reachable_seed_search_radius_cells = 4
        self.info_gain_radius_cells = 10
        self.w_gain = 0.7
        self.w_dist = 0.3

        # Goal lifecycle
        self.active_goal_world = None
        self.active_goal_cell = None
        self.active_goal_cluster = None
        self.last_published_goal_cell = None
        self.active_goal_time = None
        self.goal_start_robot_xy = None
        self.progress_check_sec = 10.0
        self.min_progress_m = 0.03

        # Avoid repeatedly choosing unreachable goals.
        self.goal_blacklist = {}

        # RViz centroid markers
        self.centroid_marker_scale_m = 0.16
        self.centroid_text_size_m = 0.12

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        # Keep the same reshape convention used by online_motion_planning_node.
        self.grid_map = np.array(msg.data, dtype=np.int16).reshape(self.width, self.height)
        if self.use_transposed_grid:
            self.grid_map = self.grid_map.T

    def inflated_map_callback(self, msg: OccupancyGrid):
        # online_motion_planning_node publishes /inflated_map in the same grid convention.
        self.inflated_map = np.array(msg.data, dtype=np.int16).reshape(
            msg.info.width, msg.info.height
        )
        if self.use_transposed_grid:
            self.inflated_map = self.inflated_map.T

    def odom_callback(self, msg: Odometry):
        self.odom_frame = msg.header.frame_id
        self.robot_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    def is_free(self, value: int) -> bool:
        return value >= 0 and value <= self.free_threshold

    def is_unknown(self, value: int) -> bool:
        return self.unknown_low <= value <= self.unknown_high

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def get_neighbors_8(self, x_idx: int, y_idx: int):
        neighbors = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x_idx + dx, y_idx + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    neighbors.append((nx, ny))
        return neighbors

    def detect_frontiers(self):
        frontiers = []
        for x_idx in range(self.width):
            for y_idx in range(self.height):
                if not self.is_free(self.grid_map[x_idx, y_idx]):
                    continue
                neighbors = self.get_neighbors_8(x_idx, y_idx)
                has_unknown_neighbor = any(
                    self.is_unknown(self.grid_map[nx, ny]) for nx, ny in neighbors
                )
                if has_unknown_neighbor:
                    frontiers.append((x_idx, y_idx))
        return frontiers

    def is_cell_traversable(self, x_idx, y_idx):
        if self.inflated_map is not None:
            return self.inflated_map[x_idx, y_idx] < self.occupied_threshold
        # Fallback if inflated map is not ready.
        return self.is_free(self.grid_map[x_idx, y_idx])

    def find_nearest_traversable_seed(self, start_cell):
        if start_cell is None:
            return None
        sx, sy = start_cell
        if not (0 <= sx < self.width and 0 <= sy < self.height):
            return None
        if self.is_cell_traversable(sx, sy):
            return start_cell

        max_radius = self.reachable_seed_search_radius_cells
        for radius in range(1, max_radius + 1):
            best_cell = None
            best_dist_sq = float("inf")
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    nx, ny = sx + dx, sy + dy
                    if not (0 <= nx < self.width and 0 <= ny < self.height):
                        continue
                    if not self.is_cell_traversable(nx, ny):
                        continue
                    dist_sq = dx * dx + dy * dy
                    if dist_sq < best_dist_sq:
                        best_dist_sq = dist_sq
                        best_cell = (nx, ny)
            if best_cell is not None:
                self.get_logger().warn(
                    "Robot cell is non-traversable; using nearby free seed cell %s for reachable-space BFS."
                    % (str(best_cell),)
                )
                return best_cell

        return None

    def compute_reachable_cells(self, start_cell):
        if start_cell is None:
            return set()
        if not (0 <= start_cell[0] < self.width and 0 <= start_cell[1] < self.height):
            return set()
        seed_cell = self.find_nearest_traversable_seed(start_cell)
        if seed_cell is None:
            return set()

        visited = set([seed_cell])
        queue = deque([seed_cell])

        while queue:
            current = queue.popleft()
            for nbr in self.get_neighbors_8(current[0], current[1]):
                if nbr in visited:
                    continue
                if self.is_cell_traversable(nbr[0], nbr[1]):
                    visited.add(nbr)
                    queue.append(nbr)
        return visited

    def cluster_frontiers(self, frontiers):
        frontier_set = set(frontiers)
        visited = set()
        clusters = []

        for cell in frontier_set:
            if cell in visited:
                continue
            queue = deque([cell])
            visited.add(cell)
            cluster = []

            while queue:
                current = queue.popleft()
                cluster.append(current)
                for nbr in self.get_neighbors_8(current[0], current[1]):
                    if nbr in frontier_set and nbr not in visited:
                        visited.add(nbr)
                        queue.append(nbr)

            clusters.append(cluster)
        return clusters

    def cluster_centroid(self, cluster):
        arr = np.array(cluster, dtype=float)
        return tuple(arr.mean(axis=0))

    def euclidean(self, a, b):
        return float(math.hypot(a[0] - b[0], a[1] - b[1]))

    def estimate_cluster_info_gain(self, cluster):
        """Estimate unknown-area gain around a cluster centroid."""
        if not cluster:
            return 0.0

        centroid = self.cluster_centroid(cluster)
        cx, cy = int(round(centroid[0])), int(round(centroid[1]))
        radius = self.info_gain_radius_cells
        unknown_count = 0

        x_min = max(0, cx - radius)
        x_max = min(self.width, cx + radius + 1)
        y_min = max(0, cy - radius)
        y_max = min(self.height, cy + radius + 1)

        for x_idx in range(x_min, x_max):
            for y_idx in range(y_min, y_max):
                if self.is_unknown(self.grid_map[x_idx, y_idx]):
                    unknown_count += 1

        return float(unknown_count)

    def normalize_list(self, values):
        if not values:
            return []
        v_min = min(values)
        v_max = max(values)
        if abs(v_max - v_min) < 1e-9:
            return [0.0 for _ in values]
        return [(v - v_min) / (v_max - v_min) for v in values]

    def choose_best_cluster(self, clusters, robot_cell):
        if not clusters:
            return None
        best_cluster = None
        best_distance = float("inf")
        for cluster in clusters:
            centroid = self.cluster_centroid(cluster)
            dist = self.euclidean(robot_cell, centroid)
            if dist < best_distance:
                best_distance = dist
                best_cluster = cluster
        return best_cluster

    def choose_goal_from_cluster(self, best_cluster, robot_cell):
        if not best_cluster:
            return None

        # Ignore goals that are too close to avoid tiny oscillatory movements.
        far_enough = [
            cell
            for cell in best_cluster
            if self.euclidean(robot_cell, cell) >= self.min_goal_distance_cells
        ]
        candidate_cells = far_enough if far_enough else list(best_cluster)
        return min(candidate_cells, key=lambda cell: self.euclidean(robot_cell, cell))

    def is_blacklisted(self, cell):
        expire_time = self.goal_blacklist.get(cell)
        if expire_time is None:
            return False
        if self.now_seconds() >= expire_time:
            del self.goal_blacklist[cell]
            return False
        return True

    def blacklist_goal(self, cell, reason, cluster_cells=None):
        if cell is None:
            return
        expire_time = self.now_seconds() + self.blacklist_duration_sec

        # Prefer blacklisting the full failed frontier cluster so we do not keep
        # retrying nearby points from the same bad region.
        if cluster_cells:
            for c in cluster_cells:
                if 0 <= c[0] < self.width and 0 <= c[1] < self.height:
                    self.goal_blacklist[c] = expire_time
            self.get_logger().info(
                "Blacklisting failed frontier cluster (%d cells) for %.1fs (%s)"
                % (len(cluster_cells), self.blacklist_duration_sec, reason)
            )
            return

        # Fallback: blacklist neighborhood around failed goal cell.
        for dx in range(-self.blacklist_radius_cells, self.blacklist_radius_cells + 1):
            for dy in range(-self.blacklist_radius_cells, self.blacklist_radius_cells + 1):
                nx, ny = cell[0] + dx, cell[1] + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    self.goal_blacklist[(nx, ny)] = expire_time
        self.get_logger().info(
            "Blacklisting goal region around %s (radius=%d) for %.1fs (%s)"
            % (str(cell), self.blacklist_radius_cells, self.blacklist_duration_sec, reason)
        )

    def cleanup_blacklist(self):
        now_t = self.now_seconds()
        expired = [cell for cell, t in self.goal_blacklist.items() if now_t >= t]
        for cell in expired:
            del self.goal_blacklist[cell]

    def is_cell_free_in_inflated_map(self, x_idx, y_idx):
        if self.inflated_map is None:
            # If inflated map is not ready yet, do not block goal generation.
            return True
        return self.inflated_map[x_idx, y_idx] < self.occupied_threshold

    def is_goal_cell_valid(self, x_idx, y_idx):
        # Ignore map-border cells where projected map artifacts are common.
        if (
            x_idx < self.border_margin_cells
            or y_idx < self.border_margin_cells
            or x_idx >= self.width - self.border_margin_cells
            or y_idx >= self.height - self.border_margin_cells
        ):
            return False

        if not self.is_cell_free_in_inflated_map(x_idx, y_idx):
            return False

        # Clearance check: reject goals too close to inflated obstacles.
        for dx in range(-self.goal_clearance_cells, self.goal_clearance_cells + 1):
            for dy in range(-self.goal_clearance_cells, self.goal_clearance_cells + 1):
                nx, ny = x_idx + dx, y_idx + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if not self.is_cell_free_in_inflated_map(nx, ny):
                        return False
        return True

    def is_known_interior_cell(self, x_idx, y_idx):
        # We want commanded goals inside known free area, not exactly on frontier.
        for dx in range(-self.unknown_clearance_radius, self.unknown_clearance_radius + 1):
            for dy in range(-self.unknown_clearance_radius, self.unknown_clearance_radius + 1):
                nx, ny = x_idx + dx, y_idx + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if self.is_unknown(self.grid_map[nx, ny]):
                        return False
        return True

    def cluster_inward_direction(self, cluster):
        # Estimate inward normal of the frontier cluster:
        # accumulate vectors from unknown neighbors toward frontier cells.
        if not cluster:
            return (0.0, 0.0)

        vx_total = 0.0
        vy_total = 0.0
        for fx, fy in cluster:
            for nx, ny in self.get_neighbors_8(fx, fy):
                if self.is_unknown(self.grid_map[nx, ny]):
                    vx_total += fx - nx
                    vy_total += fy - ny

        norm = math.hypot(vx_total, vy_total)
        if norm < 1e-6:
            return (0.0, 0.0)
        return (vx_total / norm, vy_total / norm)

    def frontier_to_interior_goal(
        self, frontier_cell, robot_cell, inward_direction, reachable_cells
    ):
        fx, fy = frontier_cell
        direction = inward_direction

        # Try shifting inward from frontier toward known space.
        for step in range(self.inward_goal_offset_cells, -1, -1):
            gx = int(round(fx + direction[0] * step))
            gy = int(round(fy + direction[1] * step))
            if not (0 <= gx < self.width and 0 <= gy < self.height):
                continue
            if reachable_cells is not None and (gx, gy) not in reachable_cells:
                continue
            # Critical: enforce min distance on the final goal cell (not only frontier cell).
            if self.euclidean(robot_cell, (gx, gy)) < self.min_goal_distance_cells:
                continue
            if (
                self.last_published_goal_cell is not None
                and self.euclidean((gx, gy), self.last_published_goal_cell)
                < self.min_new_goal_separation_cells
            ):
                continue
            if self.is_blacklisted((gx, gy)):
                continue
            if not self.is_goal_cell_valid(gx, gy):
                continue
            if not self.is_known_interior_cell(gx, gy):
                continue
            return (gx, gy)

        return None

    def choose_valid_goal_from_clusters(self, clusters, robot_cell, reachable_cells):
        if not clusters:
            return (None, None)

        # Score clusters using both information gain and travel distance.
        cluster_infos = []
        for cluster in clusters:
            centroid = self.cluster_centroid(cluster)
            distance = self.euclidean(robot_cell, centroid)
            info_gain = self.estimate_cluster_info_gain(cluster)
            cluster_infos.append(
                {
                    "cluster": cluster,
                    "distance": distance,
                    "info_gain": info_gain,
                }
            )

        norm_distances = self.normalize_list([c["distance"] for c in cluster_infos])
        norm_gains = self.normalize_list([c["info_gain"] for c in cluster_infos])

        for i, c in enumerate(cluster_infos):
            c["score"] = self.w_gain * norm_gains[i] - self.w_dist * norm_distances[i]

        # Higher score first; tie-break by shorter distance.
        sorted_clusters = [
            c["cluster"]
            for c in sorted(
                cluster_infos,
                key=lambda c: (-c["score"], c["distance"]),
            )
        ]

        for cluster in sorted_clusters:
            inward_direction = self.cluster_inward_direction(cluster)
            # Reuse min-distance rule first, then fallback to closest.
            far_enough = [
                cell
                for cell in cluster
                if self.euclidean(robot_cell, cell) >= self.min_goal_distance_cells
            ]
            ordered_candidates = sorted(
                far_enough if far_enough else list(cluster),
                key=lambda cell: self.euclidean(robot_cell, cell),
            )

            for cand in ordered_candidates:
                if self.is_blacklisted(cand):
                    continue
                interior_goal = self.frontier_to_interior_goal(
                    cand, robot_cell, inward_direction, reachable_cells
                )
                if interior_goal is not None:
                    return (interior_goal, cluster)

        return (None, None)

    def world_to_cell(self, x_world, y_world):
        if self.origin is None or self.resolution is None:
            return None
        x_idx = int((x_world - self.origin.position.x) / self.resolution)
        y_idx = int((y_world - self.origin.position.y) / self.resolution)
        if x_idx < 0 or y_idx < 0 or x_idx >= self.width or y_idx >= self.height:
            return None
        return (x_idx, y_idx)

    def cell_to_world(self, x_idx, y_idx):
        x_world = self.origin.position.x + (x_idx + 0.5) * self.resolution
        y_world = self.origin.position.y + (y_idx + 0.5) * self.resolution
        return (x_world, y_world)

    def publish_cluster_centroids(self, clusters):
        marker_array = MarkerArray()

        # Always clear previous markers first.
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        if self.map_msg is None or self.origin is None or self.resolution is None:
            self.centroid_markers_pub.publish(marker_array)
            return

        frame_id = self.map_msg.header.frame_id
        stamp = self.get_clock().now().to_msg()

        for idx, cluster in enumerate(clusters):
            centroid = self.cluster_centroid(cluster)
            cx_world = self.origin.position.x + (centroid[0] + 0.5) * self.resolution
            cy_world = self.origin.position.y + (centroid[1] + 0.5) * self.resolution

            point_marker = Marker()
            point_marker.header.frame_id = frame_id
            point_marker.header.stamp = stamp
            point_marker.ns = "frontier_cluster_centroids"
            point_marker.id = idx
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.pose.position.x = cx_world
            point_marker.pose.position.y = cy_world
            point_marker.pose.position.z = 0.05
            point_marker.pose.orientation.w = 1.0
            point_marker.scale.x = self.centroid_marker_scale_m
            point_marker.scale.y = self.centroid_marker_scale_m
            point_marker.scale.z = self.centroid_marker_scale_m
            point_marker.color.r = 0.10
            point_marker.color.g = 0.95
            point_marker.color.b = 1.00
            point_marker.color.a = 0.95
            marker_array.markers.append(point_marker)

            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = "frontier_cluster_centroid_labels"
            text_marker.id = 10000 + idx
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = cx_world
            text_marker.pose.position.y = cy_world
            text_marker.pose.position.z = 0.20
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = self.centroid_text_size_m
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"C{idx}"
            marker_array.markers.append(text_marker)

        self.centroid_markers_pub.publish(marker_array)

    def is_goal_reached(self):
        if self.active_goal_world is None or self.robot_xy is None:
            return False
        return self.euclidean(self.robot_xy, self.active_goal_world) <= self.goal_reached_distance_m

    def publish_goal(self, goal_world_xy):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.odom_frame
        goal_msg.pose.position.x = goal_world_xy[0]
        goal_msg.pose.position.y = goal_world_xy[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

    def reset_active_goal(self):
        self.active_goal_world = None
        self.active_goal_cell = None
        self.active_goal_cluster = None
        self.active_goal_time = None
        self.goal_start_robot_xy = None

    def update_goal(self):
        if self.grid_map is None or self.robot_xy is None:
            return
        self.cleanup_blacklist()

        # Keep current goal until reached, but reset if it times out.
        if self.active_goal_world is not None:
            if self.is_goal_reached():
                self.get_logger().info("Frontier goal reached, selecting next goal.")
                self.reset_active_goal()
            elif self.active_goal_time is not None:
                elapsed = (self.get_clock().now() - self.active_goal_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout_sec:
                    self.get_logger().info("Goal timeout, selecting a new frontier goal.")
                    self.blacklist_goal(
                        self.active_goal_cell,
                        "timeout/no-reach",
                        cluster_cells=self.active_goal_cluster,
                    )
                    self.reset_active_goal()
                elif (
                    self.goal_start_robot_xy is not None
                    and elapsed >= self.progress_check_sec
                ):
                    start_dist = self.euclidean(self.goal_start_robot_xy, self.active_goal_world)
                    current_dist = self.euclidean(self.robot_xy, self.active_goal_world)
                    progress = start_dist - current_dist
                    if progress < self.min_progress_m:
                        self.get_logger().info(
                            "Low progress toward goal (%.3fm), selecting new goal." % progress
                        )
                        self.blacklist_goal(
                            self.active_goal_cell,
                            "low-progress",
                            cluster_cells=self.active_goal_cluster,
                        )
                        self.reset_active_goal()
                    else:
                        return
                else:
                    return
            else:
                return

        robot_cell = self.world_to_cell(self.robot_xy[0], self.robot_xy[1])
        if robot_cell is None:
            self.get_logger().warn("Robot pose is outside map bounds, cannot select frontier goal.")
            return

        frontiers = self.detect_frontiers()
        if not frontiers:
            self.publish_cluster_centroids([])
            self.get_logger().info("No frontier cells available.")
            return

        reachable_cells = self.compute_reachable_cells(robot_cell)
        if not reachable_cells:
            self.get_logger().warn("Could not compute reachable free-space from robot cell.")
            return
        frontiers = [cell for cell in frontiers if cell in reachable_cells]
        if not frontiers:
            self.publish_cluster_centroids([])
            self.get_logger().info("No reachable frontier cells available.")
            return

        clusters = self.cluster_frontiers(frontiers)
        self.publish_cluster_centroids(clusters)
        goal_cell, source_cluster = self.choose_valid_goal_from_clusters(
            clusters, robot_cell, reachable_cells
        )
        if goal_cell is None:
            self.get_logger().info(
                "No valid frontier goal after filtering with inflated-map clearance."
            )
            return

        goal_world = self.cell_to_world(goal_cell[0], goal_cell[1])
        self.publish_goal(goal_world)
        self.active_goal_world = goal_world
        self.active_goal_cell = goal_cell
        self.active_goal_cluster = list(source_cluster) if source_cluster is not None else None
        self.last_published_goal_cell = goal_cell
        self.active_goal_time = self.get_clock().now()
        self.goal_start_robot_xy = self.robot_xy
        self.get_logger().info(
            "Published frontier goal x=%.3f y=%.3f (cell=%s)"
            % (goal_world[0], goal_world[1], str(goal_cell))
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
