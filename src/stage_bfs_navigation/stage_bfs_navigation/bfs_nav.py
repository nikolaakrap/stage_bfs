import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import yaml
import time
import math
from collections import deque

from geometry_msgs.msg import Twist, PointStamped, Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan

import tf2_ros
from tf_transformations import euler_from_quaternion


def yaw_from_quat(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class BFSNav(Node):
    def __init__(self):
        super().__init__('bfs_nav')

        # parameters
        self.declare_parameter('map_yaml', '/home/user/stage_bfs_ws/maps/stage_map.yaml')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/base_scan')

        # follower tuning
        self.declare_parameter('wp_tol', 0.15)
        self.declare_parameter('lin_speed', 0.20)
        self.declare_parameter('ang_kp', 1.6)
        self.declare_parameter('max_ang', 1.2)

        # collision stop tuning
        self.declare_parameter('stop_dist', 0.35)
        self.declare_parameter('front_angle_deg', 30.0)

        # recovery tuning
        self.declare_parameter('recovery_backup_speed', -0.12)  # negative = back
        self.declare_parameter('recovery_backup_time', 0.7)     # seconds
        self.declare_parameter('recovery_turn_speed', 0.8)      # rad/s
        self.declare_parameter('recovery_turn_time', 1.0)       # seconds

        # dynamic obstacle blocking (prevents infinite replanning through same obstacle)
        self.declare_parameter('block_ahead_cells', 6)   # how many cells ahead to mark as obstacle
        self.declare_parameter('block_width_cells', 2)   # thickness in cells
        self.declare_parameter('max_replans', 8)         # safety stop

        # read parameters
        self.map_yaml = self.get_parameter('map_yaml').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value

        self.wp_tol = float(self.get_parameter('wp_tol').value)
        self.lin_speed = float(self.get_parameter('lin_speed').value)
        self.ang_kp = float(self.get_parameter('ang_kp').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))

        self.rec_back_speed = float(self.get_parameter('recovery_backup_speed').value)
        self.rec_back_time = float(self.get_parameter('recovery_backup_time').value)
        self.rec_turn_speed = float(self.get_parameter('recovery_turn_speed').value)
        self.rec_turn_time = float(self.get_parameter('recovery_turn_time').value)

        self.block_ahead_cells = int(self.get_parameter('block_ahead_cells').value)
        self.block_width_cells = int(self.get_parameter('block_width_cells').value)
        self.max_replans = int(self.get_parameter('max_replans').value)

        self.replan_counter = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publishers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.explored_pub = self.create_publisher(MarkerArray, '/explored_nodes', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.path_marker_pub = self.create_publisher(Marker, '/path_marker', 10)
        self.goal_marker_pub = self.create_publisher(MarkerArray, '/goal_markers', 10)
        self.target_marker_pub = self.create_publisher(Marker, '/target_waypoint', 10)

        # subscriptions
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)

        # map
        self.maze = None          # 0 free, 1 obstacle
        self.origin = None        # [x, y, yaw]
        self.resolution = None
        self.load_map_from_yaml(self.map_yaml)

        # start & goal
        self.start_grid = None
        self.goal_grid = None

        # last goal (used for auto-replanning after recovery)
        self.last_goal_grid = None
        self.last_goal_world = None

        # path following
        self.path_world = []      # [(x,y), ...]
        self.wp_idx = 0
        self.following = False

        # laser
        self.last_scan = None

        # recovery state machine
        self.state = 'IDLE'  # IDLE, FOLLOW, REC_BACKUP, REC_TURN
        self.state_start_t = None

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info("BFSNav ready.")
        self.get_logger().info("Publish Points: 1. START, 2. GOAL.")

    # ---------------- MAP LOADING ----------------
    def load_map_from_yaml(self, yaml_path):
        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)

        self.resolution = float(map_yaml['resolution'])
        self.origin = map_yaml['origin']
        image_path = map_yaml['image']

        if not image_path.startswith('/'):
            import os
            base_dir = os.path.dirname(yaml_path)
            image_path = os.path.join(base_dir, image_path)

        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"Couldn't load map: {image_path}")

        img = np.flipud(img)

        # obstacle pixels are very dark (same as your Dijkstra code)
        self.maze = (img < 10).astype(np.uint8)

        self.get_logger().info(f"Loaded map: {image_path}")
        self.get_logger().info(f"Map shape: {self.maze.shape}, resolution: {self.resolution}, origin: {self.origin}")

    def world_to_grid(self, x, y):
        c = int((x - self.origin[0]) / self.resolution)
        r = int((y - self.origin[1]) / self.resolution)
        return r, c

    def grid_to_world(self, rc):
        r, c = rc
        x = self.origin[0] + (c + 0.5) * self.resolution
        y = self.origin[1] + (r + 0.5) * self.resolution
        return x, y

    def is_free(self, r, c):
        rows, cols = self.maze.shape
        if 0 <= r < rows and 0 <= c < cols:
            return self.maze[r, c] == 0
        return False

    def nearest_free(self, r, c, max_r=25):
        q = deque([(r, c)])
        seen = {(r, c)}
        while q:
            rr, cc = q.popleft()
            if self.is_free(rr, cc):
                return (rr, cc)
            for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
                nr, nc = rr + dr, cc + dc
                if (nr, nc) in seen:
                    continue
                if abs(nr - r) > max_r or abs(nc - c) > max_r:
                    continue
                seen.add((nr, nc))
                q.append((nr, nc))
        return None

    # ---------------- LASER ----------------
    def on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def front_blocked(self):
        if self.last_scan is None:
            return False

        scan = self.last_scan
        ranges = np.array(scan.ranges, dtype=np.float32)
        valid = np.isfinite(ranges)
        if not np.any(valid):
            return False

        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
        angles = angles[valid]
        ranges = ranges[valid]

        mask = np.abs(angles) <= self.front_angle
        front = ranges[mask]
        if front.size == 0:
            return False

        return np.min(front) < self.stop_dist

    # ---------------- TF POSE ----------------
    def get_robot_pose_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, rclpy.time.Time())
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            yaw = yaw_from_quat(tf.transform.rotation)
            return x, y, yaw
        except Exception:
            return None

    # ---------------- DYNAMIC BLOCKING ----------------
    def block_cells_ahead(self):
        """
        When laser says "blocked", we mark a small patch of cells in front of the robot
        as obstacle in self.maze. This prevents BFS from picking the same impossible
        corridor repeatedly.
        """
        pose = self.get_robot_pose_map()
        if pose is None:
            return

        rx, ry, ryaw = pose
        rows, cols = self.maze.shape

        # direction forward (world)
        step_x = math.cos(ryaw)
        step_y = math.sin(ryaw)

        # mark several cells ahead, with thickness
        for i in range(1, self.block_ahead_cells + 1):
            wx = rx + i * self.resolution * step_x
            wy = ry + i * self.resolution * step_y
            r, c = self.world_to_grid(wx, wy)

            for dr in range(-self.block_width_cells, self.block_width_cells + 1):
                for dc in range(-self.block_width_cells, self.block_width_cells + 1):
                    r2 = r + dr
                    c2 = c + dc
                    if 0 <= r2 < rows and 0 <= c2 < cols:
                        self.maze[r2, c2] = 1  # obstacle

    # ---------------- CLICK START/GOAL ----------------
    def clicked_point_callback(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        r, c = self.world_to_grid(x, y)

        if not self.is_free(r, c):
            nf = self.nearest_free(r, c, max_r=25)
            if nf is None:
                self.get_logger().warn(f"Click ({x:.2f},{y:.2f}) -> ({r},{c}) not free & no free nearby.")
                return
            r, c = nf
            x, y = self.grid_to_world((r, c))
            self.get_logger().warn(f"Click not free -> snap to ({r},{c}) world ({x:.2f},{y:.2f}).")

        # reset replan counter when user gives a new goal
        # (so it doesn't trip max_replans immediately)
        if self.start_grid is None:
            self.replan_counter = 0

        if self.start_grid is None:
            self.start_grid = (r, c)
            self.goal_grid = None
            self.stop_robot()
            self.following = False
            self.state = 'IDLE'
            self.get_logger().info(f"START = {self.start_grid} world ({x:.2f},{y:.2f})")
            self.publish_goal_markers()
            return

        if self.goal_grid is None:
            self.goal_grid = (r, c)
            self.last_goal_grid = self.goal_grid
            self.last_goal_world = (x, y)
            self.get_logger().info(f"GOAL = {self.goal_grid} world ({x:.2f},{y:.2f})")
            self.publish_goal_markers()

            self.plan_and_start_follow()
            self.start_grid = None
            self.goal_grid = None
            return

        # fallback reset
        self.start_grid = (r, c)
        self.goal_grid = None
        self.publish_goal_markers()

    # ---------------- MARKERS ----------------
    def publish_goal_markers(self):
        ma = MarkerArray()

        def sphere(mid, x, y, rr, gg, bb):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "goals"
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.2
            m.pose.orientation.w = 1.0
            s = self.resolution * 3.0
            m.scale.x = s
            m.scale.y = s
            m.scale.z = s
            m.color.r = rr
            m.color.g = gg
            m.color.b = bb
            m.color.a = 1.0
            return m

        if self.start_grid:
            x, y = self.grid_to_world(self.start_grid)
            ma.markers.append(sphere(0, x, y, 0.0, 1.0, 0.0))
        if self.goal_grid:
            x, y = self.grid_to_world(self.goal_grid)
            ma.markers.append(sphere(1, x, y, 1.0, 0.0, 0.0))

        self.goal_marker_pub.publish(ma)

    def explored_marker(self, rc, mid):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "explored_nodes"
        m.id = mid
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        x, y = self.grid_to_world(rc)
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.0
        m.pose.orientation.w = 1.0
        s = self.resolution * 0.9
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 0.35
        return m

    def publish_target_waypoint_marker(self, x, y):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "target_wp"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.25
        m.pose.orientation.w = 1.0
        s = self.resolution * 2.0
        m.scale.x = s
        m.scale.y = s
        m.scale.z = s
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0
        self.target_marker_pub.publish(m)

    def visualize_path(self, path_cells):
        if not path_cells:
            return None

        stamp = self.get_clock().now().to_msg()

        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = stamp
        line.ns = "path"
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.resolution * 0.8
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0

        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = stamp

        world_pts = []
        for rc in path_cells:
            x, y = self.grid_to_world(rc)
            world_pts.append((x, y))

            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            line.points.append(p)

            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_marker_pub.publish(line)
        self.path_pub.publish(path_msg)
        return world_pts

    # ---------------- BFS ----------------
    def bfs(self, start, goal):
        rows, cols = self.maze.shape
        visited = np.zeros((rows, cols), dtype=bool)
        prev = np.full((rows, cols, 2), -1, dtype=np.int32)

        q = deque([start])
        visited[start] = True

        explored = MarkerArray()
        mid = 0
        nbrs = [(-1,0),(1,0),(0,-1),(0,1)]

        while q:
            r, c = q.popleft()

            explored.markers.append(self.explored_marker((r, c), mid))
            mid += 1
            if len(explored.markers) >= 200:
                self.explored_pub.publish(explored)
                explored = MarkerArray()
                time.sleep(0.03)

            if (r, c) == goal:
                break

            for dr, dc in nbrs:
                nr, nc = r + dr, c + dc
                if 0 <= nr < rows and 0 <= nc < cols:
                    if (not visited[nr, nc]) and self.maze[nr, nc] == 0:
                        visited[nr, nc] = True
                        prev[nr, nc] = [r, c]
                        q.append((nr, nc))

        if explored.markers:
            self.explored_pub.publish(explored)

        # reconstruct
        path = []
        cur = goal
        while True:
            path.append(cur)
            rr, cc = cur
            pr, pc = prev[rr, cc]
            if pr == -1:
                break
            cur = (pr, pc)

        path.reverse()
        if path and path[0] == start:
            return path
        return []

    # ---------------- PLANNING ----------------
    def plan_and_start_follow(self):
        # clearing old markers
        clear = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        clear.markers.append(m)
        self.explored_pub.publish(clear)
        self.path_marker_pub.publish(m)

        start = self.start_grid
        goal = self.goal_grid
        if start is None or goal is None:
            self.get_logger().warn("Start/goal not set.")
            return

        t0 = time.time()
        path_cells = self.bfs(start, goal)
        t1 = time.time()

        if not path_cells:
            self.get_logger().warn("BFS: no path.")
            self.following = False
            self.state = 'IDLE'
            return

        self.get_logger().info(f"BFS path: {len(path_cells)} nodes, {t1 - t0:.3f}s")

        world_pts = self.visualize_path(path_cells)
        if not world_pts or len(world_pts) < 2:
            self.get_logger().warn("Path too short.")
            self.following = False
            self.state = 'IDLE'
            return

        self.path_world = world_pts
        self.wp_idx = 0
        self.following = True
        self.state = 'FOLLOW'

    # ---------------- RECOVERY ----------------
    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def start_recovery(self):
        self.following = False
        self.state = 'REC_BACKUP'
        self.state_start_t = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().warn("RECOVERY: backup start")

    def do_recovery_backup(self, now_s):
        cmd = Twist()
        cmd.linear.x = self.rec_back_speed
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        if (now_s - self.state_start_t) >= self.rec_back_time:
            self.state = 'REC_TURN'
            self.state_start_t = now_s
            self.get_logger().warn("RECOVERY: turn start")

    def do_recovery_turn(self, now_s):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.rec_turn_speed
        self.cmd_pub.publish(cmd)

        if (now_s - self.state_start_t) >= self.rec_turn_time:
            self.stop_robot()
            self.state = 'IDLE'
            self.get_logger().warn("RECOVERY: done")

            if self.last_goal_world is not None:
                self.replan_from_current_pose_to_last_goal()

    def replan_from_current_pose_to_last_goal(self):
        # safety: avoid infinite loops
        self.replan_counter += 1
        if self.replan_counter > self.max_replans:
            self.get_logger().warn("Too many replans -> stop. Click a new goal.")
            self.following = False
            self.state = 'IDLE'
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            self.get_logger().warn("Cannot replan: TF not available.")
            return

        rx, ry, _ = pose
        sr, sc = self.world_to_grid(rx, ry)

        if not self.is_free(sr, sc):
            nf = self.nearest_free(sr, sc, max_r=25)
            if nf is None:
                self.get_logger().warn("Can't find free start after recovery.")
                return
            sr, sc = nf

        # keep goal grid cached
        if self.last_goal_grid is None:
            gr, gc = self.world_to_grid(self.last_goal_world[0], self.last_goal_world[1])
            if not self.is_free(gr, gc):
                nf = self.nearest_free(gr, gc, max_r=25)
                if nf is None:
                    self.get_logger().warn("Can't find free goal.")
                    return
                gr, gc = nf
            self.last_goal_grid = (gr, gc)

        start = (sr, sc)
        goal = self.last_goal_grid

        self.get_logger().info(f"Auto-replan #{self.replan_counter}: start {start} -> goal {goal}")
        self.start_grid = start
        self.goal_grid = goal
        self.plan_and_start_follow()
        self.start_grid = None
        self.goal_grid = None

    # ---------------- MAIN LOOP ----------------
    def loop(self):
        now_s = self.get_clock().now().nanoseconds / 1e9

        # recovery states
        if self.state == 'REC_BACKUP':
            self.do_recovery_backup(now_s)
            return

        if self.state == 'REC_TURN':
            self.do_recovery_turn(now_s)
            return

        # normal following
        if self.state != 'FOLLOW' or (not self.following) or (not self.path_world):
            return

        # if blocked -> block cells ahead + recovery
        if self.front_blocked():
            self.stop_robot()
            self.block_cells_ahead()   # <-- key change to avoid repeated impossible paths
            self.start_recovery()
            return

        pose = self.get_robot_pose_map()
        if pose is None:
            return

        rx, ry, ryaw = pose

        if self.wp_idx >= len(self.path_world):
            self.following = False
            self.state = 'IDLE'
            self.stop_robot()
            return

        tx, ty = self.path_world[self.wp_idx]
        dx = tx - rx
        dy = ty - ry
        dist = math.hypot(dx, dy)

        if dist < self.wp_tol:
            self.wp_idx += 1
            if self.wp_idx >= len(self.path_world):
                self.following = False
                self.state = 'IDLE'
                self.stop_robot()
            return

        self.publish_target_waypoint_marker(tx, ty)

        target_yaw = math.atan2(dy, dx)
        yaw_err = (target_yaw - ryaw + math.pi) % (2 * math.pi) - math.pi

        cmd = Twist()
        cmd.angular.z = max(-self.max_ang, min(self.max_ang, self.ang_kp * yaw_err))

        lin = min(self.lin_speed, 0.8 * dist)
        lin *= max(0.0, 1.0 - abs(yaw_err))
        cmd.linear.x = max(0.0, lin)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BFSNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cmd_pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
