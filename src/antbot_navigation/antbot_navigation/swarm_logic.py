import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import random

# ── Arena boundary (2 × 2 m interior, walls at ±1.05 m) ─────────────────────
ARENA_MIN_X = -1.0
ARENA_MAX_X =  1.0
ARENA_MIN_Y = -1.0
ARENA_MAX_Y =  1.0
BOUNDARY_MARGIN = 0.22       # start turning this far from the wall

# ── Robot physical radius (used for inter-bot separation) ────────────────────
BOT_RADIUS      = 0.13       # metres (base cylinder r = 0.10 + wheel clearance)
SAFE_BOT_GAP    = 0.04       # extra clearance beyond 2 × BOT_RADIUS
BOT_SAFE_DIST   = 2 * BOT_RADIUS + SAFE_BOT_GAP   # ≈ 0.30 m  → repel below this

# ── IR sensor (simulates ESP32 IR proximity array) ───────────────────────────
IR_OBSTACLE_DIST = 0.20      # metres – cube / wall considered "close"

# ── Colour sensor threshold (simulated via camera blob area) ─────────────────
COLOR_SENSOR_MIN_AREA = 75000  # px² – triggers STOPPED state (stops very close to target)

# ── All known namespaces ──────────────────────────────────────────────────────
ALL_BOTS = ['bot1', 'bot2', 'bot3']

OFFSETS = {
    'bot1': (-0.75, -0.75),
    'bot2': (-0.75, -0.45),
    'bot3': (-0.45, -0.75)
}


class SwarmBot(Node):
    """
    Simulates one antbot.  State machine:
        SEARCH       → ant-like random walk, boundary + IR + peer avoidance
        APPROACH     → green blob in camera → visual servo toward it
        STOPPED      → colour-sensor threshold reached → full stop + broadcast
        GO_TO_TARGET → received peer broadcast → navigate to that point
    """

    def __init__(self):
        super().__init__('swarm_bot')

        self.declare_parameter('namespace', 'bot1')
        self.ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.spawn_x, self.spawn_y = OFFSETS.get(self.ns, (0.0, 0.0))

        self.state  = 'SEARCH'
        self.search_state_time = 0.0
        self.bridge = CvBridge()
        self.found_green = False

        # Odometry of THIS bot
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # Positions of PEER bots  {namespace: (x, y)}
        self.peer_positions = {}

        # Target broadcast by another bot
        self.target_x = None
        self.target_y = None

        # Camera / colour sensor state
        self.green_centroid_x = None
        self.green_blob_area  = 0.0

        # IR readings
        self.ir_front_left  = float('inf')
        self.ir_front       = float('inf')
        self.ir_front_right = float('inf')

        # Ant random-walk state
        self.search_state_time = self._now()
        self.search_duration   = random.uniform(1.0, 2.0)
        self.search_action     = 'straight'
        self.turn_direction    = random.choice([-1, 1])
        self.pheromone_yaw     = random.uniform(-math.pi, math.pi)

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, f'/{self.ns}/cmd_vel', 10)
        self.loc_pub = self.create_publisher(Point, '/green_cube_location', 10)
        self.state_pub = self.create_publisher(String, f'/{self.ns}/state', 10)

        # ── Subscribers – self ────────────────────────────────────────────────
        self.create_subscription(Image,     f'/{self.ns}/camera/image_raw', self._img_cb,  10)
        self.create_subscription(Odometry,  f'/{self.ns}/odom',             self._odom_cb, 10)
        self.create_subscription(LaserScan, f'/{self.ns}/scan',             self._ir_cb,   10)
        self.create_subscription(Point,     '/green_cube_location',          self._loc_cb,  10)
        self.create_subscription(String,    '/swarm_control',               self._control_cb, 10)

        # ── Subscribers – peer odometry (inter-bot collision avoidance) ───────
        for peer in ALL_BOTS:
            if peer != self.ns:
                self.create_subscription(
                    Odometry,
                    f'/{peer}/odom',
                    lambda msg, p=peer: self._peer_odom_cb(msg, p),
                    10
                )

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(f'[{self.ns}] ready – PAUSED')

    # ── Utility ───────────────────────────────────────────────────────────────
    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _wrap(self, a):
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _odom_cb(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        q        = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

    def _peer_odom_cb(self, msg, peer_ns):
        self.peer_positions[peer_ns] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    def _ir_cb(self, msg):
        ranges = msg.ranges
        n      = len(ranges)
        if n == 0:
            return

        def get_min_in_sector(deg_start, deg_end):
            rad_start = math.radians(deg_start)
            rad_end   = math.radians(deg_end)
            
            idx_start = int((rad_start - msg.angle_min) / msg.angle_increment)
            idx_end   = int((rad_end - msg.angle_min) / msg.angle_increment)
            
            i_min = max(0, min(n - 1, min(idx_start, idx_end)))
            i_max = max(0, min(n - 1, max(idx_start, idx_end)))
            
            vals = [r for r in ranges[i_min : i_max + 1] if math.isfinite(r) and r > 0.01]
            return min(vals) if vals else float('inf')

        self.ir_front_left  = get_min_in_sector(10, 30)
        self.ir_front       = get_min_in_sector(-10, 10)
        self.ir_front_right = get_min_in_sector(-30, -10)

    def _img_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'img_cb: {e}')
            return

        hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([38, 50, 50]), np.array([85, 255, 255]))
        cnts, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        best, best_area = None, 0
        for c in cnts:
            a = cv2.contourArea(c)
            if a > 60 and a > best_area:
                best_area, best = a, c

        if best is not None:
            M = cv2.moments(best)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                w  = img.shape[1]
                self.green_centroid_x = (cx - w / 2.0) / (w / 2.0)
                self.green_blob_area  = best_area

                if self.state in ('SEARCH', 'GO_TO_TARGET'):
                    self.get_logger().info(f'[{self.ns}] Green spotted → APPROACH')
                    self.state = 'APPROACH'

                # Stop when we are very close:
                # - We are in APPROACH state, confirmed by a large green area (> 35000 pixels)
                # - And our front IR sensor measures < 0.06 m or camera blob is extremely large (fallback)
                is_close = (self.state == 'APPROACH' and
                            best_area > 35000 and (
                                self.ir_front < 0.06 or
                                self.ir_front_left < 0.06 or
                                self.ir_front_right < 0.06 or
                                best_area > 120000
                            ))

                if is_close and self.state in ('APPROACH', 'SEARCH'):
                    if self.state != 'STOPPED':
                        self.get_logger().info(
                            f'[{self.ns}] ★ GREEN CUBE REACHED (area={int(best_area)}, ir_front={self.ir_front:.2f}, L={self.ir_front_left:.2f}, R={self.ir_front_right:.2f}) → STOPPED')
                    self.state = 'STOPPED'
                    self.found_green = True
                    p = Point()
                    p.x = self.x + self.spawn_x
                    p.y = self.y + self.spawn_y
                    p.z = 0.0
                    self.loc_pub.publish(p)
        else:
            self.green_centroid_x = None
            self.green_blob_area  = 0.0
            if self.state == 'APPROACH':
                self.state = 'SEARCH'

    def _loc_cb(self, msg):
        local_x = msg.x - self.spawn_x
        local_y = msg.y - self.spawn_y
        if self.state == 'SEARCH':
            self.get_logger().info(
                f'[{self.ns}] Peer found green @ ({msg.x:.2f},{msg.y:.2f}) → GO_TO_TARGET')
            self.target_x = local_x
            self.target_y = local_y
            self.state    = 'GO_TO_TARGET'
        elif self.state == 'GO_TO_TARGET':
            self.target_x = local_x
            self.target_y = local_y

    def _control_cb(self, msg):
        cmd = msg.data.upper()
        if cmd == 'START':
            if self.state == 'PAUSED':
                self.state = 'SEARCH'
                self.search_state_time = self._now()
                self.get_logger().info(f'[{self.ns}] START command received → SEARCH')
        elif cmd == 'STOP':
            self.state = 'PAUSED'
            self.get_logger().info(f'[{self.ns}] STOP command received → PAUSED')
        elif cmd == 'RESET':
            self.state = 'SEARCH'
            self.found_green = False
            self.target_x = None
            self.target_y = None
            self.green_centroid_x = None
            self.green_blob_area  = 0.0
            self.search_state_time = self._now()
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f'[{self.ns}] RESET command received → SEARCH')

    # ── Avoidance helpers ─────────────────────────────────────────────────────

    def _boundary_avoid(self):
        """Returns a Twist to escape walls, or None if clear."""
        cmd    = Twist()
        flee   = False
        turn_z = 0.0

        # West wall (x is near min)
        if self.x < ARENA_MIN_X + BOUNDARY_MARGIN:
            if abs(self.yaw) > math.pi / 2:
                flee = True
                turn_z = 1.2 if self.yaw < 0 else -1.2

        # East wall (x is near max)
        elif self.x > ARENA_MAX_X - BOUNDARY_MARGIN:
            if abs(self.yaw) < math.pi / 2:
                flee = True
                turn_z = 1.2 if self.yaw > 0 else -1.2

        # South wall (y is near min)
        if self.y < ARENA_MIN_Y + BOUNDARY_MARGIN:
            if self.yaw < 0:
                flee = True
                turn_z = -1.2 if self.yaw < -math.pi / 2 else 1.2

        # North wall (y is near max)
        elif self.y > ARENA_MAX_Y - BOUNDARY_MARGIN:
            if self.yaw > 0:
                flee = True
                turn_z = 1.2 if self.yaw > math.pi / 2 else -1.2

        if flee:
            cmd.linear.x  = 0.0
            cmd.angular.z = turn_z
            return cmd
        return None

    def _ir_avoid(self):
        """Returns a Twist to dodge a cube/wall detected by IR, or None."""
        fl, fc, fr = self.ir_front_left, self.ir_front, self.ir_front_right
        cmd = Twist()
        if fc < IR_OBSTACLE_DIST:
            cmd.linear.x  = -0.05
            cmd.angular.z = 1.0 if fl > fr else -1.0
            return cmd
        if fl < IR_OBSTACLE_DIST:
            cmd.linear.x  = 0.05
            cmd.angular.z = -0.7
            return cmd
        if fr < IR_OBSTACLE_DIST:
            cmd.linear.x  = 0.05
            cmd.angular.z =  0.7
            return cmd
        return None

    def _bot_repulsion(self):
        """
        Returns a Twist repulsion vector if any peer is dangerously close.
        Uses peer odometry  (mirrors what ESP32 would do via RF distance).
        """
        if not self.peer_positions:
            return None

        repel_x, repel_y = 0.0, 0.0
        triggered = False

        for peer_ns, (px, py) in self.peer_positions.items():
            dx   = self.x - px
            dy   = self.y - py
            dist = math.sqrt(dx**2 + dy**2)

            if dist < BOT_SAFE_DIST and dist > 0.01:
                # Repulsive force proportional to how close they are
                strength  = (BOT_SAFE_DIST - dist) / BOT_SAFE_DIST
                repel_x  += (dx / dist) * strength
                repel_y  += (dy / dist) * strength
                triggered = True

        if not triggered:
            return None

        # Convert repulsion vector to a steering command
        repel_yaw  = math.atan2(repel_y, repel_x)
        err_yaw    = self._wrap(repel_yaw - self.yaw)
        cmd        = Twist()

        if abs(err_yaw) > 0.3:
            cmd.linear.x  = 0.0
            cmd.angular.z = 1.0 if err_yaw > 0 else -1.0
        else:
            cmd.linear.x  = 0.15
            cmd.angular.z = 0.8 * err_yaw

        return cmd

    def _control_loop(self):
        if self.search_state_time == 0.0:
            self.search_state_time = self._now()

        # Publish current state for the GUI server
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)

        cmd = Twist()

        # ── PAUSED: hold position ────────────────────────────────────────────
        if self.state == 'PAUSED':
            self.cmd_pub.publish(cmd)
            return

        # ── STOPPED: hold position forever ───────────────────────────────────
        if self.state == 'STOPPED':
            self.cmd_pub.publish(cmd)
            if self.found_green:
                # Continuously broadcast green cube location to peers
                p = Point()
                p.x = self.x + self.spawn_x
                p.y = self.y + self.spawn_y
                p.z = 0.0
                self.loc_pub.publish(p)
            return

        # ── Safety priority: boundary → IR obstacle → inter-bot repulsion ────
        # Bypass IR obstacle avoidance in APPROACH state so we can get close to the green cube
        avoid = (
            self._boundary_avoid() or
            (None if self.state == 'APPROACH' else self._ir_avoid()) or
            self._bot_repulsion()
        )

        # ── SEARCH: ant-like random walk ──────────────────────────────────────
        if self.state == 'SEARCH':
            if avoid:
                self.cmd_pub.publish(avoid)
                self.search_state_time = self._now()
                return

            now = self._now()
            if now - self.search_state_time > self.search_duration:
                self.search_state_time = now
                if self.search_action == 'straight':
                    self.search_action   = 'turn'
                    self.search_duration = random.uniform(0.4, 0.9)
                    self.turn_direction  = random.choice([-1, 1])
                else:
                    self.search_action       = 'straight'
                    # Levy Flight: 80% short steps, 20% long steps
                    if random.random() < 0.20:
                        self.search_duration = random.uniform(4.0, 7.0) # long run to cross arena
                    else:
                        self.search_duration = random.uniform(1.5, 3.0) # normal run
                    self.pheromone_yaw      += random.uniform(-1.2, 1.2)
                    self.pheromone_yaw       = self._wrap(self.pheromone_yaw)

            if self.search_action == 'straight':
                err          = self._wrap(self.pheromone_yaw - self.yaw)
                cmd.linear.x  = 0.35  # Increased search speed for efficiency
                cmd.angular.z = 0.6 * err + random.uniform(-0.02, 0.02)
            else:
                cmd.linear.x  = 0.0
                cmd.angular.z = self.turn_direction * random.uniform(0.8, 1.3)

        # ── APPROACH: visual servo onto green blob ────────────────────────────
        elif self.state == 'APPROACH':
            if avoid:
                self.cmd_pub.publish(avoid)
                return
            if self.green_centroid_x is not None:
                # Slow down proportionally as we get closer to the target (area increases)
                ratio = min(1.0, self.green_blob_area / COLOR_SENSOR_MIN_AREA)
                cmd.linear.x  = 0.16 * (1.0 - 0.75 * ratio)  # range: 0.16 down to 0.04 m/s
                cmd.angular.z = -1.5 * self.green_centroid_x
            else:
                self.state = 'SEARCH'

        # ── GO_TO_TARGET: navigate to peer-reported location ──────────────────
        elif self.state == 'GO_TO_TARGET':
            if avoid:
                self.cmd_pub.publish(avoid)
                return

            if self.target_x is None:
                self.state = 'SEARCH'
                return

            dx   = self.target_x - self.x
            dy   = self.target_y - self.y
            dist = math.sqrt(dx**2 + dy**2)

            if dist < 0.32:
                self.get_logger().info(f'[{self.ns}] Arrived near target ({dist:.2f}m) → STOPPED')
                self.state = 'STOPPED'
                return

            target_yaw = math.atan2(dy, dx)
            err_yaw    = self._wrap(target_yaw - self.yaw)

            if abs(err_yaw) > 0.25:
                cmd.angular.z = 1.0 if err_yaw > 0 else -1.0
            else:
                cmd.linear.x  = 0.20
                cmd.angular.z = 0.6 * err_yaw

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    bot = SwarmBot()
    rclpy.spin(bot)
    bot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
