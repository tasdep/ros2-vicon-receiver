from collections import deque
from dataclasses import dataclass
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


@dataclass(frozen=True)
class PoseSample:
    stamp_s: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class PoseVelocityEstimator(Node):
    def __init__(self) -> None:
        super().__init__("pose_velocity_estimator")

        self.declare_parameter("pose_topic", "/vicon/drone/drone")
        self.declare_parameter("velocity_topic", "/vicon/drone/velocity")
        self.declare_parameter("method", "lowpass_fd")
        self.declare_parameter("alpha", 0.35)
        self.declare_parameter("window_size", 7)
        self.declare_parameter("min_dt", 0.001)
        self.declare_parameter("max_dt", 0.2)

        self.pose_topic = self.get_parameter("pose_topic").value
        self.velocity_topic = self.get_parameter("velocity_topic").value
        self.method = self.get_parameter("method").value
        self.alpha = float(self.get_parameter("alpha").value)
        self.window_size = int(self.get_parameter("window_size").value)
        self.min_dt = float(self.get_parameter("min_dt").value)
        self.max_dt = float(self.get_parameter("max_dt").value)

        if self.method not in {"finite_difference", "lowpass_fd", "window_regression"}:
            raise ValueError(
                "method must be one of: finite_difference, lowpass_fd, window_regression"
            )
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("alpha must be in the range (0, 1]")
        if self.window_size < 2:
            raise ValueError("window_size must be at least 2")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=max(10, self.window_size),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.samples: deque[PoseSample] = deque(maxlen=self.window_size)
        self.filtered_velocity: tuple[float, float, float, float, float, float] | None = None

        self.publisher = self.create_publisher(TwistStamped, self.velocity_topic, qos)
        self.subscription = self.create_subscription(
            PoseStamped, self.pose_topic, self.receive_pose, qos
        )

        self.get_logger().info(
            f"Estimating velocity from {self.pose_topic} -> "
            f"{self.velocity_topic} using method={self.method}"
        )

    def receive_pose(self, msg: PoseStamped) -> None:
        stamp_s = self.stamp_to_seconds(msg)
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds * 1e-9

        sample = PoseSample(
            stamp_s=stamp_s,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            qx=msg.pose.orientation.x,
            qy=msg.pose.orientation.y,
            qz=msg.pose.orientation.z,
            qw=msg.pose.orientation.w,
        )
        self.samples.append(sample)

        linear_velocity, angular_velocity = self.estimate_velocity()
        if linear_velocity is None or angular_velocity is None:
            return

        out = TwistStamped()
        out.header = msg.header
        out.twist.linear.x = linear_velocity[0]
        out.twist.linear.y = linear_velocity[1]
        out.twist.linear.z = linear_velocity[2]
        out.twist.angular.x = angular_velocity[0]
        out.twist.angular.y = angular_velocity[1]
        out.twist.angular.z = angular_velocity[2]
        self.publisher.publish(out)

    def stamp_to_seconds(self, msg: PoseStamped) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    def estimate_velocity(
        self,
    ) -> tuple[tuple[float, float, float] | None, tuple[float, float, float] | None]:
        if len(self.samples) < 2:
            return None, None

        if self.method == "window_regression":
            linear_velocity = self.window_regression_velocity()
            angular_velocity = self.window_regression_angular_velocity()
        else:
            linear_velocity = self.finite_difference_velocity()
            angular_velocity = self.angular_velocity()

        if linear_velocity is None or angular_velocity is None:
            return None, None

        if self.method == "lowpass_fd":
            linear_velocity, angular_velocity = self.lowpass(
                linear_velocity, angular_velocity
            )

        return linear_velocity, angular_velocity

    def finite_difference_velocity(self) -> tuple[float, float, float] | None:
        previous = self.samples[-2]
        current = self.samples[-1]
        dt = current.stamp_s - previous.stamp_s
        if dt < self.min_dt or dt > self.max_dt:
            self.filtered_velocity = None
            return None
        return (
            (current.x - previous.x) / dt,
            (current.y - previous.y) / dt,
            (current.z - previous.z) / dt,
        )

    def angular_velocity(self) -> tuple[float, float, float] | None:
        previous = self.samples[-2]
        current = self.samples[-1]
        dt = current.stamp_s - previous.stamp_s
        if dt < self.min_dt or dt > self.max_dt:
            self.filtered_velocity = None
            return None

        q_prev = self.normalize_quaternion(
            (previous.qw, previous.qx, previous.qy, previous.qz)
        )
        q_curr = self.normalize_quaternion(
            (current.qw, current.qx, current.qy, current.qz)
        )
        if q_prev is None or q_curr is None:
            return None

        if self.quaternion_dot(q_prev, q_curr) < 0.0:
            q_curr = tuple(-component for component in q_curr)

        dq = self.quaternion_multiply(self.quaternion_conjugate(q_prev), q_curr)
        dq = self.normalize_quaternion(dq)
        if dq is None:
            return None

        return self.rotation_vector_from_quaternion(dq, dt)

    def lowpass(
        self,
        linear_velocity: tuple[float, float, float],
        angular_velocity: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        velocity = linear_velocity + angular_velocity
        if self.filtered_velocity is None:
            self.filtered_velocity = velocity
            return linear_velocity, angular_velocity

        a = self.alpha
        self.filtered_velocity = tuple(
            a * value + (1.0 - a) * previous
            for value, previous in zip(velocity, self.filtered_velocity)
        )
        return self.filtered_velocity[:3], self.filtered_velocity[3:]

    def window_regression_velocity(self) -> tuple[float, float, float] | None:
        samples = self.samples_in_valid_window(list(self.samples))
        if len(samples) < 2:
            return None

        times = [sample.stamp_s - samples[-1].stamp_s for sample in samples]
        return (
            self.linear_slope(times, [sample.x for sample in samples]),
            self.linear_slope(times, [sample.y for sample in samples]),
            self.linear_slope(times, [sample.z for sample in samples]),
        )

    def window_regression_angular_velocity(self) -> tuple[float, float, float] | None:
        samples = self.samples_in_valid_window(list(self.samples))
        if len(samples) < 2:
            return None

        reference = samples[-1]
        q_ref = self.normalize_quaternion(
            (reference.qw, reference.qx, reference.qy, reference.qz)
        )
        if q_ref is None:
            return None

        times = []
        rotation_vectors = []
        for sample in samples:
            q_sample = self.normalize_quaternion(
                (sample.qw, sample.qx, sample.qy, sample.qz)
            )
            if q_sample is None:
                return None
            if self.quaternion_dot(q_ref, q_sample) < 0.0:
                q_sample = tuple(-component for component in q_sample)

            q_rel = self.quaternion_multiply(self.quaternion_conjugate(q_ref), q_sample)
            q_rel = self.normalize_quaternion(q_rel)
            if q_rel is None:
                return None
            rotation_vectors.append(self.quaternion_to_rotation_vector(q_rel))
            times.append(sample.stamp_s - reference.stamp_s)

        return (
            self.linear_slope(times, [vector[0] for vector in rotation_vectors]),
            self.linear_slope(times, [vector[1] for vector in rotation_vectors]),
            self.linear_slope(times, [vector[2] for vector in rotation_vectors]),
        )

    def samples_in_valid_window(self, samples: list[PoseSample]) -> list[PoseSample]:
        t0 = samples[-1].stamp_s
        times = [sample.stamp_s - t0 for sample in samples]

        if times[-1] - times[0] < self.min_dt:
            return []
        if times[-1] - times[0] > self.max_dt:
            samples = [sample for sample in samples if t0 - sample.stamp_s <= self.max_dt]
            if len(samples) < 2:
                return []
        return samples

    def linear_slope(self, times: list[float], values: list[float]) -> float:
        mean_t = sum(times) / len(times)
        mean_v = sum(values) / len(values)
        denominator = sum((t - mean_t) ** 2 for t in times)
        if math.isclose(denominator, 0.0):
            return 0.0
        numerator = sum((t - mean_t) * (v - mean_v) for t, v in zip(times, values))
        return numerator / denominator

    def normalize_quaternion(
        self, q: tuple[float, float, float, float]
    ) -> tuple[float, float, float, float] | None:
        norm = math.sqrt(sum(component * component for component in q))
        if norm < 1e-12:
            return None
        return tuple(component / norm for component in q)

    def quaternion_dot(
        self,
        a: tuple[float, float, float, float],
        b: tuple[float, float, float, float],
    ) -> float:
        return sum(component_a * component_b for component_a, component_b in zip(a, b))

    def quaternion_conjugate(
        self, q: tuple[float, float, float, float]
    ) -> tuple[float, float, float, float]:
        return (q[0], -q[1], -q[2], -q[3])

    def quaternion_multiply(
        self,
        a: tuple[float, float, float, float],
        b: tuple[float, float, float, float],
    ) -> tuple[float, float, float, float]:
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    def quaternion_to_rotation_vector(
        self, q: tuple[float, float, float, float]
    ) -> tuple[float, float, float]:
        w, x, y, z = q
        if w < 0.0:
            w, x, y, z = -w, -x, -y, -z
        vector_norm = math.sqrt(x * x + y * y + z * z)
        if vector_norm < 1e-12:
            return (0.0, 0.0, 0.0)
        angle = 2.0 * math.atan2(vector_norm, max(min(w, 1.0), -1.0))
        scale = angle / vector_norm
        return (x * scale, y * scale, z * scale)

    def rotation_vector_from_quaternion(
        self, q: tuple[float, float, float, float], dt: float
    ) -> tuple[float, float, float]:
        rotation_vector = self.quaternion_to_rotation_vector(q)
        return tuple(component / dt for component in rotation_vector)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseVelocityEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
