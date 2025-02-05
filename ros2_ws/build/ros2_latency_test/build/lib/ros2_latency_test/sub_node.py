#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String


class LatencySubNode(Node):
    def __init__(self):
        super().__init__('latency_sub_node')
        
        # 서브스크라이버 생성
        self.subscription_ = self.create_subscription(
            String,
            'latency_test',
            self.sub_callback,
            10
        )
        
        # 레이턴시 누적 리스트
        self.latencies = []
        # 100번까지만 측정
        self.max_count = 100

    def sub_callback(self, msg):
        # 수신 시각(초 단위)
        now = self.get_clock().now().nanoseconds / 1e9
        

        try:
            idx = msg.data.index(";")
            pub_time_str = msg.data[:idx]  # 세미콜론 앞까지가 발행 시각
            pub_time = float(pub_time_str)
        except Exception as e:
            self.get_logger().error(f"Error parsing timestamp: {e}")
            return

        latency = now - pub_time
        self.latencies.append(latency)

        self.get_logger().info(f"Received {len(msg.data) - (idx + 1)} bytes, latency={latency*1000:.3f} ms")

        # 100개 수신 후 결과 출력하고 노드 종료
        if len(self.latencies) >= self.max_count:
            avg_latency = sum(self.latencies) / len(self.latencies)
            max_latency = max(self.latencies)
            self.get_logger().info("--- Measurement Results ---")
            self.get_logger().info(f"Average latency: {avg_latency*1000:.3f} ms")
            self.get_logger().info(f"Max latency:     {max_latency*1000:.3f} ms")

            # 종료
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LatencySubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()