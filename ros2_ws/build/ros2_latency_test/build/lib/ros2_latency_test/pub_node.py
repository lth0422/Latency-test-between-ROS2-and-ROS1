#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Float64

class LatencyPubNode(Node):
    def __init__(self):
        super().__init__('latency_pub_node')
        # 퍼블리셔 생성
        self.pub_ = self.create_publisher(String, 'latency_test', 10)
        
        # 12Hz 주기로 발행할 타이머 생성
        timer_period = 1.0 / 12.0  # 12Hz
        self.timer_ = self.create_timer(timer_period, self.timer_callback)

        self.count = 0
    def timer_callback(self):
        # 현재 시각(초 단위, float) 가져오기
        now = self.get_clock().now().nanoseconds / 1e9
        

        # msg = Float64()
        # msg.data = now  # 퍼블리시 시점 기록
        # self.publisher_.publish(msg)

        # self.get_logger().info(f'Publishing timestamp: {now:.6f} s')

         # 전송할 큰 데이터(1MB)
        data_size = 1024 * 1024 # 1MB
        large_payload = "0" * data_size

        # 메시지 안에 "타임스탬프 + 구분자 + 대용량 문자열"을 넣음
        # 수신 노드는 맨 앞의 타임스탬프를 파싱해 레이턴시를 구할 것
        msg_str = f"{now};{large_payload}"
        
        msg = String()
        msg.data = msg_str

        self.pub_.publish(msg)
        self.count += 1

        self.get_logger().info(f"Published {data_size} bytes, count={self.count}, time={now:.6f}")

def main(args=None):
    rclpy.init(args=args)
    node = LatencyPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
