import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import struct
import socket
import math

class YdlidarNode(Node):
    def __init__(self):
        super().__init__('ydlidar_node')

        self.declare_parameter('host', '0.0.0.0')  # 监听所有接口
        self.declare_parameter('socket_port', 8889)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('angle_max', 180.0)
        self.declare_parameter('angle_min', -180.0)
        self.declare_parameter('max_range', 16.0)
        self.declare_parameter('min_range', 0.01)
        self.declare_parameter('full_scan_threshold', 180.0)
        
        self.data_buffer = bytearray()
        self._init_udp()
        
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.get_parameter('frame_id').value
        self.scan_msg.range_min = self.get_parameter('min_range').value
        self.scan_msg.range_max = self.get_parameter('max_range').value
        self.current_angle = 0.0
        self.offset_angle = 3/180*math.pi

        self.scan_count = 0
        self.last_report_time = self.get_clock().now()
        self.last_report_count = 0

        self.lidar_type = 0
        self.full_scan_buffer = []
        self.current_angle = 0.0
        self.is_new_scan = True

        self.count = 0

        self.scan_complete_threshold = math.radians(self.get_parameter('full_scan_threshold').value)
        self.create_timer(0.001, self.process_udp)

    def _init_udp(self):
        host = self.get_parameter('host').value
        port = self.get_parameter('socket_port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.get_logger().info(f"等待激光雷达UDP数据: {host}:{port}")

    def _verify_checksum(self, data, Lidar_type):
        if Lidar_type == 'LidarF2':
            check_data = bytearray(data[0:8])

            si_data = data[10:]  
            for i in range(0, len(si_data), 3):
                si1 = si_data[i]
                si2 = si_data[i+1]
                si3 = si_data[i+2]
                
                check_data.extend(struct.pack('<H', si1))
                
                distance_word = (si3 << 8) | si2
                check_data.extend(struct.pack('<H', distance_word))
            if len(check_data) % 2 != 0:
                check_data += b'\x00'
                
            checksum = 0
            for i in range(0, len(check_data), 2):
                word = struct.unpack('<H', check_data[i:i+2])[0]
                checksum ^= word
                
            return checksum == struct.unpack('<H', data[8:10])[0]
        elif Lidar_type == 'LidarX2':
            check_data = data[0:8] + data[10:]
            if len(check_data) % 2 != 0:
                check_data += b'\x00'
                
            checksum = 0
            for i in range(0, len(check_data), 2):
                word = struct.unpack('<H', check_data[i:i+2])[0]
                checksum ^= word
                
            return checksum == struct.unpack('<H', data[8:10])[0]
    
    def parse_packet(self, data, Lidar_type):
        if Lidar_type == 'LidarF2':
            if data[0:2] != b'\xAA\x55':
                raise ValueError("无效数据包头")
                
            ct = data[2]
            lsn = data[3]
            fsa = struct.unpack('<H', data[4:6])[0]
            lsa = struct.unpack('<H', data[6:8])[0]
            
            start_angle = (fsa >> 1) / 64.0
            end_angle = (lsa >> 1) / 64.0
            
            samples = []
            for i in range(lsn):
                offset = 10 + i*3
                intensity_byte = data[offset] 
                s2 = data[offset+1]
                s3 = data[offset+2] 
                distance_raw = (s3 << 6) + (s2 >> 2)
                distance = distance_raw / 1000.0
                if lsn > 1:
                    angle_diff = (end_angle - start_angle + 360) % 360
                    angle = start_angle + angle_diff * i / (lsn - 1)
                else:
                    angle = start_angle
                    
                samples.append({
                    'distance': distance,
                    'angle': angle % 360
                })
            
            return {
                'start_angle': start_angle,
                'end_angle': end_angle,
                'sample_count': lsn,
                'samples': samples
            }
        if Lidar_type == 'LidarX2':
            if data[0:2] != b'\xAA\x55':
                raise ValueError("无效数据包头")
            ct = data[2]
            lsn = data[3]
            fsa = struct.unpack('<H', data[4:6])[0]
            lsa = struct.unpack('<H', data[6:8])[0]
            
            start_angle = (fsa >> 1) / 64.0
            end_angle = (lsa >> 1) / 64.0
            
            samples = []
            for i in range(lsn):
                offset = 10 + i*2
                sample = struct.unpack('<H', data[offset:offset+2])[0]
                distance = sample/4/ 1000.0
                if lsn > 1:
                    angle_diff = (end_angle - start_angle + 360) % 360 
                    angle = start_angle + (end_angle - start_angle) * i / (lsn - 1)
                else:
                    angle = start_angle
                    
                samples.append({
                    'distance': distance,
                    'angle': angle % 360
                })
            
            return {
                'start_angle': start_angle,
                'end_angle': end_angle,
                'sample_count': lsn,
                'samples': samples
            }
    
    def process_udp(self):
        try:
            try:
                while True:
                    data, addr = self.sock.recvfrom(4096)
                    if data:
                        self.data_buffer += data
            except BlockingIOError:
                pass

            header_pos = self.data_buffer.find(b'\xAA\x55')
            if header_pos < 0:
                return
            if header_pos > 0:
                self.data_buffer = self.data_buffer[header_pos:]
                
            next_header_pos = self.data_buffer.find(b'\xAA\x55', header_pos + 2)
            if next_header_pos < 0:
                return
            lsn = self.data_buffer[3]
            data_between_headers = self.data_buffer[header_pos:next_header_pos]
            
            if self.lidar_type == 0:
                if lsn == 1:
                    bytes_between_headers = next_header_pos - header_pos
                    Si_size = (bytes_between_headers-10)/lsn
                    
                    if Si_size == 3:
                        self.lidar_type = 'LidarF2'
                        if self._verify_checksum(data_between_headers, self.lidar_type):
                            self.get_logger().info(f"雷达F2数据校验正确,开始解析数据")
                            self.offset_angle = 3/180*math.pi
                        else:
                            self.lidar_type = 0
                            return
                        
                    if Si_size == 2:
                        self.lidar_type = 'LidarX2'
                        if self._verify_checksum(data_between_headers, self.lidar_type):
                            self.get_logger().info(f"雷达X2数据校验正确,开始解析数据")
                            self.offset_angle = 15/180*math.pi
                        else:
                            self.lidar_type = 0
                            return
                else:
                    if next_header_pos > 0:
                        self.data_buffer = self.data_buffer[next_header_pos:]
                    return

            self.count = self.count + lsn
            if self.lidar_type == 'LidarF2':
                packet_len = 10 + 3 * lsn
            elif self.lidar_type == 'LidarX2':
                packet_len = 10 + 2 * lsn
            else:
                return
            if len(self.data_buffer) < packet_len:
                return
            
            packet = self.data_buffer[:packet_len]
            try:
                if self._verify_checksum(packet, self.lidar_type):
                    data = self.parse_packet(packet, self.lidar_type)
                    if next_header_pos > 0:
                        self.data_buffer = self.data_buffer[next_header_pos:]
                    self._publish_scan(data)
                else:
                    self.count = 0
                    if next_header_pos > 0:
                        self.data_buffer = self.data_buffer[next_header_pos:]
            except Exception as e:
                self.get_logger().warn(f"数据包解析失败: {str(e)}")
                    
        except Exception as e:
            self.get_logger().error(f"数据处理异常: {str(e)}")

    def _publish_scan(self, data):
        self.full_scan_buffer.extend(data['samples'])
        self.current_angle = data['end_angle']
        angle_diff = data['start_angle'] - self.current_angle
        if angle_diff > math.pi:
            self._publish_full_scan()
            self.full_scan_buffer = []
            self.is_new_scan = True

    def _publish_full_scan(self):
        if len(self.full_scan_buffer) < 100:
            return
        
        self.full_scan_buffer.reverse()

        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_msg.angle_min = -math.pi-self.offset_angle
        self.scan_msg.angle_max = math.pi-self.offset_angle
        self.scan_msg.time_increment = 0.0003
        self.scan_msg.scan_time = 0.156
        self.scan_msg.angle_increment = 2 * math.pi / len(self.full_scan_buffer)
        self.scan_msg.ranges = [s['distance'] for s in self.full_scan_buffer]
        
        self.publisher.publish(self.scan_msg)
        self.full_scan_buffer.clear()
        
        # 发布速率统计
        self.scan_count += 1
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_report_time).nanoseconds / 1e9
        
        if time_diff >= 3.0:
            scan_diff = self.scan_count - self.last_report_count
            rate = scan_diff / time_diff
            self.get_logger().info(f"/scan话题平均发布速率: {rate:.2f} Hz")
            self.last_report_time = current_time
            self.last_report_count = self.scan_count

def main(args=None):
    rclpy.init(args=args)
    node = YdlidarNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()