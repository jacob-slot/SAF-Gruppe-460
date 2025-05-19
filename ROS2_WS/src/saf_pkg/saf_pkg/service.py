from interfaces.srv import ProcessingTimeService
from std_msgs.msg import String
import json
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(ProcessingTimeService, 'get_processing_time', self.service_callback)
        # Create a publisher to broadcast service activity
        self.publisher = self.create_publisher(String, 'processing_time_events', 10)

    def service_callback(self, request, response):
        
        # open processing_times_table.csv with separator ;, and find the processing time for the column nr. station_id and line nr. carrier_id
        with open("processing_times_table.csv", "r") as f:
            lines = f.readlines()
            processing_time = int(lines[request.carrier_id].split(';')[request.station_id])

        self.get_logger().info('Incoming request\na: %d b: %d' % (request.carrier_id, request.station_id))
        self.get_logger().info('Processing time: %d' % processing_time)
        
        # Publish service activity as a JSON string
        event_msg = String()
        event_data = {
            'event_type': 'service_call',
            'carrier_id': request.carrier_id,
            'station_id': request.station_id,
            'processing_time': processing_time,
            'timestamp': self.get_clock().now().to_msg().sec
        }
        event_msg.data = json.dumps(event_data)
        self.publisher.publish(event_msg)

        response.processing_time = processing_time
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()