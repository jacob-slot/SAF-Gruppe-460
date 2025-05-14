import sys
import socket
from interfaces.srv import ProcessingTimeService
import rclpy
from rclpy.node import Node

host='172.20.66.159'
port=10

class Client(Node):

    def __init__(self):
        super().__init__('client_node')

        self.client = self.create_client(ProcessingTimeService, 'get_processing_time')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.request = ProcessingTimeService.Request()

        # start TCP server
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            print(f"Server listening on {host}:{port}")
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                while True:
                    
                    # if connection is broken, break
                    if not conn:
                        print("Connection broken")
                        break

                    # receive string data from the client and save as xml file
                    data = conn.recv(1024)
                    
                    # remove all characters after </data> tag
                    data = data[:data.find(b'</data>')+7]
                    
                    print("Received:", data)
                    with open("data.xml", "wb") as f:
                        f.write(data)
                    
                    station_id = int(data[data.find(b'<station_id>')+12:data.find(b'</station_id>')].decode('utf-8'))
                    print("Station ID:", station_id)

                    carrier_id = int(data[data.find(b'<carrier_id>')+12:data.find(b'</carrier_id>')].decode('utf-8'))
                    print("Carrier ID:", carrier_id)

                    # send request to service
                    processing_time = self.send_request(carrier_id, station_id)
                    
                    conn.sendall((processing_time).to_bytes(2, byteorder='little', signed=True))




    def send_request(self, carrier_id, station_id):
        self.request.carrier_id = carrier_id
        self.request.station_id = station_id
        return self.client.call_async(self.request)


def main():
    rclpy.init()

    client_node = Client()
    
    rclpy.spin(client_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()