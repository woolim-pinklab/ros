from my_first_package_msgs.srv import MultiSpawn 

import rclpy as rp
from rclpy.node import Node


class MultiSpawning(Node):

    def __init__(self):
        super().__init__('multi_spawn')
        self.server = self.create_service(MultiSpawn, 'multi_spawn', self.callback_service)  

    def callback_service(self, request, response):
        print("Request : ", request)

        response.x = [1.0, 2.0, 3.0]
        response.y = [10.0, 20.0]
        response.theta = [100.0, 200.0, 300.0]

        return response

def main(args=None):
    rp.init(args=args)
    multi_spawn = MultiSpawning()
    rp.spin(multi_spawn)
    rp.shutdown()

if __name__ == '__main__':
    main()