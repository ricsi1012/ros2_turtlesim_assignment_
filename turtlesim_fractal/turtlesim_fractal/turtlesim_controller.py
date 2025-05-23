import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_client   = self.create_client(SetPen,          '/turtle1/set_pen')


    def call_service(self, client, req):
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r, req.g, req.b = r, g, b
        req.width, req.off   = width, off
        self.call_service(self.set_pen_client, req)

    def teleport(self, x, y, theta_deg=0.0):
        req = TeleportAbsolute.Request()
        req.x, req.y = x, y
        req.theta    = math.radians(theta_deg)
        self.call_service(self.teleport_client, req)

    def draw_line(self, x1, y1, x2, y2, color, width=3):
        self.set_pen(0,0,0,0,1)
        self.teleport(x1, y1)
        r, g, b = color
        self.set_pen(r, g, b, width, 0)
        self.teleport(x2, y2)
        self.set_pen(0,0,0,0,1)

    def draw_R(self, sx, sy):
        self.draw_line(sx,   sy,   sx,   sy+6,   (255,0,0))
        self.draw_line(sx,   sy+6, sx+2, sy+6,   (255,0,0))
        self.draw_line(sx+2, sy+6, sx+2, sy+4,   (255,0,0))
        self.draw_line(sx+2, sy+4, sx,   sy+4,   (255,0,0))
        self.draw_line(sx,   sy+4, sx+2, sy,     (255,0,0))

    def draw_O(self, sx, sy):
        self.draw_line(sx,   sy,   sx,   sy+6, (0,255,0))
        self.draw_line(sx,   sy+6, sx+2, sy+6, (0,255,0))
        self.draw_line(sx+2, sy+6, sx+2, sy,   (0,255,0))
        self.draw_line(sx+2, sy,   sx,   sy,   (0,255,0))

    def draw_S(self, sx, sy):

        self.draw_line(sx+2, sy+6, sx,   sy+6, (0,0,255))
        self.draw_line(sx,   sy+6, sx,   sy+3, (0,0,255))
        self.draw_line(sx,   sy+3, sx+2, sy+3, (0,0,255))
        self.draw_line(sx+2, sy+3, sx+2, sy,   (0,0,255))
        self.draw_line(sx+2, sy,   sx,   sy,   (0,0,255))

    def main(self):
   
        self.draw_R(1.0, 2.0)
        self.draw_O(4.0, 2.0)
        self.draw_S(7.0, 2.0)
        rclpy.spin(self)

def main():
    rclpy.init()
    controller = TurtleController()
    controller.main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
