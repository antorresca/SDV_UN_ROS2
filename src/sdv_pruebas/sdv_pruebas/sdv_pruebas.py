import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class SdvPruebas(Node):
    
    def __init__(self):
        super().__init__('sdv_pruebas')
        self.pub = self.create_publisher(
            String,
            '/vel2cmd',
            10)
        timer_sec = 0.1 #Verificar (Con 0.1 funciona adecuadamente al colocar 'mt 1' en la tiva)
        self.timer = self.create_timer(timer_sec, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        vel = 20
        if self.i >= 0 and self.i<50:
            msg.data = "m 1 -"+str(vel)+" -"+str(vel)
        elif self.i >=50 and self.i<100:
            msg.data = "m 1 "+str(vel)+" "+str(vel)
        self.pub.publish(msg)
        if self.i == 99:
            self.i = 0
        else:
            self.i += 1


def main(arg=None):
    rclpy.init(args=arg)

    sdv_pruebas = SdvPruebas()

    rclpy.spin(sdv_pruebas)

    sdv_pruebas.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

