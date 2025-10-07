import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class GoToNode(Node):
    def __init__(self):
        super().__init__('p2p_move')

        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.declare_parameter('cmd_vel_topic', '/crazyflie/cmd_vel')
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.5)
        self.declare_parameter('goal_z', 0.5)
        self.declare_parameter('tolerance', 0.05)   

        self.declare_parameter('k_z', 0.8)            
        self.declare_parameter('k_pitch_roll', 0.2)              
        self.declare_parameter('max_v_z', 0.4)          
        self.declare_parameter('max_tilt_rad', 0.1)   
        self.declare_parameter('brake_dist', 0.2)       
        self.declare_parameter('rate_hz', 50.0)          

        self.goal = (
            float(self.get_parameter('goal_x').value),
            float(self.get_parameter('goal_y').value),
            float(self.get_parameter('goal_z').value),
        )
        self.tol = float(self.get_parameter('tolerance').value)
        self.k_z = float(self.get_parameter('k_z').value)
        self.k_pitch_roll  = float(self.get_parameter('k_pitch_roll').value)
        self.max_v_z = float(self.get_parameter('max_v_z').value)
        self.max_tilt_rad  = float(self.get_parameter('max_tilt_rad').value)
        self.rate_hz  = float(self.get_parameter('rate_hz').value)

        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic  = self.get_parameter('cmd_vel_topic').value

        self.sub = self.create_subscription(Odometry, odom_topic, self.cb_odom, 10)
        self.pub = self.create_publisher(Twist, cmd_topic, 10)

        self.pos = None  # (x, y, z)
        self.timer = self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.do_control)

        self.get_logger().info(f'Move started. goal={self.goal}')

    def cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)

    def do_control(self):
        if self.pos is None:
            return

        gx, gy, gz = self.goal
        x, y, z = self.pos
        ex, ey, ez = gx - x, gy - y, gz - z

        dist_xy = math.hypot(ex, ey)
        z_done = (abs(ez) <= self.tol)
        xy_done = (dist_xy <= self.tol) 
        tw = Twist()

        if not z_done:
            vz = clamp(self.k_z  * ez, -self.max_v_z , self.max_v_z )
            tw.linear.z = vz
            tw.angular.x = 0.0
            tw.angular.y = 0.0
            tw.angular.z = 0.0
            self.pub.publish(tw)
            return
        
        if not xy_done:
            pitch_cmd = clamp(self.k_pitch_roll * ex, -self.max_tilt_rad, self.max_tilt_rad)  
            roll_cmd  = clamp(self.k_pitch_roll * ey, -self.max_tilt_rad, self.max_tilt_rad)
            if dist_xy < self.brake_dist:
                pitch_cmd *= 0.3
                roll_cmd  *= 0.3      
             
            tw.linear.z = 0.0
            tw.angular.x = pitch_cmd
            tw.angular.y = roll_cmd
            tw.angular.z = 0.0
        
        else:
            tw.linear.z = 0.0
            tw.angular.x = 0.0
            tw.angular.y = 0.0
            tw.angular.z = 0.0
            self.get_logger().info('Goal reached. Holding position.')

        self.pub.publish(tw)


def main():
    rclpy.init()
    node = GoToNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
