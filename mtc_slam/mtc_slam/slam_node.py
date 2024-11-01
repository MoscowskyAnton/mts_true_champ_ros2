import rclpy
from rclpy.node import Node
from rclpy import time
import numpy as np
from mtc_msgs.msg import Sensors
#from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseArray, Pose
from threading import Lock
import time as pytime

from visualization_msgs.msg import Marker, MarkerArray

def norm_angle(value):
    return (value + np.pi) % (2*np.pi) - np.pi

def substract_angles(target, source):
    return np.arctan2(np.sin(target-source), np.cos(target-source))

def norm_pdf(a, sigma):
    return np.exp(-np.power(a, 2)/(2*np.power(sigma,2)))/(np.sqrt(2*np.pi)*sigma)

def mean_angles(angles, weights):
    x = y = 0.
    for angle, weight in zip(angles, weights):
        x += np.cos(angle) * weight
        y += np.sin(angle) * weight
    mean = np.arctan2(y, x)
    return mean

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q


class SLAM(Node):

    def __init__(self):
        super().__init__('slam')

        # labirinth params
        #self.declare_parameter('labirinth_size', rclpy.Parameter.Type.INTEGER_ARRAY)
        self.declare_parameter('labirinth_size', [0, 0])
        self.declare_parameter('cell_size', [0.0, 0.0])
        self.declare_parameter('pile_size', 0.0)

        self.cell_size = self.get_parameter('cell_size').value
        self.pile_size = self.get_parameter('pile_size').value

        lab_size = self.get_parameter('labirinth_size').value
        # score, n, x, y1, y2
        self.v_walls = np.ones((lab_size[0]+1, lab_size[1], 5), dtype = float)
        self.v_walls[:, :, 0] = -1
        self.v_walls[:, :, 1] = 0

        self.v_walls[0, :, 0] = 2
        self.v_walls[-1, :, 0] = 2

        for ix in range(self.v_walls.shape[0]):
            for iy in range(self.v_walls.shape[1]):
                self.v_walls[ix, iy, 2:] = self.get_v_wall_coords(ix, iy)

        self.h_walls = np.ones((lab_size[0], lab_size[1] + 1, 5), dtype = float)
        self.h_walls[:, :, 0] = -1
        self.h_walls[:, :, 1] = 0

        self.h_walls[:, 0, 0] = 2
        self.h_walls[:, -1, 0] = 2

        for ix in range(self.h_walls.shape[0]):
            for iy in range(self.h_walls.shape[1]):
                self.h_walls[ix, iy, 2:] = self.get_h_wall_coords(ix, iy)

        # robot params
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_yaw', 0.0)

        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.yaw = self.get_parameter('start_yaw').value

        # PF params
        self.declare_parameter('n_particles', 1)
        self.declare_parameter('wall_th', 0.5)
        #self.declare_parameter('alpha', None)
        self.declare_parameter('alpha', [.0, .0, .0, .0, .0, .0])
        self.declare_parameter('sensor_sigma', .1)

        self.declare_parameter('update_rate', 10)

        self.NP = self.get_parameter('n_particles').value
        self.P = np.ones((self.NP, 3)) * [self.x, self.y, self.yaw]
        self.W = np.ones(self.NP)

        self.alpha = self.get_parameter('alpha').value
        if len(self.alpha) != 6:
            self.get_logger().error(f'Param error: alpha len must be exactly 6!')
            exit()
        self.get_logger().info(f"alpha are {self.alpha}")

        self.last_odom = None
        self.last_odom_upd = None
        self.sensors_data = None

        # mapping params
        self.declare_parameter('on_wall_th', 10)
        self.declare_parameter('wall_clear_th', 2)

        self.on_wall_th = self.cell_size[0] / self.get_parameter('on_wall_th').value
        self.wall_clear_th = self.cell_size[0] / self.get_parameter('wall_clear_th').value

        self.odom_mutex = Lock()
        self.sens_mutex = Lock()

        # ros stuff
        self.odom_pub = self.create_publisher(Odometry, '~/odom', 10)
        self.particles_pub = self.create_publisher(PoseArray, '~/particles', 10)
        self.wall_markers_pub = self.create_publisher(MarkerArray, '~/marker_map', 10)

        self.sensors_sub = self.create_subscription(Sensors, 'sensors', self.sensors_cb,         10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 1)

        self.proc_timer = self.create_timer(1.0/self.get_parameter('update_rate').value, self.proc_cb)



    def proc_cb(self):
        now = self.get_clock().now()
        now_ = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        #self.get_logger().info(f"Process {now}")
        #with self.odom_mutex:
            #if not self.last_odom is None:
                #if not self.last_odom_upd is None:
                    #v, w, t = self.last_odom
                    #dt = now_ - t
                    #self.motion_update(v, w, dt)
                    #self.get_logger().info(f"Motion updated! {v} {w} {dt}")
                #self.last_odom_upd = now_
                #self.last_odom = None

        with self.odom_mutex:
            #with self.sens_mutex:
            if not self.sensors_data is None:
                tock = pytime.time()
                self.sensors_update(self.sensors_data)
                tick = pytime.time()
                self.get_logger().info(f'Sensor update took {tick-tock} seconds')
                self.update_map(self.sensors_data)
                self.sensors_data = None
                self.update_pose()
                self.resampling()
            else:
                self.update_pose()

            #  put vis so on
            odom_msg = Odometry()
            odom_msg.header.frame_id = "map"
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.child_frame_id = "robot"

            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y

            q = quaternion_from_euler(0, 0, self.yaw)
            odom_msg.pose.pose.orientation = Quaternion(x = q[1], y = q[2], z = q[3], w = q[0])

            self.odom_pub.publish(odom_msg)

            pose_array_msg = PoseArray()
            pose_array_msg.header = odom_msg.header
            for pn in range(self.NP):
                pose = Pose()
                pose.position.x = self.P[pn, 0]
                pose.position.y = self.P[pn, 1]
                q = quaternion_from_euler(0, 0, self.P[pn, 2])
                pose.orientation = Quaternion(x = q[1], y = q[2], z = q[3], w = q[0])
                pose_array_msg.poses.append(pose)
            self.particles_pub.publish(pose_array_msg)

        ma_msg = MarkerArray()
        z_wall = 0.2
        for i in range(self.v_walls.shape[0]):
            for j in range(self.v_walls.shape[1]):
                if self.v_walls[i, j, 0] != -1:
                    mrk = Marker()
                    mrk.header.frame_id = 'map'
                    mrk.header.stamp = now.to_msg()

                    mrk.ns = "vertical_walls"
                    mrk.id = i*100+j
                    mrk.type = Marker.CUBE
                    mrk.pose.position.x = self.v_walls[i, j, 2]
                    mrk.pose.position.y = (self.v_walls[i, j, 3] + self.v_walls[i, j, 4])/2
                    mrk.pose.position.z = z_wall/2

                    mrk.pose.orientation.w = 1.

                    if self.v_walls[i, j, 0] == 2:
                        mrk.color.r = .5
                        mrk.color.g = .0
                        mrk.color.b = .0
                        mrk.color.a = 1.
                    else:
                        mrk.color.r = 0.5
                        mrk.color.g = 0.5
                        mrk.color.b = 0.5
                        mrk.color.a = min(1., self.v_walls[i, j, 0])

                    mrk.scale.x = self.pile_size
                    mrk.scale.y = self.cell_size[1] + self.pile_size
                    mrk.scale.z = z_wall

                    ma_msg.markers.append(mrk)
        for i in range(self.h_walls.shape[0]):
            for j in range(self.h_walls.shape[1]):
                if self.h_walls[i, j, 0] != -1:
                    mrk = Marker()
                    mrk.header.frame_id = 'map'
                    mrk.header.stamp = now.to_msg()

                    mrk.ns = "horisontal_walls"
                    mrk.id = i*100+j
                    mrk.type = Marker.CUBE
                    mrk.pose.position.x = (self.h_walls[i, j, 2] + self.h_walls[i, j, 3])/2
                    mrk.pose.position.y = self.h_walls[i, j, 4]
                    mrk.pose.position.z = z_wall/2

                    mrk.pose.orientation.w = 1.

                    if self.h_walls[i, j, 0] == 2:
                        mrk.color.r = .5
                        mrk.color.g = .0
                        mrk.color.b = .0
                        mrk.color.a = 1.
                    else:
                        mrk.color.r = 0.5
                        mrk.color.g = 0.5
                        mrk.color.b = 0.5
                        mrk.color.a = min(1., self.h_walls[i, j, 0])

                    mrk.scale.x = self.cell_size[0] + self.pile_size
                    mrk.scale.y = self.pile_size
                    mrk.scale.z = z_wall

                    ma_msg.markers.append(mrk)
        self.wall_markers_pub.publish(ma_msg)

    def odom_cb(self, msg):
        #now = self.get_clock().now()
        #now_ = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9
        t = time.Time().from_msg(msg.header.stamp)
        #with self.odom_mutex:
        #self.last_odom = msg.twist.twist.linear.x, msg.twist.twist.angular.z, now_
        t_ = t.seconds_nanoseconds()[0] - t.seconds_nanoseconds()[1] * 1e-9
        if not self.last_odom is None:
            #self.last_odom = np.mean([self.last_odom[0], msg.twist.twist.linear.x]), np.mean([self.last_odom[1], msg.twist.twist.angular.z]), t_
            dt = t_ - self.last_odom[2]
            if dt > 0:
                v, w, _ = self.last_odom
                self.get_logger().info(f"Motion upd {v} {w} {dt}")
                self.motion_update(v, w, dt)
                self.last_odom = msg.twist.twist.linear.x, msg.twist.twist.angular.z, t_
        else:
            self.last_odom = msg.twist.twist.linear.x, msg.twist.twist.angular.z, t_



    def sensors_cb(self, msg):
        with self.sens_mutex:
            self.sensors_data = [[a, r] for r, a in zip(msg.rangefinders_distances, msg.rangefinders_angles)]


    def motion_update(self, v, w, dt):
        if v == 0.0 and w == 0.0:
            return
        for pn in range(self.NP):
            v_ = v + np.random.normal(0.0, self.alpha[0] * np.abs(v) + self.alpha[1] * np.abs(w))
            w_s = self.alpha[2] * np.abs(v) + self.alpha[3] * np.abs(w)
            w_add = np.random.normal(0.0, w_s)
            w_ = w + w_add
            y_ = np.random.normal(0, self.alpha[4] * np.abs(v) + self.alpha[5] * np.abs(w))
            if w_ == 0:
                self.get_logger().error(f"w_ is {w_}, v,w = {v},{w}, w_add = {w_add} w_s = {w_s}")
            v2w = v_ / w_
            th = self.P[pn, 2]
            self.P[pn, 0] = self.P[pn, 0] - v2w * np.sin(th) + v2w * np.sin(th + w_ * dt)
            self.P[pn, 1] = self.P[pn, 1] + v2w * np.cos(th) - v2w * np.cos(th + w_ * dt)
            self.P[pn, 2] = norm_angle(th + w_ * dt + y_ * dt)

    # sensors_data are
    # [[angle, range]]
    def sensors_update(self, sensors_data):
        # calc particles weights
        #print(self.P)
        for pn in range(self.NP):
            w = 0
            for sensor in sensors_data:
                a, r = sensor
                sensor_seg = [self.P[pn, 0], self.P[pn, 1], self.P[pn, 0] + r * np.cos(self.P[pn, 2] + a), self.P[pn, 1] + r * np.sin(self.P[pn, 2] + a)]
                beam = (self.P[pn, 0], self.P[pn, 1], self.P[pn, 2] + a)
                #best_wall = None
                best_wall_d = np.inf

                # v
                tau_v = (self.v_walls[:, :, 2] - beam[0])/np.cos(beam[2])
                y_v = beam[1] + np.sin(beam[2]) * tau_v

                ind_v = (self.v_walls[:, :, 0] > self.get_parameter('wall_th').value) &  (tau_v > 0) & (y_v >= self.v_walls[:, :, 3]) & (y_v <= self.v_walls[:,:, 4])
                #self.get_logger().info(ind_v.shape)

                if np.count_nonzero(ind_v) != 0:
                    best_wall_d = np.min(tau_v[ind_v])

                # h
                tau_h = (self.h_walls[:, :, 4] - beam[1])/np.sin(beam[2])
                x_h = beam[0] + np.cos(beam[2]) * tau_h

                ind_h = (self.h_walls[:, :, 0] > self.get_parameter('wall_th').value) & (tau_h > 0) & (x_h >= self.h_walls[:, :, 2]) & (x_h <= self.h_walls[:, :, 3])

                if np.count_nonzero(ind_h) != 0:
                    best_wall_d = min(best_wall_d, np.min(tau_h[ind_h]))

                # check v
                #for vx in range(self.v_walls.shape[0]):
                    #for vy in range(self.v_walls.shape[1]):
                        #if self.v_walls[vx, vy, 0] >= self.get_parameter('wall_th').value:
                            #dist = self.process_v_wall((vx, vy), beam)
                            #if dist != -1:
                                #if best_wall_d > dist:
                                    #best_wall = (vx, vy)
                                    #best_wall_d = dist
                            #wall_coords = self.get_v_wall_coords(vx, vy, True)
                            #stat, r2w, da = self.analyse_v_wall(wall_coords, sensor_seg, self.on_wall_th * 3)
                            ##print(stat, r2w)
                            #if stat == 1: # potential range on
                                #if best_wall_d > r2w:
                                    #best_wall = (vx, vy)
                                    #best_wall_d = r2w
                # check h
                #for vx in range(self.h_walls.shape[0]):
                    #for vy in range(self.h_walls.shape[1]):
                        #if self.h_walls[vx, vy, 0] >= self.get_parameter('wall_th').value:
                            ##wall_coords = self.get_h_wall_coords(vx, vy, True)
                            ##stat, r2w, da = self.analyse_h_wall(wall_coords, sensor_seg, self.on_wall_th * 3)
                            ###print(stat, r2w)
                            ##if stat == 1: # potential range on
                                ##if best_wall_d > r2w:
                                    ##best_wall = (vx, vy)
                                    ##best_wall_d = r2w
                            #dist = self.process_h_wall((vx, vy), beam)
                            #if dist != -1:
                                #if best_wall_d > dist:
                                    ##best_wall = (vx, vy)
                                    #best_wall_d = dist

                if not np.isinf(best_wall_d):
                    w += norm_pdf(best_wall_d, self.get_parameter('sensor_sigma').value)

            self.W[pn] = w / len(sensors_data)

    def update_pose(self):
        #print(self.W)
        self.W = np.nan_to_num(self.W)
        sumW = np.sum(self.W)
        if sumW == 0:
            self.get_logger.error("W is 0")
            #plt.show()

        Wnorm = self.W / sumW

        prod = Wnorm @ self.P[:,:2]
        self.x = prod[0]
        self.y = prod[1]
        self.yaw = mean_angles(self.P[:,2].tolist(), self.W.tolist())

    def resampling(self):
        # make a resamplinmg
        self.get_logger().info("Resampling")
        Wnorm = self.W / np.sum(self.W)
        indexes = np.random.choice(self.NP, size = (self.NP), p = Wnorm)
        self.P = self.P[indexes,:]
        self.W = self.W[indexes]

    def update_map(self, sensors_data):
        for sensor in sensors_data:
            a, r = sensor
            sensor_seg = [self.x, self.y, self.x + r * np.cos(self.yaw + a), self.y + r * np.sin(self.yaw + a)]
            # check v
            for vx in range(self.v_walls.shape[0]):
                for vy in range(self.v_walls.shape[1]):
                    #if self.v_walls[vx, vy] == -1:
                    if self.v_walls[vx, vy, 0] < 2:
                        wall_coords = self.get_v_wall_coords(vx, vy, False)
                        w, r2w, da = self.analyse_v_wall(wall_coords, sensor_seg, self.on_wall_th)
                        if w != -1:
                            #print(w)
                            self.v_walls[vx, vy, 1] += 1
                            if self.v_walls[vx, vy, 0] == -1:
                                self.v_walls[vx, vy, 0] = w
                            else:
                                self.v_walls[vx, vy, 0] += (w - self.v_walls[vx, vy, 0])/self.v_walls[vx, vy, 1]
            # check h
            for vx in range(self.h_walls.shape[0]):
                for vy in range(self.h_walls.shape[1]):
                    if self.h_walls[vx, vy, 0] < 2:
                        wall_coords = self.get_h_wall_coords(vx, vy, False)
                        w, r2w, da = self.analyse_h_wall(wall_coords, sensor_seg, self.on_wall_th)
                        if w != -1:
                            self.h_walls[vx, vy, 1] += 1
                            #self.h_walls[vx, vy, 0] += (w - self.h_walls[vx, vy, 0])#/self.h_walls[vx, vy, 1]
                            if self.h_walls[vx, vy, 0] == -1:
                                self.h_walls[vx, vy, 0] = w
                            else:
                                self.h_walls[vx, vy, 0] += (w - self.h_walls[vx, vy, 0])/self.h_walls[vx, vy, 1]

    def get_h_wall_coords(self, ix, iy, with_pile = True):
        xh1 = ix * (self.cell_size[0]+self.pile_size) - (self.cell_size[0]+self.pile_size) / 2
        xh2 = ix * (self.cell_size[0]+self.pile_size) + (self.cell_size[0]+self.pile_size) / 2
        yh = iy * (self.cell_size[1]+self.pile_size) - (self.cell_size[1]+self.pile_size) / 2
        if with_pile:
            return xh1, xh2, yh
        else:
            return xh1 + self.pile_size, xh2 - self.pile_size, yh



    def get_v_wall_coords(self, ix, iy, with_pile = True):
        xh = ix * (self.cell_size[0]+self.pile_size) - (self.cell_size[0]+self.pile_size) / 2
        yh1 = iy * (self.cell_size[1]+self.pile_size) - (self.cell_size[1]+self.pile_size) / 2
        yh2 = iy * (self.cell_size[1]+self.pile_size) + (self.cell_size[1]+self.pile_size) / 2
        if with_pile:
            return xh, yh1, yh2
        return xh, yh1 + self.pile_size, yh2 - self.pile_size


    # analyse how range deals with horisontal wall
    # -1 - not interacts in any way
    # 0 - goes throu
    # 1 - range on wall
    # wall_segment = (x1, x2, y)
    # range_segment = (rob_x, rob_y, range_x, range_y)
    def analyse_h_wall(self, wall_segment, range_segment, th):
        a_r = np.arctan2(range_segment[3] - range_segment[1], range_segment[2] - range_segment[0])
        a1 = np.arctan2(wall_segment[2] - range_segment[1], wall_segment[0] - range_segment[0])
        a2 = np.arctan2(wall_segment[2] - range_segment[1], wall_segment[1] - range_segment[0])

        a3 = np.arctan2(0.5*(wall_segment[2] + wall_segment[1]) - range_segment[1], wall_segment[0] - range_segment[0])

        da = substract_angles(a_r, a3)

        a1, a2 = min(a1, a2), max(a1, a2)
        if a_r >= a1 and a_r <= a2:
            range_to_wall = (range_segment[3] - wall_segment[2]) / np.cos(np.pi/2 - a_r)
            if np.abs(range_to_wall) < th:#self.on_wall_th:
                return 1, range_to_wall, da
            elif (range_segment[3] - wall_segment[2]) * np.sign(np.sin(a_r)) > self.wall_clear_th:
            #elif range_to_wall * np.sign(np.sin(a_r)) > self.wall_clear_th:
                return 0, range_to_wall, da
        return -1, None, None

    # vid = (i, j) in self.v_walls
    # beam = (px, py, alpha)
    def process_v_wall(self, vid, beam):
        px, py, alpha = beam
        x, y1, y2 = self.v_walls[vid[0], vid[1], 2:]
        tau = (x - px)/np.cos(alpha)
        if tau < 0:
            return -1
        y = py + np.sin(alpha) * tau
        if y1 <= y and y <= y2:
            #d = (x - px)/np.cos(alpha)
            return tau
        return -1

    # vid = (i, j) in self.v_walls
    # beam = (px, py, alpha)
    def process_h_wall(self, vid, beam):
        px, py, alpha = beam
        x1, x2, y = self.h_walls[vid[0], vid[1], 2:]
        tau = (y - py)/np.sin(alpha)
        if tau < 0:
            return -1
        x = px + np.cos(alpha) * tau
        if x1 <= x and x <= x2:
            #d = (y - py)/np.cos(alpha)
            return tau
        return -1


    # wall_segment = (x, y1, y2)
    def analyse_v_wall(self, wall_segment, range_segment, th):
        a_r = np.arctan2(range_segment[3] - range_segment[1], range_segment[2] - range_segment[0])
        a1 = np.arctan2(wall_segment[1] - range_segment[1], wall_segment[0] - range_segment[0])
        a2 = np.arctan2(wall_segment[2] - range_segment[1], wall_segment[0] - range_segment[0])

        a3 = np.arctan2(0.5*(wall_segment[2] + wall_segment[1]) - range_segment[1], wall_segment[0] - range_segment[0])

        da = substract_angles(a_r, a3)

        if np.abs(a3) > np.pi/2:
            if a_r < 0:
                a_r += 2*np.pi
            if a1 < 0:
                a1 += 2*np.pi
            if a2 < 0:
                a2 += 2*np.pi
            a1, a2 = a2, a1

        if a_r >= a1 and a_r <= a2:
            range_to_wall = (range_segment[2] - wall_segment[0]) / np.cos(a_r)
            if np.abs(range_to_wall) < th:#self.on_wall_th:
                return 1, range_to_wall, da
            elif (range_segment[2] - wall_segment[0]) * np.sign(np.cos(a_r)) > self.wall_clear_th:
            #elif range_to_wall * np.sign(np.cos(a_r)) > self.wall_clear_th:
                return 0, range_to_wall, da
        return -1, None, None


    def cell_to_coords(self, cell):
        x = cell[0] * (self.cell_size[0] + self.pile_size)
        y = cell[1] * (self.cell_size[1] + self.pile_size)
        return [x, y]


    def draw_map(self, ranges = None, text_prob = False):
        plt.figure("map")
        plt.cla()
        # visited cells
        # for n in range(self.was_cells.shape[0]):
        #     for m in range(self.was_cells.shape[1]):
        #         if self.was_cells[n, m] == 0:
        #             coords = self.cell_to_coords((n, m))
        #             plt.plot(coords[0], coords[1], 's', color = 'grey')
        # v walls
        for n in range(self.v_walls.shape[0]):
            for m in range(self.v_walls.shape[1]):

                xh, yh1, yh2 = self.get_v_wall_coords(n, m, True)
                if self.v_walls[n, m, 0] == -1:
                    plt.plot([xh, xh], [yh1, yh2], ':', color = 'grey')
                elif self.v_walls[n, m, 0] == 2:
                    plt.plot([xh, xh], [yh1, yh2], color = 'black')
                else:
                    alpha = max(0, min(1, round(self.v_walls[n, m, 0], 3)))
                    #print(alpha)
                    plt.plot([xh, xh], [yh1, yh2], color = 'black', alpha = alpha)
                    if text_prob:
                        plt.text(xh, (yh1+yh2)/2, f"{alpha}")

        # h walls
        for n in range(self.h_walls.shape[0]):
            for m in range(self.h_walls.shape[1]):
                xh1, xh2, yh = self.get_h_wall_coords(n, m, True)
                if self.h_walls[n, m, 0] == -1:
                    plt.plot([xh1, xh2], [yh, yh], ':', color = 'grey')
                elif self.h_walls[n, m, 0] == 2:
                    plt.plot([xh1, xh2], [yh, yh], color = 'black')
                else:
                    alpha = max(0, min(1, round(self.h_walls[n, m, 0], 3)))
                    print(self.h_walls[n, m, 0])
                    plt.plot([xh1, xh2], [yh, yh], color = 'black', alpha = alpha)
                    if text_prob:
                        plt.text((xh1 + xh2)/2, yh, f"{alpha}")

        # localization
        max_lenght = self.cell_size[0] / 3
        max_w = np.max(self.W)
        scale = max_lenght / max_w
        color = 'orange'#'cyan'
        for p in range(self.P.shape[0]):
            lenght = self.W[p] * scale
            #plt.arrow(self.P[p,0], self.P[p,1], np.cos(self.P[p,2])*lenght, np.sin(self.P[p,2])*lenght, color = color, shape = 'full', head_width=0.1, alpha = 0.5)
            plt.plot(self.P[p,0], self.P[p,1], '.', color = color, alpha = self.W[p] / max_w)

        plt.arrow(self.x, self.y, max_lenght * np.cos(self.yaw), max_lenght* np.sin(self.yaw), color = 'red')
        plt.plot(self.x, self.y, '.r')

        if not ranges is None:
            for sens_range in ranges:
                a, r = sens_range
                x = self.x + r * np.cos(a + self.yaw)
                y = self.y + r * np.sin(a + self.yaw)
                plt.plot(x, y, '*r')

        plt.title(f"Map")
        plt.gca().set_aspect('equal', adjustable='box')


def main(args = None):
    rclpy.init(args = args)

    slam = SLAM()
    rclpy.spin(slam)

    slam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
