
import numpy as np
from geometry_msgs.msg import Pose, Quaternion, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry

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


def skukozjit_walls(walls, score = 0.5):
    scores = walls[:, :, 0]
    th = scores.copy().astype(np.int8)
    th[scores >= score] = 1
    th[scores < score] = 0
    th[scores == -1] = -1
    return th.flatten().tolist()
    
def raskukozjit_h_walls(walls, cell_n):    
    return np.array(walls).reshape((cell_n, cell_n+1))

def raskukozjit_v_walls(walls, cell_n):    
    return np.array(walls).reshape((cell_n+1, cell_n))


class SLAM(object):
    
    def __init__(self, lab_size, cell_size, pile_size, start_pose, start_pose_sigmas, n_particels, wall_th, alpha, range_sigma, on_wall_th, wall_clear_th):
        
        self.print_func = print

        self.cell_size = cell_size
        self.pile_size = pile_size
        
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
                
                
        self.x = start_pose[0]
        self.y = start_pose[1]
        self.yaw = start_pose[2]
        
        self.wall_th = wall_th
        self.range_sigma = range_sigma
        
        self.NP = n_particels
        self.init_pf(start_pose, start_pose_sigmas)
        
        if len(alpha) != 6:
            #self.get_logger().error(f'Param error: alpha len must be exactly 6!')
            exit()
        self.alpha = alpha
        
        self.on_wall_th = on_wall_th
        self.wall_clear_th = wall_clear_th
        
    def init_pf(self, start_pose, start_pose_sigmas):
        self.P = np.ones((self.NP, 3))
        
        self.P[:, 0] = start_pose[0] + np.random.normal(0, start_pose_sigmas[0])
        self.P[:, 1] = start_pose[1] + np.random.normal(0, start_pose_sigmas[1])
        self.P[:, 2] = norm_angle(start_pose[2] + np.random.normal(0, start_pose_sigmas[2]))
        
        self.W = np.ones(self.NP)
        
    def motion_update(self, v, w, dt):
        if v == 0.0 and w == 0.0:
            return
        for pn in range(self.NP):
            v_ = v + np.random.normal(0.0, self.alpha[0] * np.abs(v) + self.alpha[1] * np.abs(w))
            w_s = self.alpha[2] * np.abs(v) + self.alpha[3] * np.abs(w)
            w_add = np.random.normal(0.0, w_s)
            w_ = w + w_add
            y_ = np.random.normal(0, self.alpha[4] * np.abs(v) + self.alpha[5] * np.abs(w))
            #if w_ == 0:
                #self.get_logger().error(f"w_ is {w_}, v,w = {v},{w}, w_add = {w_add} w_s = {w_s}")
            v2w = v_ / w_
            th = self.P[pn, 2]
            self.P[pn, 0] = self.P[pn, 0] - v2w * np.sin(th) + v2w * np.sin(th + w_ * dt)
            self.P[pn, 1] = self.P[pn, 1] + v2w * np.cos(th) - v2w * np.cos(th + w_ * dt)
            self.P[pn, 2] = norm_angle(th + w_ * dt + y_ * dt)

    def motion_update_step(self, dr, dyaw):
        dx = dr * np.cos(self.P[:, 2])
        dy = dr * np.sin(self.P[:, 2])

        self.P[:,0] += dx + np.random.normal(0, self.alpha[0] * np.abs(dx) + self.alpha[1] * np.abs(dyaw), self.NP)
        self.P[:,1] += dy + np.random.normal(0, self.alpha[2] * np.abs(dy) + self.alpha[3] * np.abs(dyaw), self.NP)
        self.P[:,2] += dyaw + np.random.normal(0, self.alpha[4] * np.abs(dr) + self.alpha[5] * np.abs(dyaw), self.NP)


    # sensors_data are
    # [[angle, range]]
    def sensors_update(self, sensors_data, sensor_w = None):
        # calc particles weights
        #print(self.P)
        #best_walls = []
        for pn in range(self.NP):
            w = 0
            for i, sensor in enumerate(sensors_data):
                a, r = sensor
                #sensor_seg = [self.P[pn, 0], self.P[pn, 1], self.P[pn, 0] + r * np.cos(self.P[pn, 2] + a), self.P[pn, 1] + r * np.sin(self.P[pn, 2] + a)]
                beam = (self.P[pn, 0], self.P[pn, 1], self.P[pn, 2] + a)
                #best_wall = None
                best_wall_d = np.inf

                # v
                tau_v = (self.v_walls[:, :, 2] - beam[0])/np.cos(beam[2])
                y_v = beam[1] + np.sin(beam[2]) * tau_v

                ind_v = (self.v_walls[:, :, 0] > self.wall_th) &  (tau_v > 0) & (y_v >= self.v_walls[:, :, 3]) & (y_v <= self.v_walls[:,:, 4])

                if np.count_nonzero(ind_v) != 0:
                    best_wall_d = np.min(tau_v[ind_v])

                # h
                tau_h = (self.h_walls[:, :, 4] - beam[1])/np.sin(beam[2])
                x_h = beam[0] + np.cos(beam[2]) * tau_h

                ind_h = (self.h_walls[:, :, 0] > self.wall_th) & (tau_h > 0) & (x_h >= self.h_walls[:, :, 2]) & (x_h <= self.h_walls[:, :, 3])

                if np.count_nonzero(ind_h) != 0:
                    best_wall_d = min(best_wall_d, np.min(tau_h[ind_h]))            

                if not np.isinf(best_wall_d):
                    wr = norm_pdf(best_wall_d - r, self.range_sigma)
                    if not sensor_w is None:
                        wr *= sensor_w[i]
                    w += wr

                #best_walls.append(best_wall_d - r)
                #self.print_func(best_wall_d)

            #self.W[pn] = w / len(sensors_data)
            self.W[pn] = w
            
                
        #return best_walls

    def update_pose_mean(self):
        #print(self.W)
        self.W = np.nan_to_num(self.W)
        sumW = np.sum(self.W)
        #if sumW == 0:
            #self.get_logger.error("W is 0")
            #plt.show()

        Wnorm = self.W / sumW

        prod = Wnorm @ self.P[:,:2]
        self.x = prod[0]
        self.y = prod[1]
        self.yaw = mean_angles(self.P[:,2].tolist(), self.W.tolist())

    def update_pose_max(self):
        ind = np.argmax(self.W)
        self.x = self.P[ind, 0]
        self.y = self.P[ind, 1]
        self.yaw = self.P[ind, 2]

    def resampling(self):
        # make a resamplinmg
        #self.get_logger().info("Resampling")
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
        xh1 = ix * (self.cell_size[0]+self.pile_size)# - (self.cell_size[0]+self.pile_size) / 2
        xh2 = ix * (self.cell_size[0]+self.pile_size) + (self.cell_size[0]+self.pile_size)
        yh = iy * (self.cell_size[1]+self.pile_size)# - (self.cell_size[1]+self.pile_size) / 2
        if with_pile:
            return xh1, xh2, yh
        else:
            return xh1 + self.pile_size, xh2 - self.pile_size, yh



    def get_v_wall_coords(self, ix, iy, with_pile = True):
        xh = ix * (self.cell_size[0]+self.pile_size)# - (self.cell_size[0]+self.pile_size) / 2
        yh1 = iy * (self.cell_size[1]+self.pile_size)# - (self.cell_size[1]+self.pile_size) / 2
        yh2 = iy * (self.cell_size[1]+self.pile_size) + (self.cell_size[1]+self.pile_size)
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
        
    # ROS2
    def to_pose_msg(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.yaw)
        pose.orientation = Quaternion(x = q[1], y = q[2], z = q[3], w = q[0])

        return pose

    def to_odom_msg(self, header, child_frame_id = ""):
        odom = Odometry()
        odom.header = header
        odom.child_frame_id = child_frame_id
        odom.pose.pose = self.to_pose_msg()

        dX = np.zeros(self.P.shape)
        dX[:, 0] = self.P[:, 0] - self.x
        dX[:, 1] = self.P[:, 1] - self.y
        dX[:, 2] = substract_angles(self.P[:,2], self.yaw)
        cov_mat = np.dot(dX * np.expand_dims(self.W, axis=1), dX.T)

        cov = np.zeros(36, np.float64)
        cov[0] = cov_mat[0,0] # xx
        cov[1] = cov_mat[0,1] # xy
        cov[5] = cov_mat[0,2] # x alpha
        cov[6] = cov_mat[1,0] # yx
        cov[7] = cov_mat[1,1] # yy
        cov[11] = cov_mat[1,2] # y alpha
        cov[30] = cov_mat[2,0] # alpha x
        cov[31] = cov_mat[2,1] # alpha y
        cov[35] = cov_mat[2,2] # alpha alpha

        odom.pose.covariance = list(cov)

        return odom

    def to_pose_array_msg(self, header):
        pose_array_msg = PoseArray()
        pose_array_msg.header = header
        for pn in range(self.NP):
            pose = Pose()
            pose.position.x = self.P[pn, 0]
            pose.position.y = self.P[pn, 1]
            q = quaternion_from_euler(0, 0, self.P[pn, 2])
            pose.orientation = Quaternion(x = q[1], y = q[2], z = q[3], w = q[0])
            pose_array_msg.poses.append(pose)
        return pose_array_msg

    def to_marker_array_msg(self, header):
        ma_msg = MarkerArray()
        z_wall = 0.2
        for i in range(self.v_walls.shape[0]):
            for j in range(self.v_walls.shape[1]):
                if self.v_walls[i, j, 0] != -1:
                    mrk = Marker()
                    mrk.header = header

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
                    elif self.v_walls[i, j, 0] > self.wall_th:
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
                    mrk.header = header

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
        return ma_msg

    def to_weights_marker_msg(self, header, max_z = 1.0):
        msg = Marker()
        msg.header = header
            
        msg.ns = "weight"
        msg.type = Marker.LINE_LIST
        
        msg.pose.orientation.w = 1.0
        msg.color.a = 1.0
        msg.color.r = 1.0

        
        msg.scale.x = 0.02
        
        scale = max_z / np.max(self.W)
            
        for pn in range(self.NP):
            p0 = Point()
            p0.x = self.P[pn, 0]
            p0.y = self.P[pn, 1]
            #p1 = p0
            p1 = Point()
            p1.x = self.P[pn, 0]
            p1.y = self.P[pn, 1]
            p1.z = scale * self.W[pn]
            msg.points.append(p0)
            msg.points.append(p1)
            
        return msg         
    
    
    #def to_sensors_marker(self, header, robot_pose, ranges_dists, ranges_angles, compass):
        #ma_msg = MarkerArray()
        
        #for i, r, a in enumerate(zip(ranges_dists, ranges_angles)):
            ##r, a = v
            #a_msg = Marker()
            #a_msg.header = header
            #a_msg.ns = "ranges"
            #a_msg.type = Marker.ARROW
            #a_msg.id = i
            
            #a_msg = 
            
        
        
    
            
        
