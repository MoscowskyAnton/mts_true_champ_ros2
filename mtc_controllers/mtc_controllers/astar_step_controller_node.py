import rclpy
from rclpy.node import Node
from mtc_msgs.srv import StepSLAMCommand
from std_srvs.srv import Trigger
import numpy as np

from mtc_slam.slam_lib import norm_angle, euler_from_quaternion, raskukozjit_h_walls, raskukozjit_v_walls

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class AStarController(Node):

    def __init__(self):
        super().__init__('astar_controller')

        self.running = False


        self.cmd_cli = self.create_client(StepSLAMCommand, "/mtc/slam/cmd")
        self.clear_cli = self.create_client(Trigger, "/mtc/slam/clear_map")

        self.declare_parameter('labirinth_size', 16)
        self.declare_parameter('cell_total_size', [0.0, 0.0])

        self.declare_parameter('sim', False)
        self.sim = self.get_parameter('sim').value

        self.declare_parameter('fwd_th', 0.0)
        self.fwd_th = self.get_parameter('fwd_th').value

        self.declare_parameter('stuck_mp', 1000.)
        self.stuck_mp = self.get_parameter('stuck_mp').value

        self.labirinth_size = self.get_parameter('labirinth_size').value
        self.cell_total_size = self.get_parameter('cell_total_size').value

        self.declare_parameter('angle_th', np.deg2rad(30))
        self.angle_th = self.get_parameter('angle_th').value

        self.declare_parameter('avoid_lim', -1)
        self.avoid_lim = self.get_parameter('avoid_lim').value

        self.was_cells = np.zeros((self.labirinth_size, self.labirinth_size), dtype = bool)

        self.target_cells = [(7,7), (7,8), (8,7), (8,8)] # TODO params

        self.plan_pub = self.create_publisher(Path, '~/global_plan', 10)
        self.path_pub = self.create_publisher(Path, '~/passed_path', 10)

        self.start_pause_srv = self.create_service(Trigger, '~/start_pause', self.start_pause_cb)
        #self.cmd_srv = self.create_service(Trigger, '~/start_pause', self.start_pause_cb)



    def start_pause_cb(self, request, responce):
        self.running = not self.running
        responce.success = True
        if self.running:
            responce.message = "Robot in action"
        else:
            responce.message = "Robot paused"
        return responce


    def get_command_from_plan(self, pose, plan):
        pass



    def set_cmd_and_get_pose_and_map(self, cmd):
        request = StepSLAMCommand.Request()
        request.direction = int(cmd[0])
        request.value = float(cmd[1])

        future = self.cmd_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        responce = future.result()

        x = responce.pose.pose.position.x
        y = responce.pose.pose.position.y
        yaw = euler_from_quaternion(responce.pose.pose.orientation)[2]

        v_walls = raskukozjit_v_walls(responce.v_walls, self.labirinth_size)
        h_walls = raskukozjit_h_walls(responce.h_walls, self.labirinth_size)

        return (x, y , yaw), (v_walls, h_walls)


    def get_cell_from_pose(self, pose):
        c_x = int(pose[0] / self.cell_total_size[0])
        c_y = int(pose[1] / self.cell_total_size[1])
        return c_x, c_y

    def get_pose_from_cell(self, cell):
        x = cell[0] * self.cell_total_size[0] + self.cell_total_size[0]/2
        y = cell[1] * self.cell_total_size[1] + self.cell_total_size[1]/2
        return [x, y]

    def get_closest_target(self, cell):
        dists = [np.hypot(cell[0] - tc[0], cell[1] - tc[1]) for tc in self.target_cells]
        closest_target_index = dists.index(min(dists))
        return self.target_cells[closest_target_index]


    def a_star_way_clear(self, cell, shift):
        to_cell = (cell[0] + shift[0], cell[1] + shift[1])
        if to_cell[0] < 0 or to_cell[0] >= self.labirinth_size or to_cell[1] < 0 or to_cell[1] >= self.labirinth_size:
            return False

        if shift[0] != 0:
            # look for v walls
            if shift[0] == 1:
                if self.v_walls[cell[0]+1, cell[1]] == 0 or self.v_walls[cell[0]+1, cell[1]] == -1:
                    return True
                else:
                    return False
            else:
                if self.v_walls[cell[0], cell[1]] == 0 or self.v_walls[cell[0], cell[1]] == -1:
                    return True
                else:
                    return False
        elif shift[1] != 0:
            if shift[1] == 1:
                #print(cell[0], cell[1]+1, self.MAP_h_walls[cell[0], cell[1] + 1])
                if self.h_walls[cell[0], cell[1] + 1] == 0 or self.h_walls[cell[0], cell[1] + 1] == -1:
                    return True
                else:
                    return False
            else:
                if self.h_walls[cell[0], cell[1]] == 0 or self.h_walls[cell[0], cell[1]] == -1:
                    return True
                else:
                    return False

    def a_star_plan(self, from_cell, target_cell):
        self.get_logger().info(f"Planning from {from_cell} to {target_cell}")

        def a_star_heuristic(c1, c2):
            w = 1.0  # weight of heuristic
            d = w * np.hypot(c1[0] - c2[0], c1[1] - c2[1])
            return d

        open_set = {}
        closed_set = {}

        way_found = False

        open_set[from_cell] = (0, None) # init cost, no parent
        while True:

            if len(open_set) == 0:
                #print("A*: Every is open now, break")
                break



            current = min(open_set, key=lambda o: open_set[o][0] + a_star_heuristic(o, target_cell) )

            closed_set[current] = open_set[current]
            if current == target_cell:
                way_found = True
                #print("A*: way found!")
                break

            del open_set[current]

            def a_star_motion():
                return [(1, 0, 1),
                        (0, 1, 1),
                        (-1, 0, 1),
                        (0, -1, 1)]

            for motion in a_star_motion():
                to_cell = (current[0] + motion[0], current[1] + motion[1])
                if to_cell in closed_set:
                    continue

                if not self.a_star_way_clear(current, motion):
                    continue

                cell_params = (closed_set[current][0] + motion[2], current) # upd cost, set parent
                #print(f"{current}+ {to_cell}: {cell_params}")
                if to_cell in open_set:
                    if cell_params[0] < open_set[to_cell][0]:
                        open_set[to_cell] = cell_params
                else:
                    open_set[to_cell] = cell_params

        if way_found:
            start_from = target_cell
        else:
            start_from = min(closed_set, key=lambda o:a_star_heuristic(o, target_cell) if self.was_cells[o] == False else np.inf)
        # else: # NOTE actually that case will never happen now
        #     start_from = min(closed_set, key=lambda o:a_star_heuristic(o, target_cell) if self.MAP[o] == 0 else np.inf)

        # make a way
        path = []
        while True:
            parent = closed_set[start_from][1]
            if parent is None:
                break
            path.append((start_from[0] - parent[0], start_from[1] - parent[1]))
            start_from = parent

        path.reverse()
        return way_found, path

    def get_cmd_from_plan(self, current_pose, plan):
        shift = plan[0]
        current_cell = self.get_cell_from_pose(current_pose)
        target_cell = [current_cell[0] + shift[0], current_cell[1] + shift[1]]

        target_pose = self.get_pose_from_cell(target_cell)

        dx = target_pose[0] - current_pose[0]
        dy = target_pose[1] - current_pose[1]

        dr = np.hypot(dx, dy)
        a = np.arctan2(dy, dx)
        da = norm_angle(a - current_pose[2])
        self.get_logger().info(f"Cmd {dr} {da}")
        mda = np.abs(da)
        if mda > self.angle_th:
            if da > 0:
                return (StepSLAMCommand.Request().LEFT, mda)
            else:
                return (StepSLAMCommand.Request().RIGHT, mda)

        if self.sim:
            return (StepSLAMCommand.Request().FORWARD, 0.95) # for sim
        else:
            if self.fwd_th != 0 and dr > self.fwd_th:
                return (StepSLAMCommand.Request().FORWARD, self.fwd_th)
            return (StepSLAMCommand.Request().FORWARD, dr)


    def spin(self):
        cmd = (0, 0.0)
        avoid_cnt = 0
        msg_passed_path = Path()
        msg_passed_path.header.frame_id = "map"
        msg_passed_path.poses.clear()
        prev_pose = None
        while rclpy.ok():

            if self.running:
                # do command
                #self.get_logger().info(f"Executi is {plan}")
                pose, walls = self.set_cmd_and_get_pose_and_map(cmd)

                pose_msg = PoseStamped()
                pose_msg.pose.position.x = pose[0]
                pose_msg.pose.position.y = pose[1]
                msg_passed_path.poses.append(pose_msg)
                msg_passed_path.header.stamp = self.get_clock().now().to_msg()
                self.path_pub.publish(msg_passed_path)

                if (not prev_pose is None) and cmd[0] == StepSLAMCommand.Request().FORWARD:
                    dr = np.hypot(pose[0] - prev_pose[0], pose[1] - prev_pose[1])
                    self.get_logger().info(f"dr: {dr}, cmd/: {cmd[1]/self.stuck_mp}")

                    if (self.sim and dr < cmd[1]) or (not self.sim and dr < cmd[1] / self.stuck_mp):
                        self.get_logger().info(f"Robot stuck, move backward")
                        if self.sim:
                            cmd = (StepSLAMCommand.Request().BACKWARD, 0.95)
                        else:
                            cmd = (StepSLAMCommand.Request().BACKWARD, 0.01)
                        continue


                current_cell = self.get_cell_from_pose(pose)
                self.was_cells[current_cell] = True

                # get_plan
                target_cell = self.get_closest_target(pose)
                self.v_walls, self.h_walls = walls
                way_found, plan = self.a_star_plan(current_cell, target_cell)
                self.get_logger().info(f"Plan is {plan}")
                self.plan_pub.publish(self.plan_to_path_msg(plan, current_cell))

                # plan to cmd
                if not way_found and len(plan) == 0:
                    self.get_logger().info(f"No escape!")
                    if self.avoid_lim != -1 and self.avoid_lim <= avoid_cnt:
                        self.get_logger().info(f"Clearing map...")
                        avoid_cnt = 0
                        # reset walls

                        future = self.clear_cli.call_async(Trigger.Request())
                        rclpy.spin_until_future_complete(self, future)
                        responce = future.result()
                        cmd = (0, 0.0)

                    else:
                        self.get_logger().info(f"Try rotating {avoid_cnt}/{self.avoid_lim}")
                        # no way may be because of labirinth errors, try rotating
                        cmd = (np.random.choice([StepSLAMCommand.Request().LEFT,
                                                StepSLAMCommand.Request().RIGHT]),
                                                np.pi/4)
                        avoid_cnt += 1


                elif len(plan) > 0:
                    cmd = self.get_cmd_from_plan(pose, plan)
                else:
                    # we are on target
                    self.get_logger().info("Target reached! Pause controller")
                    self.running = False
                    cmd = (0, 0.0)
                prev_pose = pose


            rclpy.spin_once(self)


    def plan_to_path_msg(self, plan, from_cell):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        cell = list(from_cell)
        plan_ = [(0,0)]+plan
        for shift in plan_:
            pose_msg = PoseStamped()
            #pose_msg.header = msg.header

            cell[0] = cell[0] + shift[0]
            cell[1] = cell[1] + shift[1]

            coords = self.get_pose_from_cell(cell)
            pose_msg.pose.position.x = float(coords[0])
            pose_msg.pose.position.y = float(coords[1])
            pose_msg.pose.orientation.w = 1.0

            msg.poses.append(pose_msg)

        return msg



def main():
    rclpy.init()

    astar_controller = AStarController()
    astar_controller.spin()


if __name__ == '__main__':
    main()
