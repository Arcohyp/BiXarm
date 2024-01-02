#!/usr/bin/env python3
# For 2023.12 Project
# Author: Xirun Cheng

"""
# Notice
#   1. XarmAPI -> ProjectXarm -> BiXarm
#   2. Multi-Threads
"""

import time
import copy
from threading import Thread
from xarm.wrapper import XArmAPI
# We need reconstruct the Config.
# The core points: Must ensure at the beginning
# 1. zero point: zp
# 2. low point: lp
# 3. high point: hp
# 4. camera point: cp
# 5. basket points: bp
#     f1:
#     f2:
#     f3:
#     f4:
#     f5:
#     b1:
#     b2:
#     b3:
#     b4:
#     b5:
# 6. middle point: mp
#     mf4:
#     mf5:
#     mb4:
#     mb5:
# 7. aligning_point: ap

# The reference points: Can be referenced from core points
# 1. middle point: mp
#     mf1
#     mf2
#     mf3
#     mb1
#     mb2
#     mb3

# Same trajectory:
# Stage 0, Start: zero_point, high_point
# Stage 1, Pick Object: camera_point, photo_time, zero_point, low_point, gripper_on, zero_point, high_point
# Stage 2, Place Object: mid_point, basket_point
# Stage 3, Back for next loop: gripper_off, mid_point, high_point
# Stage 4, Back to start: high_point, zero_point


def cnt(list1, list2):
    return [x + y for x, y in zip(list1, list2)]


class Config:
    """
    These are some parameters for arm's movement.

    Args:
        - loop: How many times the planing played
        - IP_f: Near the hole
        - IP_b: Away from the hole
        - others: can be referred to XArmAPI reference

    Returns:
        - None
    """

    # Loop
    loop = 1

    # XArm Information
    IP_f = '192.168.1.220'
    IP_b = '192.168.1.206'
    speed = 60
    acc = 160

    # State/Point
    quit = False
    zero_point = [207.0, 0.0, 112.0, 180.0, 0.0, 0.0]
    aligning_point = [206.4, 0.0, 256.2, 180.0, 0.0, 0.0]
    camera_point = [176.1, 0.0, 410.0, 180.0, 0.0, 0.0]
    low_point = [206.4, 0.0, 82.0, 180.0, 0.0, 0.0]
    high_point = [145.0, 0.0, 580.0, 180.0, 0.0, 0.0]
    rotate = [0.0, 0.0, 0.0, 0.0, 0.0, -179.0]

    # For 4 and 5
    mid_point_f = [261.8, 282.2, 433.3, 180.0, 0.0, 47.2]
    mid_point_b = [147.3, -191.3, 433.3, 180.0, 0.0, -47.2]

    f_basket_points = [[-121.6, -409.7, -64.6, -179.6, 3.2, -116.3],  # 1
                       [222.4, -359.4, -85.5, 180.0, 0.8, -58.1],  # 2
                       [569.0, -358.3, -58.0, 179.5, -4.6, -31.9],  # 3
                       [542.6, 405.6, -19.8, 177.3, -34.5, 95.4],  # 4
                       [507.5, 374.5, 508.5, 111.2, -12.2, 167.8]]  # 5

    b_basket_points = [[588.5, 371.4, 41.6, 180.0, 0.5, 32.4],  # 1
                       [275.6, 337.3, -20.8, 180.0, 2.6, 50.9],  # 2
                       [-27.7, 396.5, -22.1, 180.0, 2.6, 94.1],  # 3
                       [48.9, -365.0, -87.9, 178.9, -23.9, -83.9],  # 4
                       [9.2, -442.2, 517.0, 179.8, -52.8, -88.6]]  # 5

    # End Effector. You can set gripper_on to 0 in order to just seeing the trajectories.
    gripper_on = 1
    gripper_off = 0

    # Sleep Time
    photo_time = '1'
    long_interval = '29'
    short_interval = '3'

    # 5 stages of xarm planning
    stage0 = [zero_point, high_point]
    stage1 = [camera_point, photo_time, zero_point, low_point, gripper_on, aligning_point, high_point]
    stage1_r = [camera_point, photo_time, cnt(zero_point, rotate), cnt(low_point, rotate), gripper_on,
                cnt(aligning_point, rotate), cnt(high_point, rotate), high_point]
    # stage2 = [mid_point, basket_point], dynamic get from get_stage_2_3()
    # stage3 = [gripper_off, mid_point, high_point], dynamic get from get_stage_2_3()
    stage4 = [high_point, zero_point]


def get_mid_point(list1):
    mid_point = copy.deepcopy(list1)
    mid_point[2] = 250
    return mid_point


def get_stage_1(rotation):
    return Config.stage1_r if rotation else Config.stage1


def get_stage_2_3(xarm, basket):
    def get_stage_points(midpoint, points):
        """Return stage2 and stage3 points based on xarm and basket."""
        stage2 = [midpoint, points]
        stage3 = [Config.gripper_off, midpoint, Config.high_point]
        return stage2, stage3

    if basket <= 2:
        mid_point = get_mid_point(Config.b_basket_points[basket]) if xarm == 'b' else get_mid_point(
            Config.f_basket_points[basket])
    else:
        mid_point = Config.mid_point_b if xarm == 'b' else Config.mid_point_f

    return get_stage_points(mid_point,
                            Config.b_basket_points[basket] if xarm == 'b' else Config.f_basket_points[basket])


class ProjectXarm(XArmAPI):
    def __init__(self, port=None):
        """
        This is an overwrote initialization of arms, clean warnings and errors, set mode and state.

        Args:
            port
        """
        super().__init__(port)
        self.clean_warn()
        self.clean_error()
        self.motion_enable(True)
        self.set_mode(0)
        self.set_state(0)
        # self.reset()
        time.sleep(1)

    def _set_point(self, position):
        """
        Repack the set_position() by using position.

        Args:
            position: list

        Returns:

        """
        if self.error_code == 0 and not Config.quit:
            if self.set_position(*position, speed=Config.speed, mvacc=Config.acc, radius=-1.0, wait=True) != 0:
                Config.quit = True
        return Config.quit

    def set_zero_point(self):
        return self._set_point(Config.zero_point)

    def set_safe_point(self):
        return self._set_point(Config.high_point)

    def grip_object(self):
        """
        Open the vacuum gripper and grip object.
        """
        # Deal with the vacuum gripper
        if self.error_code == 0 and not Config.quit:
            self.set_suction_cup(True, wait=False, delay_sec=0)
            # Just check once
            if not self.get_vacuum_gripper():
                self.set_suction_cup(False, wait=False, delay_sec=0)
                self.set_suction_cup(True, wait=False, delay_sec=0)

        return Config.quit

    def release_object(self):
        """
            Close the vacuum gripper and drop object.
        """
        # Deal with the vacuum gripper
        if not Config.quit:
            self.set_suction_cup(False, wait=False, delay_sec=0)
            # Just check once
            if not self.get_vacuum_gripper():
                self.set_suction_cup(True, wait=False, delay_sec=0)
                self.set_suction_cup(False, wait=False, delay_sec=0)

        return Config.quit

    def action_handler(self, action_list):
        """
        Handle actions from action_list.

        Args:
            action_list: list

        Returns:

        """
        for i in range(len(action_list)):
            if isinstance(action_list[i], int):
                Config.quit = self.grip_object() if action_list[i] else self.release_object()
            elif isinstance(action_list[i], str):
                time.sleep(int(action_list[i]))
            else:
                Config.quit = self._set_point(action_list[i])
            if Config.quit:
                break

        return Config.quit

    # def _check_position(self, target_position):
    #     '''
    #         Discard.
    #     '''
    #     p1 = np.array(target_position)
    #     _, p2 = self.get_position()
    #     print(p2)
    #     print(cosine_similarity([p1],[p2]))
    #     return cosine_similarity([p1],[p2])


class BiXarm(object):
    """
        1st floor: | 3 | 2 | 1 |
        xArms:         b   f
        1st floor:   | 4 |
        2nd floor:   | 5 |

        For a ProjectXarm, it would be at least 9(= 4 + 5) key points according to the below process:
            - same point:
                zero point -> camera point -> low point -> high point
            - different points for xarm_b/xarm_f:
                1~5 basket point(may insert points for smooth planning)

        The whole process is:
        0. Move up for taking a photo.
        1. Move down to approach the object.
        2. Open the vacuum gripper.
        3. Move up for next destination.
        4. Move to target basket.
            (1) Move to 1(1st floor)
            (2) Move to 2(1st floor)
            (3) Move to 3(1st floor)
            (4) Move to 4(1st floor)
            (5) Move to 5(2nd floor)
        5. Close the vacuum gripper.(Object will drop into target basket.)
        6. Back to zero point.
    """

    xarm_b = ProjectXarm(Config.IP_b)
    xarm_f = ProjectXarm(Config.IP_f)

    def multi_threads(self, f_stage: list, b_stage: list, reverse=False, interval='0'):
        def start_thread(stage, arm):
            if stage:
                thread = Thread(target=arm.action_handler, args=([stage, ]))
                return thread
            return None

        t1 = start_thread(f_stage, self.xarm_f)
        t2 = start_thread(b_stage, self.xarm_b)

        if t1 and t2:
            if reverse:
                t2.start()
                time.sleep(int(interval))
                t1.start()
            else:
                t1.start()
                time.sleep(int(interval))
                t2.start()
            # End
            t1.join()
            t2.join()

        elif t1:
            t1.start()
            t1.join()
        elif t2:
            t2.start()
            t2.join()

    def quick_start(self):
        print(f'xarm_f and xarm_b are quick starting. Execute stage0. From zero point to high point.')
        self.multi_threads(f_stage=Config.stage0, b_stage=Config.stage0)

    def quick_back(self):
        print(f'xarm_f and xarm_b are quick backing. Execute stage4. From high point to zero point.')
        self.multi_threads(f_stage=Config.stage4, b_stage=Config.stage4)
        self.xarm_b.reset()
        self.xarm_f.reset()

    def baskets_planning(self, times):
        """
        Using timestamps to realize synchronization of two arms.
        - init = stage0
        - ex = stage1/stage1_r
        - bx(x = 1,2,3) = basket1, basket2, basket3

            time:|   0  |  1 |  2 |  3 |  4 |  5 |  6 |  7 |
            f:   | init | ex | b1 | ex | b2 | ex | b3 | ...
            b:   | init |    | ex | b1 | ex | b2 | ex | b3 |

        We just need to deal with the 1st f ex and 3rd b b3.

        Args:
            times:

        Returns:

        """
        task = 'basket planning'

        # Quick start
        # 0
        self.quick_start()

        stage1 = get_stage_1(rotation=0)

        # 1
        print(f'Execute %s timestamp 1.' % task)
        self.multi_threads(f_stage=stage1, b_stage=[])

        for i in range(int(times)):
            # 2
            print(f'Execute %s timestamp 2.' % task)
            stage2, stage3 = get_stage_2_3(xarm='f', basket=0)
            self.multi_threads(f_stage=stage2+stage3, b_stage=stage1)

            # 3
            print(f'Execute %s timestamp 3.' % task)
            stage1 = get_stage_1(rotation=1)
            stage2, stage3 = get_stage_2_3(xarm='b', basket=0)
            self.multi_threads(f_stage=stage1, b_stage=stage2+stage3, reverse=True, interval=Config.short_interval)

            # 4
            print(f'Execute %s timestamp 4.' % task)
            stage2, stage3 = get_stage_2_3(xarm='f', basket=1)
            self.multi_threads(f_stage=stage2+stage3, b_stage=stage1)

            # 5
            print(f'Execute %s timestamp 5.' % task)
            stage2, stage3 = get_stage_2_3(xarm='b', basket=1)
            self.multi_threads(f_stage=stage1, b_stage=stage2+stage3)

            # 6
            print(f'Execute %s timestamp 6.' % task)
            stage2, stage3 = get_stage_2_3(xarm='f', basket=2)
            self.multi_threads(f_stage=stage2+stage3, b_stage=stage1, interval=Config.long_interval)

            # 7
            print(f'Execute %s timestamp 7.' % task)
            stage2, stage3 = get_stage_2_3(xarm='b', basket=2)
            if i == times-1:
                # t1 END
                self.multi_threads(f_stage=[], b_stage=stage2+stage3)
            else:
                stage1 = get_stage_1(rotation=0)
                self.multi_threads(f_stage=stage1, b_stage=stage2+stage3)

        # Quick End
        self.quick_back()

    def shelf_planning(self, times):
        """
         Using timestamps to realize synchronization of two arms.
         - init = stage0
         - ex = stage1/stage1_r
         - bx(x = 4,5) = basket4, basket5

            time:|   0  |  1 |  2 |  3 |  4 |  5 |
            f:   | init | ex | b4 | ex | b5 | ...
            b:   | init |    | ex | b4 | ex | b5 |

        We just need to deal with the 1st f ex and 2nd b b5.

        Args:
            times:

        Returns:

        """
        task = 'shelf planning'

        # Quick start
        # 0
        self.quick_start()

        stage1 = get_stage_1(rotation=0)

        # 1
        print(f'Execute %s timestamp 1.' % task)
        self.multi_threads(f_stage=stage1, b_stage=[])

        for i in range(int(times)):
            # 2
            print(f'Execute %s timestamp 2.' % task)
            stage2, stage3 = get_stage_2_3(xarm='f', basket=3)
            self.multi_threads(f_stage=stage2+stage3, b_stage=stage1)

            # 3
            print(f'Execute %s timestamp 3.' % task)
            stage1 = get_stage_1(rotation=1)
            stage2, stage3 = get_stage_2_3(xarm='b', basket=3)
            self.multi_threads(f_stage=stage1, b_stage=stage2+stage3)

            # 4
            print(f'Execute %s timestamp 4.' % task)
            stage2, stage3 = get_stage_2_3(xarm='f', basket=4)
            self.multi_threads(f_stage=stage2+stage3, b_stage=stage1)

            # 5
            print(f'Execute %s timestamp 5.' % task)
            stage2, stage3 = get_stage_2_3(xarm='b', basket=4)
            if i == times-1:
                # t1 END
                self.multi_threads(f_stage=[], b_stage=stage2+stage3)
            else:
                self.multi_threads(f_stage=stage1, b_stage=stage2+stage3)

        # Quick End
        self.quick_back()


def main():
    bi_xarm = BiXarm()
    bi_xarm.baskets_planning(Config.loop)
    # bi_xarm.shelf_planning(Config.loop)


if __name__ == "__main__":
    # Make sure using self.reset() after self.set_zero_point()
    # Or just trust quick_start and quick_back
    main()
