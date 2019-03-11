#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from copy import deepcopy
import rospy
from moveit_commander import (MoveGroupCommander, PlanningSceneInterface,
                              RobotCommander)
from moveit_msgs.msg import MoveGroupActionResult, RobotState
from geometry_msgs.msg import Pose
from sr_benchmarking.sr_benchmarking import (AnnotationParserBase,
                                             BenchmarkingBase)
from tabulate import tabulate
from visualization_msgs.msg import Marker, MarkerArray
import time
import os
import numpy


class PlannerAnnotationParser(AnnotationParserBase):
    """
    Parses the annotations files that contains the benchmarking
    information.
    """
    def __init__(self, path_to_annotation, path_to_data):
        super(PlannerAnnotationParser, self).__init__(path_to_annotation, path_to_data)
        self.parse()

        self._load_scene()
        self._init_planning()
        self.benchmark()

    def check_results(self, results):
        """
        Returns the results from the planner, checking them against any eventual validation data
        (no validation data in our case).
        """
        return self.planner_data

    def _load_scene(self):
        """
        Loads the proper scene for the planner.
        It can be either a python static scene or bag containing an occupancy map.
        """
        scene = self._annotations["scene"]

        for element in scene:
            if element["type"] == "launch":
                self.play_launch(element["name"])
            elif element["type"] == "python":
                self.load_python(element["name"])
            elif element["type"] == "bag":
                self.play_bag(element["name"])
                for _ in range(150):
                    rospy.sleep(0.3)

        # wait for the scene to be spawned properly
        rospy.sleep(0.5)

    def _init_planning(self):
        """
        Initialises the needed connections for the planning.
        """

        self.group_id = self._annotations["group_id"]
        self.planners = self._annotations["planners"]
        self.scene = PlanningSceneInterface()
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(self.group_id)
        self._marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10, latch=True)
        self._planning_time_sub = rospy.Subscriber('/move_group/result', MoveGroupActionResult,
                                                   self._check_computation_time)
        rospy.sleep(1)

        self.group.set_num_planning_attempts(self._annotations["planning_attempts"])

        self.group.set_goal_tolerance(self._annotations["goal_tolerance"])
        self.group.set_planning_time(self._annotations["planning_time"])
        self.group.allow_replanning(self._annotations["allow_replanning"])

        self._comp_time = []

        self.planner_data = []

    def benchmark(self):
        for test_id, test in enumerate(self._annotations["tests"]):
            marker_position_1 = test["start_xyz"]
            marker_position_2 = test["goal_xyz"]
            self.space = test["space"]

            self._add_markers(marker_position_1, "Start test \n sequence", marker_position_2, "Goal")

            # Start planning in a given joint position
            joints = test["start_joints"]
            current = RobotState()
            current.joint_state.name = self.robot.get_current_state().joint_state.name
            current_joints = list(
                self.robot.get_current_state().joint_state.position)
            current_joints[0:6] = joints
            current.joint_state.position = current_joints

            self.group.set_start_state(current)
            if self.space == "joint":
                coordinates = test["goal_joints"]
            elif self.space == "pose":
                position = test["goal_xyz"]
                orientation = test["goal_orientation"]
                coordinates = self.create_pose(position, orientation)
            elif self.space == "position":
                coordinates = test["goal_xyz"]

            for planner in self.planners:
                if planner == "stomp":
                    planner = "STOMP"
                elif planner == "sbpl":
                    planner = "AnytimeD*"
                self.planner_id = planner
                self.group.set_planner_id(planner)
                self._plan_joints(coordinates, self._annotations["name"]+"-test_"+str(test_id)+"-"+self.space)

        return self.planner_data

    def create_pose(self, position, orientation):
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]

        pose.orientation.x = orientation[0]
        pose.orientation.y = orientation[1]
        pose.orientation.z = orientation[2]
        pose.orientation.w = orientation[3]
        return pose

    def _add_markers(self, point, text1, point_2, text2):
        # add marker for start and goal pose of EE
        marker_array = MarkerArray()
        marker_1 = Marker()
        marker_1.header.frame_id = "world"
        marker_1.header.stamp = rospy.Time.now()
        marker_1.type = Marker.SPHERE
        marker_1.scale.x = 0.04
        marker_1.scale.y = 0.04
        marker_1.scale.z = 0.04
        marker_1.lifetime = rospy.Duration()

        marker_2 = deepcopy(marker_1)

        marker_1.color.g = 0.5
        marker_1.color.a = 1.0
        marker_1.id = 0
        marker_1.pose.position.x = point[0]
        marker_1.pose.position.y = point[1]
        marker_1.pose.position.z = point[2]
        marker_2.color.r = 0.5
        marker_2.color.a = 1.0
        marker_2.id = 1
        marker_2.pose.position.x = point_2[0]
        marker_2.pose.position.y = point_2[1]
        marker_2.pose.position.z = point_2[2]

        marker_3 = Marker()
        marker_3.header.frame_id = "world"
        marker_3.header.stamp = rospy.Time.now()
        marker_3.type = Marker.TEXT_VIEW_FACING
        marker_3.scale.z = 0.10
        marker_3.lifetime = rospy.Duration()

        marker_4 = deepcopy(marker_3)

        marker_3.text = text1
        marker_3.id = 2
        marker_3.color.g = 0.5
        marker_3.color.a = 1.0
        marker_3.pose.position.x = point[0]
        marker_3.pose.position.y = point[1]
        marker_3.pose.position.z = point[2] + 0.15
        marker_4.text = text2
        marker_4.id = 3
        marker_4.color.r = 0.5
        marker_4.color.a = 1.0
        marker_4.pose.position.x = point_2[0]
        marker_4.pose.position.y = point_2[1]
        marker_4.pose.position.z = point_2[2] + 0.15

        marker_array.markers = [marker_1, marker_2, marker_3, marker_4]

        self._marker_pub.publish(marker_array)
        rospy.sleep(1)

    def _plan_joints(self, joints, test_name):
        # plan to joint target and determine success
        self.group.clear_pose_targets()
        if self.space == "joint":
            group_variable_values = self.group.get_current_joint_values()
            group_variable_values[0:6] = joints[0:6]
            self.group.set_joint_value_target(group_variable_values)
        elif self.space == "pose":
            self.group.set_joint_value_target(joints)
        elif self.space == "position":
            self.group.set_position_target(joints)

        plan = self.group.plan()
        plan_time = "N/A"
        total_joint_rotation = "N/A"
        comp_time = "N/A"
        plan_evaluation = "N/A"

        plan_success = self._check_plan_success(plan)
        if plan_success:
            plan_time = self._check_plan_time(plan)
            total_joint_rotation = self._check_plan_total_rotation(plan)
            plan_evaluation = self.evaluate_plan(plan)
            while not self._comp_time:
                rospy.sleep(0.5)
            comp_time = self._comp_time.pop(0)
        self.planner_data.append([self.planner_id, test_name, str(plan_success), total_joint_rotation,
                                 plan_evaluation, comp_time, plan_time])

    @staticmethod
    def _check_plan_success(plan):
        if len(plan.joint_trajectory.points) > 0:
            return True
        else:
            return False

    @staticmethod
    def _check_plan_time(plan):
        # find duration of successful plan
        number_of_points = len(plan.joint_trajectory.points)
        time = plan.joint_trajectory.points[number_of_points - 1].time_from_start.to_sec()
        return time

    @staticmethod
    def _check_plan_total_rotation(plan):
        # find total joint rotation in successful trajectory
        angles = [0, 0, 0, 0, 0, 0]
        number_of_points = len(plan.joint_trajectory.points)
        for i in range(number_of_points - 1):
            angles_temp = [abs(x - y) for x, y in zip(plan.joint_trajectory.points[i + 1].positions,
                                                      plan.joint_trajectory.points[i].positions)]
            angles = [x + y for x, y in zip(angles, angles_temp)]

        total_angle_change = sum(angles)
        return total_angle_change

    def evaluate_plan(self, plan):
        num_of_joints = len(plan.joint_trajectory.points[0].positions)
        weights = numpy.array(sorted(range(1, num_of_joints + 1), reverse=True))
        plan_array = numpy.empty(shape=(len(plan.joint_trajectory.points),
                                        num_of_joints))

        for i, point in enumerate(plan.joint_trajectory.points):
            plan_array[i] = point.positions

        deltas = abs(numpy.diff(plan_array, axis=0))
        sum_deltas = numpy.sum(deltas, axis=0)
        sum_deltas_weighted = sum_deltas * weights
        plan_quality = numpy.sum(sum_deltas_weighted)
        return plan_quality

    def _check_computation_time(self, msg):
        # get computation time for successful plan to be found
        if msg.status.status == 3:
            self._comp_time.append(msg.result.planning_time)

    def _check_path_length(self, plan):
        # find distance travelled by end effector
        # not yet implemented
        return


class PlannerBenchmarking(BenchmarkingBase):
    """
    Runs the benchmarking for the planners.
    """
    def __init__(self):
        self._path_to_results = rospy.get_param("~results", "/results")
        self._path_to_data = rospy.get_param("~data", "/data")

        super(PlannerBenchmarking, self).__init__(self._path_to_data)

        results = []
        # iterate through all annotation files to run the benchmarks
        for annotation_file in self.load_files(self._path_to_data):
            with PlannerAnnotationParser(annotation_file, self._path_to_data) as parser:
                results.append(parser.check_results(None))

        self.pretty_results(results)

    def pretty_results(self, results):
        """
        Outputs the results in a pretty format (human readable, jenkins parsable)
        """
        # TODO sort first
        # reformatting the results
        results = [item for sublist in results for item in sublist]

        row_titles = ["Planner", "Plan name", "Plan succeeded", "Total angle change",
                      "Plan quality", "Time of plan (s)", "Computation time (s)"]
        print(tabulate(results, headers=row_titles, tablefmt='orgtbl'))

        file_path = os.path.join(self._path_to_results, '')
        file_path += time.strftime("%Y_%m_%d-%H_%M_%S")
        file_path += "-planner_benchmark.xml"
        with open(file_path, 'w') as f:
            f.write(tabulate(results, headers=row_titles, tablefmt="simple"))  # ="html"))


if __name__ == '__main__':
    rospy.init_node("planner_benchmark")

    PlannerBenchmarking()
