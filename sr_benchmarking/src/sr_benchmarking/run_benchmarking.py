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

"""
See README.md
"""
import rospy
from unittest import TestCase
from sr_benchmarking_msgs.srv import ExecuteBenchmarking


class RunBenchmarking(TestCase):

    def setUp(self):
        rospy.init_node('run_benchmarking', anonymous=True)
        self._input_files = rospy.get_param('~input_files')
        self._output_file = rospy.get_param('~output_file')

    def test_run_benchmarking(self):
        rospy.wait_for_service('execute_benchmarking')
        try:
            execute_benchmarking = rospy.ServiceProxy('execute_benchmarking', ExecuteBenchmarking)
            response = execute_benchmarking(self._input_files, self._output_file)
            self.assertTrue(response.result, 'Benchmarking returned false')
        except rospy.ServiceException, e:
            self.fail('Benchmarking call failed: %s' % e)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('sr_benchmarking', 'run_benchmarking', RunBenchmarking)
