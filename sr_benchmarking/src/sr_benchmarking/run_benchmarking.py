#!/usr/bin/env python
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
