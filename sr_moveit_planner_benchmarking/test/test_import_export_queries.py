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

import os.path
import rospkg
import rospy

from unittest import TestCase

PKG = "sr_moveit_planner_benchmarking"


class TestImportExportQueries(TestCase):
    def setUp(self):
        self.test_repo_path = rospkg.RosPack().get_path('sr_moveit_planner_benchmarking') + "/test/"
        self.tmp_path_for_generated_files = "/tmp/generated_files_warehouse_moveit/"
        self.queries_to_import = self.test_repo_path + "test_queries_to_import.queries"
        self.queries_generated_after_export = self.tmp_path_for_generated_files + "test_scene.queries"
        rospy.sleep(25)

    def test_export_queries(self):
        self.assertTrue(os.path.isfile(self.queries_generated_after_export),
                        "Queries file was not generated %s" % self.queries_generated_after_export)
        with open(self.queries_to_import, 'r') as queries_imported:
            with open(self.queries_generated_after_export, 'r') as queries_exported:
                difference = set(queries_imported).difference(queries_exported)
                self.assertEqual(set([]), difference, "difference: %s" % difference)


if __name__ == '__main__':
    import rostest

    rostest.rosrun(PKG, 'test_import_export_queriess', TestImportExportQueries)
