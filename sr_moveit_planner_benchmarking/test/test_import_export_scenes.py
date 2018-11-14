#!/usr/bin/env python

import os.path
import rospkg
import rospy

from unittest import TestCase

PKG = "sr_moveit_planner_benchmarking"


class TestImportExportScenes(TestCase):
    def setUp(self):
        self.test_repo_path = rospkg.RosPack().get_path('sr_moveit_planner_benchmarking') + "/test/"
        self.tmp_path_for_generated_files = "/tmp/generated_files_warehouse_moveit/"
        self.scene_to_import = self.test_repo_path + "test_scene_to_import.scene"
        self.scene_generated_after_export = self.tmp_path_for_generated_files + "test_scene.scene"
        rospy.sleep(20)

    def test_export_scene(self):
        self.assertTrue(os.path.isfile(self.scene_generated_after_export),
                        "Scene file was not generated %s" % self.scene_generated_after_export)
        with open(self.scene_to_import, 'r') as scene_imported:
            with open(self.scene_generated_after_export, 'r') as scene_exported:
                difference = set(scene_imported).difference(scene_exported)
                self.assertEqual(set([]), difference, "difference: %s" % difference)


if __name__ == '__main__':
    import rostest

    rostest.rosrun(PKG, 'test_import_export_scenes', TestImportExportScenes)
