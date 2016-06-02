#!/usr/bin/env python

import glob
import yaml
import subprocess
from os.path import isdir
import signal
import psutil
import rospy


class AnnotationParserBase(object):
    """
    Parses the given annotation file (yaml).
    """
    def __init__(self, path_to_annotation, path_to_data):
        self._path_to_annotation = path_to_annotation
        self._path_to_data = path_to_data

        self._launch_proc = None
        self._rosbag_proc = None

        self.parse()

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.stop_bag()
        self.stop_launch()

    def parse(self):
        with open(self._path_to_annotation, 'r') as f:
            self._annotations = yaml.load(f)

    def play_launch(self, command):
        """
        Roslaunch the given command.
        """
        self._launch_proc = subprocess.Popen(command,
                                             stdin=subprocess.PIPE, shell=True)
        rospy.sleep(40)

    def stop_launch(self):
        """
        Stop the running launch file and its subprocesses
        """
        if self._launch_proc:
            self._stop_process(self._launch_proc)

    def play_bag(self, bag_file):
        """
        Plays the associated bag file
        """
        self._rosbag_proc = subprocess.Popen("rosbag play --clock " + bag_file,
                                             stdin=subprocess.PIPE, shell=True,
                                             cwd=self._path_to_data)

    def stop_bag(self):
        """
        Stops the currently running rosbag process.
        """
        if self._rosbag_proc:
            self._stop_process(self._rosbag_proc)

    def _stop_process(self, process):
        """
        Stops the given process and all its subprocesses
        """
        process_id = psutil.Process(process.pid)
        for sub_process in process_id.get_children(recursive=True):
            sub_process.send_signal(signal.SIGINT)
            sub_process.send_signal(signal.SIGTERM)
        process.wait()  # we wait for children to terminate
        try:
            process.terminate()
        except:
            pass

    def check_results(self, results):
        """
        Compares the results with the annotations found in
        self._annotations. This method is specific to the
        benchmarking used.
        """
        raise NotImplementedError("Implement this method in your own class.")

    def to_xml(self, results):
        """
        Saves the results as a Jenkins parsable xml file for display.
        """
        raise NotImplementedError("TODO implement me")


class BenchmarkingBase(object):
    """
    Base class for running the benchmarking.
    """
    def __init__(self, path_to_data):
        if not isdir(path_to_data):
            raise Exception("Could not find the directory " + path_to_data)

        self._path_to_data = path_to_data

    def load_files(self, path_to_data):
        """
        Loads the annotation file list from the given path.

        @param path_to_data the folder containing the different annotation
               files and data.
        """

        yaml_files = glob.glob(path_to_data + "/*.yaml")

        return yaml_files
