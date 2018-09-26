#!/usr/bin/env python

import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib import __version__ as matplotlibversion

import rospy
import os
import rospkg
from math import floor
import sqlite3
import rviz
import subprocess
import numpy as np

class SrMoveitPlannerBenchmarksVisualizer(Plugin):
    def __init__(self, context):
        super(SrMoveitPlannerBenchmarksVisualizer, self).__init__(context)
        self.setObjectName("SrMoveitPlannerBenchmarksVisualizer")
        self._widget = QWidget()

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_moveit_planner_benchmarking'), 'uis', 'moveit_planner_benchmarking.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Moveit Planner Benchmarks")

        self.clearance_layout = self._widget.findChild(QVBoxLayout, "clearance_layout")
        self.correct_layout = self._widget.findChild(QVBoxLayout, "correct_layout")
        self.lenght_layout = self._widget.findChild(QVBoxLayout, "lenght_layout")
        self.quality_1_layout = self._widget.findChild(QVBoxLayout, "quality_1_layout")
        self.quality_2_layout = self._widget.findChild(QVBoxLayout, "quality_2_layout")
        self.smoothness_layout = self._widget.findChild(QVBoxLayout, "smoothness_layout")
        self.plan_time_layout = self._widget.findChild(QVBoxLayout, "plan_time_layout")
        self.solved_layout = self._widget.findChild(QVBoxLayout, "solved_layout")
        self.experiments_info = self._widget.findChild(QTextBrowser, "experiments_info")

        self.perquery_quality_1_layout = self._widget.findChild(QVBoxLayout, "perquery_quality_1_layout")

        self.connect_to_database("example_benchmark.db")
        # rospy.sleep(3.0)
        self.plotStatistics()
        self.plotStatisticsPerQuery()
        self.setExperimentsInfo()

        # Scene
        self.createScenePlugin()
        self.scene_label = self._widget.findChild(QLabel, "scene_label")
        scene_name = self.findScene()
        self.scene_label.setText(scene_name)
        self.loadSceneFile(scene_name)

        # self.scene_view = self._widget.findChild(QPushButton, "scene_view")
        # self.scene_view.clicked.connect()

    def destruct(self):
        self._widget = None
        rospy.loginfo("Closing planner benchmarks visualizer")

    def connect_to_database(self, db_name):
        db_folder_path = rospkg.RosPack().get_path('sr_moveit_planner_benchmarking')
        db_path = db_folder_path + "/data/" + db_name
        conn = sqlite3.connect(db_path)
        self.c = conn.cursor()

    def setExperimentsInfo(self):
        self.c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
        experiments = self.c.fetchall()
        for experiment in experiments:
            self.c.execute("""SELECT count(*) FROM runs WHERE runs.experimentid = %d
                           GROUP BY runs.plannerid""" % experiment[0])
            numRuns = [run[0] for run in self.c.fetchall()]
            numRuns = numRuns[0] if len(set(numRuns)) == 1 else ','.join(numRuns)

            self.experiments_info.append('* Experiment "%s":' % experiment[1])
            self.experiments_info.append('   Number of averaged runs: %d' % numRuns)
            self.experiments_info.append("   Time limit per run: %g seconds" % experiment[2])
            self.experiments_info.append("   Memory limit per run: %g MB" % experiment[3])

    def plotAttribute(self, cur, planners, attribute, typename, layout):
        labels = []
        measurements = []
        nanCounts = []
        if typename == 'ENUM':
            cur.execute('SELECT description FROM enums where name IS "%s"' % attribute)
            descriptions = [t[0] for t in cur.fetchall()]
            numValues = len(descriptions)
        for planner in planners:
            cur.execute('SELECT %s FROM runs WHERE plannerid = %s AND %s IS NOT NULL' \
                        % (attribute, planner[0], attribute))
            measurement = [t[0] for t in cur.fetchall() if t[0] != None]
            if len(measurement) > 0:
                cur.execute('SELECT count(*) FROM runs WHERE plannerid = %s AND %s IS NULL' \
                            % (planner[0], attribute))
                nanCounts.append(cur.fetchone()[0])
                labels.append(planner[1])
                if typename == 'ENUM':
                    scale = 100. / len(measurement)
                    measurements.append([measurement.count(i) * scale for i in range(numValues)])
                else:
                    measurements.append(measurement)

        if len(measurements) == 0:
            print('Skipping "%s": no available measurements' % attribute)
            return

        # Add the unit if the attribute is known
        attribute = attribute.replace('_', ' ')
        if "time" in attribute:
            attribute += ' (s)'
        elif "clearance" in attribute:
            attribute += ' (m)'
        elif "cartesian" in attribute:
            attribute += ''
        elif "length" in attribute or "quality" in attribute or "smoothness" in attribute:
            attribute += ' (rad)'
        elif typename == "BOOLEAN":
            attribute += ' (%)'

        plt.clf()
        # GUI
        width = 5
        height = 4
        dpi = 100
        fig = Figure(figsize=(width, height), dpi=dpi, facecolor=(1.0, 1.0, 1.0, 1.0))
        ax = fig.add_subplot(111)

        figcanvas = FigureCanvas(fig)
        figcanvas.setParent(self._widget)

        FigureCanvas.setSizePolicy(figcanvas, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(figcanvas)

        # ax = plt.gca()
        if typename == 'ENUM':
            width = .5
            measurements = np.transpose(np.vstack(measurements))
            colsum = np.sum(measurements, axis=1)
            rows = np.where(colsum != 0)[0]
            heights = np.zeros((1, measurements.shape[1]))
            ind = range(measurements.shape[1])
            for i in rows:
                ax.bar(ind, measurements[i], width, bottom=heights[0],
                       color=matplotlib.cm.hot(int(floor(i * 256 / numValues))),
                       label=descriptions[i])
                heights = heights + measurements[i]
            plt.setp(ax, xticks=[x + width / 2. for x in ind])
            box = ax.get_position()
            ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
            props = matplotlib.font_manager.FontProperties()
            props.set_size('small')
            ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop=props)
        elif typename == 'BOOLEAN':
            width = .5
            measurementsPercentage = [sum(m) * 100. / len(m) for m in measurements]
            ind = range(len(measurements))
            ax.bar(ind, measurementsPercentage, width)
            plt.setp(ax, xticks=[x + width / 2. for x in ind])
        else:
            if int(matplotlibversion.split('.')[0]) < 1:
                ax.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5)
            else:
                ax.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5, bootstrap=1000)

        xtickNames = plt.setp(ax, xticklabels=labels)
        plt.setp(xtickNames, rotation=90)
        for tick in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)
        for tick in ax.yaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)
        fig.subplots_adjust(bottom=0.3, top=0.95, left=0.1, right=0.98)  # Squish the plot into the upper 2/3 of the page.  Leave room for labels

        ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
        if max(nanCounts) > 0:
            maxy = max([max(y) for y in measurements])
            for i in range(len(labels)):
                x = i + width / 2 if typename == 'BOOLEAN' else i + 1
                ax.text(x, .95 * maxy, str(nanCounts[i]), horizontalalignment='center', size='small')

        layout.addWidget(figcanvas)

    def plotStatistics(self):
        self.c.execute('PRAGMA FOREIGN_KEYS = ON')
        self.c.execute('SELECT id, name FROM plannerConfigs')
        planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '').replace('kConfigDefault',''))
                    for t in self.c.fetchall()]
        planners = sorted(planners, key=lambda a: a[1])

        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:
            if "path_simplify_clearance" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.clearance_layout)
            elif "path_simplify_correct" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.correct_layout)
            elif "path_simplify_length" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.lenght_layout)
            elif "path_simplify_plan_quality" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.quality_1_layout)
            if "path_simplify_plan_quality_cartesian" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.quality_2_layout)
            if "path_simplify_smoothness" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.smoothness_layout)
            if "time" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.plan_time_layout)
            if "solved" == col[1]:
                self.plotAttribute(self.c, planners, col[1], col[2], self.solved_layout)

    def plotStatisticsPerQuery(self):
        self.c.execute('PRAGMA FOREIGN_KEYS = ON')
        self.c.execute('SELECT id, name FROM plannerConfigs')
        planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '').replace('kConfigDefault',''))
                    for t in self.c.fetchall()]
        planners = sorted(planners, key=lambda a: a[1])

        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:
            # if "path_simplify_clearance" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.clearance_layout)
            # elif "path_simplify_correct" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.correct_layout)
            # elif "path_simplify_length" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.lenght_layout)
            if "path_simplify_plan_quality" == col[1]:
                self.plotAttributePerQuery(self.c, planners, col[1], col[2], self.perquery_quality_1_layout)
            # if "path_simplify_plan_quality_cartesian" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.quality_2_layout)
            # if "path_simplify_smoothness" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.smoothness_layout)
            # if "time" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.plan_time_layout)
            # if "solved" == col[1]:
            #     self.plotAttribute(self.c, planners, col[1], col[2], self.solved_layout)

    def plotAttributePerQuery(self, cur, planners, attribute, typename, layout):
        labels = []
        measurements = []
        nanCounts = []
        if typename == 'ENUM':
            cur.execute('SELECT description FROM enums where name IS "%s"' % attribute)
            descriptions = [t[0] for t in cur.fetchall()]
            numValues = len(descriptions)
        for planner in planners:
            print attribute, planner[0], attribute
            cur.execute('SELECT %s FROM runs WHERE plannerid = %s AND %s IS NOT NULL' \
                        % (attribute, planner[0], attribute))
            measurement = [t[0] for t in cur.fetchall() if t[0] != None]
            # print "measurement:", measurement
            if len(measurement) > 0:
                cur.execute('SELECT count(*) FROM runs WHERE plannerid = %s AND %s IS NULL' \
                            % (planner[0], attribute))
                nanCounts.append(cur.fetchone()[0])
                labels.append(planner[1])
                if typename == 'ENUM':
                    scale = 100. / len(measurement)
                    measurements.append([measurement.count(i) * scale for i in range(numValues)])
                else:
                    measurements.append(measurement)

        # Find the number of runs
        cur.execute('SELECT runcount FROM experiments WHERE id = 1')
        runcount = cur.fetchall()[0][0]

        # Find number of queries
        cur.execute('SELECT count(id) FROM experiments')
        num_queries = cur.fetchall()[0][0]

        if len(measurements) == 0:
            print('Skipping "%s": no available measurements' % attribute)
            return

        # Add the unit if the attribute is known
        attribute = attribute.replace('_', ' ')
        if "time" in attribute:
            attribute += ' (s)'
        elif "clearance" in attribute:
            attribute += ' (m)'
        elif "cartesian" in attribute:
            attribute += ''
        elif "length" in attribute or "quality" in attribute or "smoothness" in attribute:
            attribute += ' (rad)'
        elif typename == "BOOLEAN":
            attribute += ' (%)'

        plt.clf()
        # GUI
        width = 5
        height = 4
        dpi = 100
        fig = Figure(figsize=(width, height), dpi=dpi, facecolor=(1.0, 1.0, 1.0, 1.0))
        ax = fig.add_subplot(111)

        figcanvas = FigureCanvas(fig)
        figcanvas.setParent(self._widget)

        FigureCanvas.setSizePolicy(figcanvas, QSizePolicy.Expanding, QSizePolicy.Expanding)
        FigureCanvas.updateGeometry(figcanvas)

        # ax = plt.gca()
        if typename == 'ENUM':
            width = .5
            measurements = np.transpose(np.vstack(measurements))
            colsum = np.sum(measurements, axis=1)
            rows = np.where(colsum != 0)[0]
            heights = np.zeros((1, measurements.shape[1]))
            ind = range(measurements.shape[1])
            for i in rows:
                ax.bar(ind, measurements[i], width, bottom=heights[0],
                       color=matplotlib.cm.hot(int(floor(i * 256 / numValues))),
                       label=descriptions[i])
                heights = heights + measurements[i]
            plt.setp(ax, xticks=[x + width / 2. for x in ind])
            box = ax.get_position()
            ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])
            props = matplotlib.font_manager.FontProperties()
            props.set_size('small')
            ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), prop=props)
        elif typename == 'BOOLEAN':
            width = .5
            measurementsPercentage = [sum(m) * 100. / len(m) for m in measurements]
            ind = range(len(measurements))
            ax.bar(ind, measurementsPercentage, width)
            plt.setp(ax, xticks=[x + width / 2. for x in ind])
        else:

            if int(matplotlibversion.split('.')[0]) < 1:
                ax.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5)
            else:
                for planner in range(len(planners)):
                    # TODO: REMOVE
                    if planner == 0:
                        print "Here 4"
                        print len(planners)
                        print len(measurements[0])
                        print "num_queries", num_queries
                        print "runcount", runcount
                        matrix_measurements = np.array(measurements[0])
                        print matrix_measurements.shape
                        matrix_measurements = matrix_measurements.reshape(num_queries, runcount)
                        ax.boxplot(matrix_measurements, notch=0, sym='k+', vert=1, whis=1.5, bootstrap=1000)
                        # ax.plot()






        xtickNames = plt.setp(ax, xticklabels=labels)
        plt.setp(xtickNames, rotation=90)
        for tick in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)
        for tick in ax.yaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)
        fig.subplots_adjust(bottom=0.3, top=0.95, left=0.1, right=0.98)  # Squish the plot into the upper 2/3 of the page.  Leave room for labels

        ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
        if max(nanCounts) > 0:
            maxy = max([max(y) for y in measurements])
            for i in range(len(labels)):
                x = i + width / 2 if typename == 'BOOLEAN' else i + 1
                ax.text(x, .95 * maxy, str(nanCounts[i]), horizontalalignment='center', size='small')

        layout.addWidget(figcanvas)

    def createScenePlugin(self):
        package_path = rospkg.RosPack().get_path('sr_moveit_planner_benchmarking')
        rviz_config_approach = package_path + "/config/scene.rviz"

        reader = rviz.YamlConfigReader()

        # Configuring approach window
        config_approach = rviz.Config()
        reader.readFile(config_approach, rviz_config_approach)
        self.frame_scene = rviz.VisualizationFrame()
        self.frame_scene.setSplashPath("")
        self.frame_scene.initialize()
        self.frame_scene.setMenuBar(None)
        self.frame_scene.setStatusBar(None)
        self.frame_scene.setHideButtonVisibility(False)
        self.frame_scene.load(config_approach)

        scene_layout = self._widget.findChild(QVBoxLayout, "scene_layout")
        scene_layout.addWidget(self.frame_scene)

    def loadSceneFile(self, scene_name):
        try:
            scenes_path = "`rospack find sr_moveit_planner_benchmarking`/scenes/" + scene_name + ".scene"
            p = subprocess.Popen(['rosrun moveit_ros_planning moveit_publish_scene_from_text {}'.format(scenes_path)],
                                 shell=True)
        except rospy.ROSException as e:
            rospy.logerr("There was an error loading the scene: ", scene_name)
            rospy.logerr(e)
            return

    def findScene(self):
        self.c.execute("""SELECT setup FROM experiments""")
        setups = self.c.fetchall()

        for setup in setups:
            setup_info = setup[0]
            string_to_find = "Planning scene: \nname: "
            i_name_start = setup_info.find(string_to_find) + len(string_to_find)
            i_name_end = setup_info.find("\n", i_name_start, i_name_start + 100)
            if i_name_start > 0 and i_name_end > 0:
                scene_name = setup_info[i_name_start:i_name_end]
                break
        return scene_name


if __name__ == "__main__":
    rospy.init_node("moveit_planner_visualizer")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrMoveitPlannerBenchmarksVisualizer(None)
    planner_benchmarking_gui._widget.show()
    app.exec_()
    planner_benchmarking_gui.destruct()
