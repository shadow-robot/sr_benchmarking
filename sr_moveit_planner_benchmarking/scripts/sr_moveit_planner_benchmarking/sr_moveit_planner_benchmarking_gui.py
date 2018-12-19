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

import signal
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
        self.create_menu_bar()

        self.available_databases = []
        self.planners = []

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_moveit_planner_benchmarking'), 'uis', 'moveit_planner_benchmarking.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Moveit Planner Benchmarks")
        self.init_widget_children()
        self.create_scene_plugin()
        self.load_db_button.clicked.connect(self.load_database)
        self.planners_combo_box.currentIndexChanged.connect(self.change_plots_per_query)

        # Load databases from this repo by default
        self.get_available_databases_from_default_path()

    def init_widget_children(self):
        self.clearance_layout = self._widget.findChild(QVBoxLayout, "clearance_layout")
        self.correct_layout = self._widget.findChild(QVBoxLayout, "correct_layout")
        self.lenght_layout = self._widget.findChild(QVBoxLayout, "lenght_layout")
        self.quality_1_layout = self._widget.findChild(QVBoxLayout, "quality_1_layout")
        self.quality_2_layout = self._widget.findChild(QVBoxLayout, "quality_2_layout")
        self.smoothness_layout = self._widget.findChild(QVBoxLayout, "smoothness_layout")
        self.plan_time_layout = self._widget.findChild(QVBoxLayout, "plan_time_layout")
        self.solved_layout = self._widget.findChild(QVBoxLayout, "solved_layout")

        self.perquery_clearance_layout = self._widget.findChild(QVBoxLayout, "perquery_clearance_layout")
        self.perquery_correct_layout = self._widget.findChild(QVBoxLayout, "perquery_correct_layout")
        self.perquery_lenght_layout = self._widget.findChild(QVBoxLayout, "perquery_lenght_layout")
        self.perquery_quality_1_layout = self._widget.findChild(QVBoxLayout, "perquery_quality_1_layout")
        self.perquery_quality_2_layout = self._widget.findChild(QVBoxLayout, "perquery_quality_2_layout")
        self.perquery_smoothness_layout = self._widget.findChild(QVBoxLayout, "perquery_smoothness_layout")
        self.perquery_plan_time_layout = self._widget.findChild(QVBoxLayout, "perquery_plan_time_layout")
        self.perquery_solved_layout = self._widget.findChild(QVBoxLayout, "perquery_solved_layout")

        self.experiments_info = self._widget.findChild(QTextBrowser, "experiments_info")
        self.queries_legend = self._widget.findChild(QTextBrowser, "queries_legend")
        self.scene_label = self._widget.findChild(QLabel, "scene_label")
        self.dbs_combo_box = self._widget.findChild(QComboBox, "dbs_combo_box")
        self.planners_combo_box = self._widget.findChild(QComboBox, "planners_combo_box")
        self.load_db_button = self._widget.findChild(QPushButton, "load_button")

    def destruct(self):
        self._widget = None
        rospy.loginfo("Closing planner benchmarks visualizer")

    def update_data_display(self):
        self.set_planners_combobox()
        self.plot_statistics()
        self.change_plots_per_query()
        self.set_experiments_info()
        scene_name = self.find_scene()
        self.scene_label.setText(scene_name)
        self.load_scene_file(scene_name)

    def load_database(self):
        path_to_db = None
        db_to_be_loaded = self.dbs_combo_box.currentText()
        for db in self.available_databases:
            if db_to_be_loaded == db['rel_path']:
                path_to_db = db['full_path']
                break
        if path_to_db is not None:
            self.connect_to_database(path_to_db)
            self.get_planners_list()
            self.update_data_display()

    def create_menu_bar(self):
        self._widget.myQMenuBar = QMenuBar(self._widget)
        fileMenu = self._widget.myQMenuBar.addMenu('&File')
        setPathAction = QAction('Open dbs directory', self._widget)
        setPathAction.triggered.connect(self.get_available_databases_from_path)
        fileMenu.addAction(setPathAction)

    def get_available_databases_from_default_path(self):
        self.dbs_combo_box.clear()
        chosen_path = rospkg.RosPack().get_path('sr_moveit_planner_benchmarking')
        self.find_dbs_in_directory(chosen_path)
        self.dbs_combo_box.clear()
        for db in self.available_databases:
            self.dbs_combo_box.addItem(db['rel_path'])

    def get_available_databases_from_path(self):
        self.dbs_combo_box.clear()
        chosen_path = QFileDialog.getExistingDirectory(self._widget, 'Open file', "")
        self.find_dbs_in_directory(chosen_path)
        self.dbs_combo_box.clear()
        for db in self.available_databases:
            self.dbs_combo_box.addItem(db['rel_path'])

    def find_dbs_in_directory(self, directory):
        for root, dirs, files in os.walk(directory):
            for file in files:
                if file.endswith(".db"):
                    db_full_path = os.path.join(root, file)
                    db_rel_path = os.path.relpath(db_full_path, directory)
                    self.available_databases.append({'rel_path': db_rel_path, 'full_path': db_full_path})

    def connect_to_database(self, db_path):
        conn = sqlite3.connect(db_path)
        self.c = conn.cursor()

    def set_experiments_info(self):
        self.c.execute("""SELECT id, name, timelimit, memorylimit FROM experiments""")
        experiments = self.c.fetchall()
        self.experiments_info.clear()
        for experiment in experiments:
            self.c.execute("""SELECT count(*) FROM runs WHERE runs.experimentid = %d
                           GROUP BY runs.plannerid""" % experiment[0])
            numRuns = [run[0] for run in self.c.fetchall()]
            numRuns = numRuns[0] if len(set(numRuns)) == 1 else ','.join(numRuns)

            self.experiments_info.append('* Experiment "%s":' % experiment[1])
            self.experiments_info.append('   Number of averaged runs: %d' % numRuns)
            self.experiments_info.append("   Time limit per run: %g seconds" % experiment[2])
            self.experiments_info.append("   Memory limit per run: %g MB" % experiment[3])

    def plot_attribute(self, cur, planners, attribute, typename, layout):
        labels = []
        measurements = []
        nanCounts = []
        total_per_planner = []

        if typename == 'ENUM':
            cur.execute('SELECT description FROM enums where name IS "%s"' % attribute)
            descriptions = [t[0] for t in cur.fetchall()]
            numValues = len(descriptions)
        for planner in planners:
            cur.execute('SELECT %s FROM runs WHERE plannerid = %s AND %s IS NOT NULL'
                        % (attribute, planner[0], attribute))
            measurement = [t[0] for t in cur.fetchall() if t[0] != None]
            cur.execute('SELECT count(*) FROM runs WHERE plannerid = %s'
                        % (planner[0]))
            total_per_planner.append(cur.fetchone()[0])

            cur.execute('SELECT count(*) FROM runs WHERE plannerid = %s AND %s IS NULL'
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
            measurementsPercentage = [sum(m) * 100. / total_per_planner[counter] for counter, m in
                                      enumerate(measurements)]
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
        fig.subplots_adjust(bottom=0.3, top=0.95, left=0.1, right=0.98)

        ax.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
        # if max(nanCounts) > 0:
        #     maxy = max([max(y) for y in measurements])
        #     for i in range(len(labels)):
        #         x = i + width / 2 if typename == 'BOOLEAN' else i + 1
        #         ax.text(x, .95 * maxy, str(nanCounts[i]), horizontalalignment='center', size='small')

        self.clearLayout(layout)
        layout.addWidget(figcanvas)

    def plot_attribute_per_query(self, cur, planner, attribute, typename, layout):
        measurements = []
        if typename == 'ENUM':
            # TODO: Implement enum support
            return

        cur.execute('SELECT %s FROM runs WHERE plannerid = %s'
                    % (attribute, planner[0]))
        measurement_including_nan = [t[0] for t in cur.fetchall()]

        if 0 == len(measurement_including_nan):
            rospy.logwarn("No measurements for {} available!".format(attribute))
            self.clearLayout(layout)
            return

        # Find the number of runs
        cur.execute('SELECT runcount FROM experiments WHERE id = 1')
        runcount = cur.fetchall()[0][0]

        # Find names and number of queries
        cur.execute('SELECT id FROM experiments')
        queries = [q[0] for q in cur.fetchall()]
        num_queries = len(queries)

        cur.execute('SELECT experimentid FROM runs WHERE plannerid = %s' % (planner[0]))
        queryid_to_run_mapping = [t[0] for t in cur.fetchall()]

        per_query_runcount = []
        for query_id in queries:
            per_query_runcount.append(queryid_to_run_mapping.count(query_id))

        matrix_measurements_with_nans = []
        for runcount in per_query_runcount:
            matrix_measurements_with_nans.append(measurement_including_nan[0: runcount])
            del measurement_including_nan[0: runcount]

        # Remove NaNs from the measurement array
        matrix_measurements = []
        for per_query_result in matrix_measurements_with_nans:
            matrix_measurements.append([x for x in per_query_result if x is not None])

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

        if typename == 'BOOLEAN':
            width = .5
            measurements_percentage = []
            missing_measurements = []
            missing_measurements_index = []
            for i, m in enumerate(matrix_measurements):
                if 0 == len(m):
                    measurements_percentage.append(0)
                    missing_measurements.append(50)
                    missing_measurements_index.append(i + width / 2)
                else:
                    measurements_percentage.append(sum(m) * 100 / len(m))
            idx = range(len(measurements_percentage))
            ax.bar(idx, measurements_percentage, width)
            ax.scatter(missing_measurements_index, missing_measurements, color='r', marker='x')
            plt.setp(ax, xticks=[x + width / 2 for x in idx], xticklabels=[x + 1 for x in idx])
            ax.set_ylim([0, 100])
            ax.set_xlim([0, len(matrix_measurements)])
        else:
            if int(matplotlibversion.split('.')[0]) < 1:
                ax.boxplot(measurements, notch=0, sym='k+', vert=1, whis=1.5)
            else:
                ax.boxplot(matrix_measurements, notch=0, sym='k+', vert=1, whis=1.5, bootstrap=1000)

        for tick in ax.xaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)
        for tick in ax.yaxis.get_major_ticks():  # shrink the font size of the x tick labels
            tick.label.set_fontsize(8)

        self.clearLayout(layout)
        layout.addWidget(figcanvas)

    def plot_statistics(self):
        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:
            if "path_simplify_clearance" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.clearance_layout)
            elif "path_simplify_correct" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.correct_layout)
            elif "path_simplify_length" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.lenght_layout)
            elif "path_simplify_plan_quality" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.quality_1_layout)
            if "path_simplify_plan_quality_cartesian" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.quality_2_layout)
            if "path_simplify_smoothness" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.smoothness_layout)
            if "time" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.plan_time_layout)
            if "solved" == col[1]:
                self.plot_attribute(self.c, self.planners, col[1], col[2], self.solved_layout)

    def plot_statistics_per_query(self, planner):
        self.c.execute('PRAGMA table_info(runs)')
        colInfo = self.c.fetchall()[3:]

        for col in colInfo:
            if "path_simplify_clearance" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_clearance_layout)
            elif "path_simplify_correct" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_correct_layout)
            elif "path_simplify_length" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_lenght_layout)
            if "path_simplify_plan_quality" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_quality_1_layout)
            if "path_simplify_plan_quality_cartesian" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_quality_2_layout)
            if "path_simplify_smoothness" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_smoothness_layout)
            if "time" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_plan_time_layout)
            if "solved" == col[1]:
                self.plot_attribute_per_query(self.c, planner, col[1], col[2], self.perquery_solved_layout)

        self.c.execute('SELECT name FROM experiments')
        queries = [q[0] for q in self.c.fetchall()]
        num_queries = len(queries)
        self.queries_legend.clear()
        for idx, query in enumerate(queries):
            self.queries_legend.append('Query {}: {}'.format(idx + 1, query))

    def create_scene_plugin(self):
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

        self.load_scene_file("empty")

    def load_scene_file(self, scene_name):
        try:
            scenes_path = "`rospack find sr_moveit_planner_benchmarking`/experiments/scenes/" + scene_name + ".scene"
            p = subprocess.Popen(['rosrun moveit_ros_planning moveit_publish_scene_from_text {}'.format(scenes_path)],
                                 shell=True)
        except rospy.ROSException as e:
            rospy.logerr("There was an error loading the scene: ", scene_name)
            rospy.logerr(e)
            return

    def find_scene(self):
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

    def clearLayout(self, layout):
        for i in reversed(range(layout.count())):
            layout.itemAt(i).widget().setParent(None)

    def get_planners_list(self):
        self.c.execute('PRAGMA FOREIGN_KEYS = ON')
        self.c.execute('SELECT id, name FROM plannerConfigs')
        planners = [(t[0], t[1].replace('geometric_', '').replace('control_', '').replace('kConfigDefault', ''))
                    for t in self.c.fetchall()]
        self.planners = sorted(planners, key=lambda a: a[1])

    def set_planners_combobox(self):
        self.planners_combo_box.blockSignals(True)
        self.planners_combo_box.clear()
        for planner in self.planners:
            self.planners_combo_box.addItem(planner[1])
        self.planners_combo_box.blockSignals(False)

    def change_plots_per_query(self):
        for planner in self.planners:
            if self.planners_combo_box.currentText() == planner[1]:
                self.plot_statistics_per_query(planner)
                return


if __name__ == "__main__":
    rospy.init_node("moveit_planner_visualizer")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrMoveitPlannerBenchmarksVisualizer(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
