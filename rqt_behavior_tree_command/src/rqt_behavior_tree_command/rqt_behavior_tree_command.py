from curses.panel import bottom_panel
import os
import time
import rospy
import rospkg
from std_msgs.msg import String, Bool
from behavior_tree_msgs.msg import Status
import numpy as np
import yaml
import collections

from qt_gui.plugin import Plugin
import python_qt_binding.QtWidgets as qt
import python_qt_binding.QtCore as core
import python_qt_binding.QtGui as gui

from python_qt_binding import QT_BINDING, QT_BINDING_VERSION

from python_qt_binding.QtCore import Slot, Qt, qVersion, qWarning, Signal
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from behavior_tree import behavior_tree as bt
from behavior_tree_msgs.msg import Status, BehaviorTreeCommand, BehaviorTreeCommands

class BehaviorTreeCommandPlugin(Plugin):
    def __init__(self, context):
        super(BehaviorTreeCommandPlugin, self).__init__(context)
        self.setObjectName('BehaviorTreeCommandPlugin')

        self.config_filename = ''
        
        self.button_groups = {}
        
        self.command_pub = rospy.Publisher('behavior_tree_commands', BehaviorTreeCommands, queue_size=10)

        # main layout
        self.widget = QWidget()
        self.vbox = qt.QVBoxLayout()
        self.widget.setLayout(self.vbox)
        context.add_widget(self.widget)

        # config widget
        self.config_widget = qt.QWidget()
        self.config_widget.setStyleSheet('QWidget{margin-left:-1px;}')
        self.config_layout = qt.QHBoxLayout()
        self.config_widget.setLayout(self.config_layout)
        self.config_widget.setFixedHeight(50)

        self.config_button = qt.QPushButton('Open Config...')
        self.config_button.clicked.connect(self.select_config_file)
        self.config_layout.addWidget(self.config_button)

        self.config_label = qt.QLabel('config filename: ')
        self.config_layout.addWidget(self.config_label)
        self.config_label.setMinimumWidth(1)
        self.vbox.addWidget(self.config_widget)

        # button widget
        self.button_widget = qt.QWidget()
        self.button_layout = qt.QVBoxLayout()
        self.button_widget.setLayout(self.button_layout)
        self.vbox.addWidget(self.button_widget)

    def select_config_file(self):
        starting_path = os.path.join(rospkg.RosPack().get_path('rqt_behavior_tree_command'), 'config')
        filename = qt.QFileDialog.getOpenFileName(self.widget, 'Open Config File', starting_path, "Config Files (*.yaml)")[0]
        self.set_config(filename)

    def set_config(self, filename):
        if filename != '':
            self.config_filename = filename
            if self.config_filename != None:
                self.config_label.setText('config filename: ' + self.config_filename)
                self.init_buttons(filename)

    def init_buttons(self, filename):
        y = yaml.load(open(filename, 'r').read())
        print(y)

        def get_click_function(group, button):
            def click_function():
                commands = BehaviorTreeCommands()
                for i in range(len(self.button_groups[group]['buttons'])):
                    b = self.button_groups[group]['buttons'][i]
                    command = BehaviorTreeCommand()
                    command.condition_name = self.button_groups[group]['condition_names'][i]
                    
                    if b != button and b.isChecked():
                        b.toggle()
                        command.status = Status.FAILURE
                    elif b == button and not b.isChecked():
                        command.status = Status.FAILURE
                    elif b == button and b.isChecked():
                        command.status = Status.SUCCESS
                    commands.commands.append(command)
                self.command_pub.publish(commands)
            return click_function
        
        for group in y['groups']:
            group_name = next(iter(group.keys()))
            if group_name not in self.button_groups.keys():
                self.button_groups[group_name] = {'buttons' : [], 'condition_names': []}

            group_widget = qt.QWidget()
            group_layout = qt.QVBoxLayout()
            group_widget.setLayout(group_layout)
            self.button_layout.addWidget(group_widget)

            group_layout.addWidget(qt.QLabel(group_name))

            button_widget = qt.QWidget()
            button_layout = qt.QVBoxLayout()
            button_widget.setLayout(button_layout)
            group_layout.addWidget(button_widget)
            
            for buttons in group[group_name]:
                button_name = next(iter(buttons.keys()))
                condition_name = buttons[button_name]['condition_name']
                button = qt.QPushButton(button_name)
                button.clicked.connect(get_click_function(group_name, button))
                button.setCheckable(True)
                button_layout.addWidget(button)

                print(condition_name, bt.get_condition_topic_name(condition_name))
                self.button_groups[group_name]['buttons'].append(button)
                self.button_groups[group_name]['condition_names'].append(condition_name)
        
    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('config_filename', self.config_filename)

    def restore_settings(self, plugin_settings, instance_settings):
        self.set_config(instance_settings.value('config_filename'))

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

