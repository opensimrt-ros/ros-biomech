#!/usr/bin/env python3

import os
import sip
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileSystemModel, QTreeView, QDirModel
from python_qt_binding.QtGui import QIcon
import python_qt_binding.QtGui as QtGui
import python_qt_binding.QtCore as QtCore

def print_events(obj):
    rospy.loginfo(obj)
    a = dir(obj)
    for prop in a:
        if "Event" in prop:
            rospy.logwarn(prop)

def check_if_lib_moment_arm_exists_at_path(some_file_with_complete_path):
    directory = os.path.dirname(some_file_with_complete_path)
    file_shenanigans = os.path.basename(some_file_with_complete_path)
    filename, extension = os.path.splitext(file_shenanigans)
    rospy.logdebug(directory)
    rospy.logdebug(file_shenanigans)
    desired_lib_name = construct_lib_name_from_osim_name(filename)

    lib_path =os.path.join(directory,desired_lib_name)
    return os.path.exists(lib_path), lib_path

def construct_lib_name_from_osim_name(osim_name):
    return "libMomentArm_"+osim_name+".so"

def create_path_label(subject_id, activity_name, session_num):
    rospy.loginfo(subject_id)
    rospy.loginfo(activity_name)
    rospy.loginfo(session_num)
    return os.path.join("/srv/host_data/RTValidation", subject_id,session_num)

class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            print('unknowns: ', unknowns)

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_acquisition'), 'resource', 'MyPlugin.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface

        self._widget.start_button.setIcon(QIcon.fromTheme('media-record'))
        self._widget.start_button.clicked[bool].connect(self._handle_start_clicked)
        self._widget.stop_button.setIcon(QIcon.fromTheme('media-playback-stop'))
        self._widget.stop_button.clicked[bool].connect(self._handle_stop_clicked)

        self._widget.generate_lib_moment_arm_button.clicked[bool].connect(self._handle_lib_moment_clicked)
        

        if True:
            model = QFileSystemModel()

            model.setRootPath("/srv/host_data/models")
            model.removeColumns(1,2)
            model.setNameFilters(["*.osim"])
            model.setNameFilterDisables(False)
        if False:
            rospy.logwarn(dir(model))
            rospy.logwarn(model.columnCount())
            rospy.logwarn(model.removeColumn(1))
            rospy.logwarn(model.beginRemoveColumns)
            rospy.logwarn(help(model.beginRemoveColumns))
            rospy.logwarn(model.columnCount())
            rospy.logwarn(model.rootPath())
            rospy.logwarn(model.rootDirectory())
            #rospy.logwarn(model.size())
            #rospy.logwarn(model.event())

        if True:
            #rospy.logerr(dir(self._widget.model_selector.SelectedClicked))
            #self._widget.keyPressEvent = self._handle_model_changed_keypress

            self._widget.model_selector.setModel(model)
            self._widget.model_selector.setColumnHidden(1,True)
            self._widget.model_selector.setColumnHidden(2,True)
            self._widget.model_selector.setColumnHidden(3,True)
            self._widget.model_selector.expandAll()
            
            self._widget.model_selector.viewport().installEventFilter(self)


        
        self.model_path = ""
        self.lib_path = ""
        self.lib_path_exists = False
        self.activity_name = ""
        self.subject_id = ""
        self.session_num = ""
        self.save_path = ""

        print_events(self._widget.activity_name)
        
        self._widget.activity_name.textChanged.connect(self.update_paths)
        self._widget.session_name.textChanged.connect(self.update_paths)
        self._widget.subject_id_name.textChanged.connect(self.update_paths)
        #self._widget.subject_id_name.changeEvent = self.update_paths


            #print_events(self._widget.model_selector)
        context.add_widget(self._widget)
    def update_paths(self, event=None):
        ## update save_path from subjectid activity and session
        self.subject_id = self._widget.subject_id_name.text()
        self.activity_name = self._widget.activity_name.text()
        self.session_num = self._widget.session_name.text()
                

        self.save_path = create_path_label( self.subject_id, 
                                            self.activity_name,
                                            self.session_num
                ) 

        self._widget.resolved_path_name.setText(   self.save_path )
        self.activity_name = self._widget.activity_name.text()

    def update_things(self, event=None):
        self.model_path = self._widget.model_selected_name.text()
        self.lib_path_exists, self.lib_path = check_if_lib_moment_arm_exists_at_path(self.model_path)
        if self.lib_path_exists:
            self._widget.lib_moment_arm_text.setText("[V] "+self.lib_path)
        else:
            self._widget.lib_moment_arm_text.setText("[X] "+self.lib_path)
            


    def _handle_lib_moment_clicked(self):
        rospy.loginfo("lib_moment clicked!")
    
    def _handle_start_clicked(self):
        rospy.loginfo("start clicked!")
    
    def _handle_stop_clicked(self):
        rospy.loginfo("stop clicked!")

    def _handle_model_changed_keypress(self,event):

        key = event.key()
        ## this only works for keyboard presses
        if key == self._widget.model_selector.SelectedClicked:
            rospy.loginfo("SelectedClicked")
        if key == self._widget.model_selector.DoubleClicked:
            rospy.loginfo("DoubleClicked")
        rospy.loginfo(dir(event))
        rospy.loginfo(key)

    def eventFilter(self, source, event):
        rospy.logdebug("something")
        self.update_things()
        if not sip.isdeleted(self._widget.model_selector) and source is self._widget.model_selector.viewport():
            rospy.logdebug("i am from the model selector")
            if isinstance(event, QtGui.QMouseEvent):
                rospy.logdebug("i am a mouse event")
                #rospy.loginfo(event.buttons())
                #rospy.loginfo(dir(event))
                #rospy.loginfo(dir(event.flags()))
                #rospy.loginfo(event.modifiers())
                if event.type() == QtCore.QEvent.MouseButtonDblClick:
                    #rospy.logwarn('meta-double-click')
                    index = self._widget.model_selector.selectedIndexes()[0]
                    info = self._widget.model_selector.model().fileInfo(index)
                    if ".osim" in info.absoluteFilePath():
                        self.model_path = info.absoluteFilePath()
                        self._widget.model_selected_name.setText(self.model_path)
                        #rospy.logwarn(self.model_path)
                        self.lib_path_exists, self.lib_path = check_if_lib_moment_arm_exists_at_path(self.model_path) 
                    return True
                ## this is not working
                if event.modifiers() == QtCore.Qt.MetaModifier:
                    rospy.loginfo("i am a MetaModifier")
                    if event.type() == QtCore.QEvent.MouseButtonDblClick:
                        rospy.logwarn('meta-double-click')
                        return True
                    if event.type() == QtCore.QEvent.MouseButtonPress:
                        # kill selection when meta-key is also pressed
                        return True
        return super(MyPlugin, self).eventFilter(source, event)


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

