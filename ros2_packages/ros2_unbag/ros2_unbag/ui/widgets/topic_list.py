# MIT License

# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""
Topic List Widget Module.

Provides the TopicListWidget class for displaying ROS2 bag topics in a filterable,
checkable list. This widget appears in the left column of the GUI and manages
topic selection state for the export process.
"""

from PySide6 import QtCore, QtWidgets, QtGui

from ros2_unbag.ui.styles import TOPIC_LIST_STYLE, GRAY_SOFT, CHECKED_BG

__all__ = ["TopicListWidget"]


class TopicListWidget(QtWidgets.QWidget):
    """
    Widget for displaying and selecting ROS2 bag topics.
    
    This widget provides a list view of all topics in a bag file, allowing users to:
    - Select topics for export via checkboxes
    - Filter topics by name using a search box
    - Select all or clear all topics with bulk actions
    - Click on a topic row to view its detailed settings
    
    Topics are displayed with their name, message type, and message count.
    The widget maintains a mapping of topic names to their list items for efficient
    state tracking and updates.
    
    Signals:
        topic_selected (str): Emitted when a topic row is clicked, passes the topic name
        topic_toggled (str, bool): Emitted when a topic checkbox changes, passes topic name and new state
    """
    
    # Signal emitted when a topic is selected for editing (name)
    topic_selected = QtCore.Signal(str)
    # Signal emitted when a topic's export inclusion changes (name, is_checked)
    topic_toggled = QtCore.Signal(str, bool)

    def __init__(self, parent=None):
        """
        Initialize the TopicListWidget with UI components.

        Args:
            parent: Optional parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.topics = {}  # topic_name -> QTreeWidgetItem mapping
        self.init_ui()

    def init_ui(self):
        """
        Build the topic list UI with header, filter, list widget, and selection buttons.

        Args:
            None

        Returns:
            None
        """
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Search/Filter (Placeholder for now, or simple implementation)
        self.filter_edit = QtWidgets.QLineEdit()
        self.filter_edit.setPlaceholderText("Filter topics...")
        self.filter_edit.textChanged.connect(self.filter_topics)
        self.filter_edit.setMinimumHeight(32)
        layout.addWidget(self.filter_edit)

        # List
        self.tree_widget = QtWidgets.QTreeWidget()
        self.tree_widget.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.tree_widget.setHeaderLabels(["Topic", "# Msgs"])
        self.tree_widget.setColumnCount(2)
        header = self.tree_widget.header()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        header.setStretchLastSection(False)
        self.tree_widget.setRootIsDecorated(False)
        self.tree_widget.setStyleSheet(TOPIC_LIST_STYLE)
        self.tree_widget.itemClicked.connect(self.on_item_clicked)
        self.tree_widget.itemChanged.connect(self.on_item_changed)
        layout.addWidget(self.tree_widget)

        # Bottom controls
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_select_all = QtWidgets.QPushButton("All")
        self.btn_select_all.clicked.connect(self.select_all)
        self.btn_select_none = QtWidgets.QPushButton("None")
        self.btn_select_none.clicked.connect(self.select_none)
        
        btn_layout.addWidget(self.btn_select_all)
        btn_layout.addWidget(self.btn_select_none)
        layout.addLayout(btn_layout)

    def load_topics(self, topics_dict, message_counts):
        """
        Load topics from a bag file into the list widget.
        
        Creates a custom list item for each topic with a checkbox, topic name,
        message type, and message count. Topics are sorted alphabetically.

        Args:
            topics_dict: Dictionary mapping message types to lists of topic names.
            message_counts: Dictionary mapping topic names to message counts.

        Returns:
            None
        """
        self.tree_widget.clear()
        self.topics = {}

        # Build grouped tree: message type -> topics
        header_font = QtGui.QFont(self.tree_widget.font())
        header_font.setBold(True)
        for msg_type in sorted(topics_dict.keys()):
            parent = QtWidgets.QTreeWidgetItem([msg_type, ""])
            parent.setFirstColumnSpanned(True)
            parent.setFlags(QtCore.Qt.ItemIsEnabled)
            parent.setFont(0, header_font)
            bg = QtGui.QColor(GRAY_SOFT)
            parent.setBackground(0, bg)
            parent.setBackground(1, bg)
            self.tree_widget.addTopLevelItem(parent)

            for topic in sorted(topics_dict[msg_type]):
                count = message_counts.get(topic, 0)
                child = QtWidgets.QTreeWidgetItem([topic, f"{count}"])
                child.setData(0, QtCore.Qt.UserRole, topic)
                child.setFlags(
                    QtCore.Qt.ItemIsEnabled
                    | QtCore.Qt.ItemIsSelectable
                    | QtCore.Qt.ItemIsUserCheckable
                )
                child.setCheckState(0, QtCore.Qt.Unchecked)
                child.setTextAlignment(1, QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
                parent.addChild(child)
                self.topics[topic] = child

        self.tree_widget.expandAll()
        self.tree_widget.resizeColumnToContents(1)

    def on_item_clicked(self, item):
        """
        Handle topic item click events and emit topic_selected signal.
        
        Finds the topic name corresponding to the clicked item and emits
        the topic_selected signal to notify listeners.

        Args:
            item: The QListWidgetItem that was clicked.

        Returns:
            None
        """
        if item and item.parent():  # ignore group headers
            topic = item.data(0, QtCore.Qt.UserRole)
            if topic:
                self.topic_selected.emit(topic)

    def on_item_changed(self, item, column):
        """
        Handle checkbox state changes and emit topic_toggled signal.
        
        Called whenever a checkbox state changes in the topic list.
        Emits the topic_toggled signal with the topic name and new state.
        
        Args:
            item: The QTreeWidgetItem that changed.
            column: The column index that changed (0 for checkbox).
        """
        if column == 0 and item and item.parent():  # Only process checkbox changes on child items
            topic = item.data(0, QtCore.Qt.UserRole)
            if topic:
                is_checked = item.checkState(0) == QtCore.Qt.Checked
                self._apply_checked_style(item, is_checked)
                self.topic_toggled.emit(topic, is_checked)

    def filter_topics(self, text):
        """
        Filter the topic list based on search text.
        
        Hides topics that don't contain the search text (case-insensitive).
        Empty search text shows all topics.

        Args:
            text: Search string to filter topics by.

        Returns:
            None
        """
        search_text = text.lower()
        for topic_name, item in self.topics.items():
            # Show item if search text is empty or found in topic name
            matches = search_text in topic_name.lower() if search_text else True
            item.setHidden(not matches)
            
            # Optionally also search in message type (parent's text)
            if not matches and search_text and item.parent():
                msg_type = item.parent().text(0)
                matches = search_text in msg_type.lower()
                item.setHidden(not matches)

        # Hide parent groups that have no visible children
        for i in range(self.tree_widget.topLevelItemCount()):
            parent = self.tree_widget.topLevelItem(i)
            visible_children = any(not parent.child(j).isHidden() for j in range(parent.childCount()))
            parent.setHidden(not visible_children)

    def select_all(self):
        """
        Check all topic checkboxes to select all topics for export.

        Args:
            None

        Returns:
            None
        """
        for item in self.topics.values():
            item.setCheckState(0, QtCore.Qt.Checked)

    def select_none(self):
        """
        Uncheck all topic checkboxes to deselect all topics.

        Args:
            None

        Returns:
            None
        """
        for item in self.topics.values():
            item.setCheckState(0, QtCore.Qt.Unchecked)

    def is_checked(self, topic):
        """
        Return True if the given topic is currently checked for export.
        
        Args:
            topic: Topic name string to check.
        
        Returns:
            bool: True if topic is checked, False otherwise.
        """
        item = self.topics.get(topic)
        return item.checkState(0) == QtCore.Qt.Checked if item else False

    def set_checked(self, topic, checked, *, block_signals=False):
        """
        Set the checked state of a topic row.
        
        Args:
            topic (str): The name of the topic to set the state for.
            checked (bool): True to check the topic, False to uncheck.
            block_signals (bool): If True, signals will be temporarily blocked
                                  to prevent `on_item_changed` from being called.
        """
        item = self.topics.get(topic)
        if not item:
            return
        if block_signals:
            self.tree_widget.blockSignals(True)
        item.setCheckState(0, QtCore.Qt.Checked if checked else QtCore.Qt.Unchecked)
        self._apply_checked_style(item, checked)
        if block_signals:
            self.tree_widget.blockSignals(False)

    def _apply_checked_style(self, item, checked):
        """
        Apply visual styling to highlight checked items.
        
        Changes the background color of checked items to provide clear
        visual feedback about export selection state.
        
        Args:
            item: QTreeWidgetItem to style.
            checked: Boolean indicating if item is checked.
        """
        if checked:
            for col in range(item.columnCount()): # Apply to all columns
                item.setBackground(col, QtGui.QBrush(QtGui.QColor(CHECKED_BG)))
        else:
            for col in range(item.columnCount()): # Clear background for all columns
                item.setBackground(col, QtGui.QBrush())
