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
Processor Chain Widget Module.

Provides the ProcessorChainWidget for building ordered chains of data processors.
Each processor transforms topic data before export, and processors are applied
sequentially in the configured order.
"""

import inspect

from PySide6 import QtCore, QtWidgets

from ros2_unbag.core.processors import Processor
from ros2_unbag.ui.styles import EMPTY_HINT_STYLE

__all__ = ["ProcessorChainWidget"]


class ProcessorChainWidget(QtWidgets.QWidget):
    """
    Widget to configure an ordered chain of processors for a topic.
    
    Allows users to add, remove, reorder, and configure data processors that will
    be applied sequentially to topic messages before export. Each processor can have
    its own configuration arguments that are dynamically generated based on the
    processor's signature.
    
    The widget provides:
    - Add/remove processor entries
    - Reorder processors with up/down buttons
    - Configure processor-specific arguments with inline help
    - Visual indication when no processors are configured
    """

    def __init__(self, topic_type, available_processors, parent=None):
        """
        Initialize the processor chain widget with topic context and known processors.

        Args:
            topic_type: ROS message type string for which processors are queried.
            available_processors: Iterable of processor identifiers for the topic.
            parent: Optional Qt parent widget.

        Returns:
            None
        """
        super().__init__(parent)
        self.topic_type = topic_type
        self.available_processors = list(available_processors)
        self.entries = []

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self.chain_layout = QtWidgets.QVBoxLayout()
        self.chain_layout.setSpacing(8)
        layout.addLayout(self.chain_layout)

        self.empty_hint = QtWidgets.QLabel("No processors configured.")
        layout.addWidget(self.empty_hint)

        add_row = QtWidgets.QHBoxLayout()
        add_row.addStretch()
        self.add_button = QtWidgets.QPushButton("Add Processor")
        self.add_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.add_button.clicked.connect(self.add_entry)
        add_row.addWidget(self.add_button)
        layout.addLayout(add_row)

        self._update_empty_hint()

    def add_entry(self, preset=None):
        """
        Append a processor entry to the chain and optionally configure it.

        Args:
            preset: Optional dict containing `name` and `args` to pre-populate the entry.

        Returns:
            _ProcessorEntry: The newly created processor entry widget.
        """
        entry = _ProcessorEntry(self)
        self.entries.append(entry)
        self.chain_layout.addWidget(entry)
        self._reindex_entries()
        if preset:
            entry.apply_config(preset)
        else:
            entry.select_default()
        self._update_empty_hint()
        return entry

    def remove_entry(self, entry):
        """
        Remove an existing processor entry from the chain.

        Args:
            entry: The _ProcessorEntry instance to remove.

        Returns:
            None
        """
        if entry in self.entries:
            self.entries.remove(entry)
            self.chain_layout.removeWidget(entry)
            entry.deleteLater()
            self._reindex_entries()
            self._update_empty_hint()

    def move_entry(self, entry, delta):
        """
        Move a processor entry up or down within the chain.

        Args:
            entry: The _ProcessorEntry instance to reposition.
            delta: Signed integer indicating movement (-1 for up, +1 for down).

        Returns:
            None
        """
        if entry not in self.entries:
            return
        idx = self.entries.index(entry)
        new_idx = idx + delta
        if new_idx < 0 or new_idx >= len(self.entries):
            return
        self.entries.pop(idx)
        self.entries.insert(new_idx, entry)
        self.chain_layout.removeWidget(entry)
        self.chain_layout.insertWidget(new_idx, entry)
        self._reindex_entries()

    def get_chain(self):
        """
        Retrieve the processor chain configuration as a list of dicts.

        Args:
            None

        Returns:
            list: Ordered list of processor configurations.
        """
        chain = []
        for entry in self.entries:
            config = entry.get_config()
            if config:
                chain.append(config)
        return chain

    def set_chain(self, configs):
        """
        Replace the current processor chain with the provided configuration.

        Args:
            configs: Iterable of processor configuration dicts or strings.

        Returns:
            None
        """
        self.clear()
        for cfg in configs or []:
            self.add_entry(cfg)
        if not configs:
            self._update_empty_hint()

    def clear(self):
        """
        Remove all processors from the chain.

        Args:
            None

        Returns:
            None
        """
        for entry in list(self.entries):
            self.remove_entry(entry)

    def _reindex_entries(self):
        """
        Update the numbering and button state for chain entries.

        Args:
            None

        Returns:
            None
        """
        total = len(self.entries)
        for idx, entry in enumerate(self.entries, start=1):
            entry.set_index(idx, total)

    def _update_empty_hint(self):
        """
        Toggle the empty-chain placeholder visibility.

        Args:
            None

        Returns:
            None
        """
        self.empty_hint.setVisible(len(self.entries) == 0)


class _ProcessorEntry(QtWidgets.QFrame):
    """Single processor entry row inside the processor chain widget."""

    def __init__(self, chain_widget):
        """
        Prepare a processor entry row and render its controls.

        Args:
            chain_widget: Parent ProcessorChainWidget instance.

        Returns:
            None
        """
        super().__init__()
        self.chain_widget = chain_widget
        self.arg_inputs = {}

        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setFrameShadow(QtWidgets.QFrame.Raised)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(4)

        header = QtWidgets.QHBoxLayout()
        header.setSpacing(6)

        self.index_label = QtWidgets.QLabel("1.")
        self.index_label.setFixedWidth(24)
        header.addWidget(self.index_label)

        self.combo = QtWidgets.QComboBox()
        self.combo.addItems(self.chain_widget.available_processors)
        self.combo.currentTextChanged.connect(self._on_processor_changed)
        header.addWidget(self.combo, stretch=1)

        self.up_button = QtWidgets.QToolButton()
        self.up_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_ArrowUp))
        self.up_button.setAutoRaise(True)
        self.up_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.up_button.clicked.connect(lambda: self.chain_widget.move_entry(self, -1))
        header.addWidget(self.up_button)

        self.down_button = QtWidgets.QToolButton()
        self.down_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_ArrowDown))
        self.down_button.setAutoRaise(True)
        self.down_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.down_button.clicked.connect(lambda: self.chain_widget.move_entry(self, 1))
        header.addWidget(self.down_button)

        self.remove_button = QtWidgets.QToolButton()
        self.remove_button.setIcon(self.style().standardIcon(QtWidgets.QStyle.SP_DialogCloseButton))
        self.remove_button.setAutoRaise(True)
        self.remove_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.remove_button.clicked.connect(lambda: self.chain_widget.remove_entry(self))
        header.addWidget(self.remove_button)

        layout.addLayout(header)

        self.args_layout = QtWidgets.QFormLayout()
        self.args_layout.setContentsMargins(0, 4, 0, 0)
        self.args_layout.setFieldGrowthPolicy(QtWidgets.QFormLayout.ExpandingFieldsGrow)
        layout.addLayout(self.args_layout)

        self._on_processor_changed(self.combo.currentText())

    def set_index(self, index, total):
        """
        Update the displayed index label and navigation button enablement.

        Args:
            index: One-based index of this entry in the chain.
            total: Total number of entries in the chain.

        Returns:
            None
        """
        self.index_label.setText(f"{index}.")
        self.up_button.setEnabled(index > 1)
        self.down_button.setEnabled(index < total)

    def select_default(self):
        """
        Select the default processor for this entry.

        Args:
            None

        Returns:
            None
        """
        if self.chain_widget.available_processors:
            self.combo.setCurrentIndex(0)

    def apply_config(self, config):
        """
        Populate the entry with a stored processor configuration.

        Args:
            config: Processor configuration dict containing `name` and optional `args`.

        Returns:
            None
        """
        name = config.get("name")
        if name and name in self.chain_widget.available_processors:
            self.combo.setCurrentText(name)
        elif name:
            self.combo.addItem(name)
            self.combo.setCurrentText(name)

        args = config.get("args", {}) or {}
        self._on_processor_changed(self.combo.currentText())
        for arg_name, value in args.items():
            if arg_name in self.arg_inputs:
                self.arg_inputs[arg_name].setText(str(value))

    def get_config(self):
        """
        Return the processor configuration represented by this entry.

        Args:
            None

        Returns:
            dict: Processor configuration with `name` and optional `args`.
        """
        name = self.combo.currentText()
        args = {}
        for arg_name, edit in self.arg_inputs.items():
            value = edit.text().strip()
            if value:
                args[arg_name] = value
        return {"name": name, "args": args}

    def _clear_args(self):
        """
        Remove all argument widgets from the layout.

        Args:
            None

        Returns:
            None
        """
        while self.args_layout.rowCount():
            self.args_layout.removeRow(0)
        self.arg_inputs = {}

    def _on_processor_changed(self, processor_name):
        """
        Rebuild argument inputs when the selected processor changes.

        Args:
            processor_name: Identifier of the newly selected processor.

        Returns:
            None
        """
        self._clear_args()
        args = Processor.get_args(self.chain_widget.topic_type, processor_name)
        if not args:
            return

        for arg_name, (param, doc) in args.items():
            label = QtWidgets.QLabel()
            label.setText(f"{arg_name} (optional)" if param.default != inspect.Parameter.empty else arg_name)

            parts = []
            if doc:
                parts.append(doc)
            if param.default != inspect.Parameter.empty:
                parts.append(f"default: {param.default}")
            if param.annotation != inspect.Parameter.empty:
                annotation = getattr(param.annotation, "__name__", str(param.annotation))
                parts.append(f"Type: {annotation}")
            placeholder_text = " — ".join(parts)

            arg_edit = QtWidgets.QLineEdit()
            arg_edit.setPlaceholderText(placeholder_text)

            self.args_layout.addRow(label, arg_edit)
            self.arg_inputs[arg_name] = arg_edit
