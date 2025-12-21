import pytest

from ros2_unbag.core.exporter import Exporter
from ros2_unbag.core.processors.base import Processor
from ros2_unbag.core.routines.base import ExportMode, ExportRoutine


@ExportRoutine("custom_msgs/msg/Test", ["test_fmt"], ExportMode.MULTI_FILE)
def dummy_routine(msg, path, fmt, metadata):
    return msg


@Processor("custom_msgs/msg/Test", ["proc_a"])
def proc_a(msg):
    msg["chain"].append("a")
    return msg


@Processor("custom_msgs/msg/Test", ["proc_b"])
def proc_b(msg, multiplier):
    msg["chain"].append(f"b:{multiplier}")
    return msg


class DummyBagReader:
    def __init__(self):
        self.topic_types = {"/test": "custom_msgs/msg/Test"}


def build_exporter(topic_config):
    bag_reader = DummyBagReader()
    global_config = {"cpu_percentage": 100}
    return Exporter(bag_reader, {"/test": topic_config}, global_config)


def test_exporter_builds_ordered_processor_chain():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processors": [
            {"name": "proc_a"},
            {"name": "proc_b", "args": {"multiplier": "2"}},
        ],
    }
    exporter = build_exporter(topic_cfg)

    chain = exporter.topic_processors["/test"]
    assert len(chain) == 2

    names = []
    for handler, args in chain:
        names.append(handler.__name__)
    assert names == ["proc_a", "proc_b"]

    canonical = topic_cfg["processors"]
    assert canonical == [
        {"name": "proc_a", "args": {}},
        {"name": "proc_b", "args": {"multiplier": "2"}},
    ]
    assert "processor" not in topic_cfg
    assert "processor_args" not in topic_cfg


def test_exporter_supports_legacy_processor_fields():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processor": "proc_b",
        "processor_args": {"multiplier": "4"},
    }
    exporter = build_exporter(topic_cfg)
    chain = exporter.topic_processors["/test"]
    assert len(chain) == 1
    handler, args = chain[0]
    assert handler.__name__ == "proc_b"
    assert args == {"multiplier": "4"}


def test_exporter_applies_processor_chain_ordered():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processors": [
            {"name": "proc_a"},
            {"name": "proc_b", "args": {"multiplier": "3"}},
        ],
    }
    exporter = build_exporter(topic_cfg)
    chain = exporter.topic_processors["/test"]
    message = {"chain": []}
    for handler, args in chain:
        message = handler(msg=message, **args)
    assert message["chain"] == ["a", "b:3"]


def test_exporter_raises_for_missing_required_args():
    topic_cfg = {
        "format": "test_fmt",
        "path": ".",
        "subfolder": "",
        "naming": "%name_%index",
        "processors": [
            {"name": "proc_b"},
        ],
    }
    with pytest.raises(ValueError, match="Missing required arguments"):
        build_exporter(topic_cfg)
