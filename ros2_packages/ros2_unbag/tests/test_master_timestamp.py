
import pytest
from unittest.mock import MagicMock
from ros2_unbag.core.exporter import Exporter
from ros2_unbag.core.routines.base import ExportRoutine, ExportMode

# Mock message classes to avoid importing real ROS messages if they are missing
class MockStamp:
    def __init__(self, sec, nanosec):
        self.sec = sec
        self.nanosec = nanosec

class MockHeader:
    def __init__(self, sec, nanosec):
        self.stamp = MockStamp(sec, nanosec)

class MockMsg:
    def __init__(self, sec, nanosec):
        self.header = MockHeader(sec, nanosec)

# Register a dummy export routine for testing
# We use a unique format name to avoid collisions
@ExportRoutine("custom_msgs/msg/Dummy", ["test_master_ts_fmt"], ExportMode.MULTI_FILE)
def dummy_handler(msg, path, fmt, metadata, topic=None):
    pass

class MockBagReader:
    def __init__(self):
        # We pretend these topics exist
        self.topic_types = {
            "/master": "custom_msgs/msg/Dummy",
            "/slave": "custom_msgs/msg/Dummy"
        }
    
    def get_message_count(self):
        return {"/master": 10, "/slave": 10}
        
    def set_filter(self, topics):
        pass

def test_master_timestamp_substitution():
    # Create configuration
    export_config = {
        "/master": {
            "format": "test_master_ts_fmt",
            "naming": "master_%timestamp",
            "path": "/tmp",
            "subfolder": "",
        },
        "/slave": {
            "format": "test_master_ts_fmt",
            "naming": "slave_%master_timestamp",
            "path": "/tmp",
            "subfolder": "",
        }
    }
    
    global_config = {
        "cpu_percentage": 50,
        "resample_config": {
            "master_topic": "/master",
            "association": "last",
            "discard_eps": 0.1
        }
    }
    
    bag_reader = MockBagReader()
    exporter = Exporter(bag_reader, export_config, global_config)
    
    # Initialize state normally set in run()
    exporter.max_index = {"/master": 9, "/slave": 9}
    exporter.index_length = {"/master": 1, "/slave": 1}
    
    # Mock queues to intercept
    exporter.parallel_q = MagicMock()
    
    # 1. Test enqueue with master_ts provided
    # Master TS: 100s = 100 * 1e9 ns
    master_ts_ns = 100 * 1_000_000_000
    
    # Slave Msg: 100.05s
    slave_msg = MockMsg(100, 50_000_000)
    
    # Manually call _enqueue_export_task
    exporter._enqueue_export_task("/slave", slave_msg, master_ts=master_ts_ns)
    
    # Verify call to queue
    args, _ = exporter.parallel_q.put.call_args
    task = args[0]
    # Task structure: (topic, msg, full_path, fmt, metadata)
    full_path = str(task[2])
    
    # The naming for slave is "slave_%master_timestamp"
    # So we expect "slave_100000000000"
    expected_substr = f"slave_{master_ts_ns}"
    
    assert expected_substr in full_path, f"Expected '{expected_substr}' in '{full_path}'"

def test_master_timestamp_fallback():
    # If master_ts is NOT provided (e.g. no resampling or master topic itself), 
    # it should fallback to own timestamp (or at least not crash).
    # Current implementation: if master_ts is None, use own timestamp.
    
    export_config = {
        "/slave": {
            "format": "test_master_ts_fmt",
            "naming": "slave_%master_timestamp",
            "path": "/tmp",
            "subfolder": "",
        }
    }
    global_config = {"cpu_percentage": 50}
    
    bag_reader = MockBagReader()
    # We cheat and add /slave to topic types since MockBagReader has it hardcoded
    exporter = Exporter(bag_reader, export_config, global_config)
    
    exporter.max_index = {"/slave": 9}
    exporter.index_length = {"/slave": 1}

    exporter.parallel_q = MagicMock()
    
    slave_msg = MockMsg(200, 0) # 200s
    
    exporter._enqueue_export_task("/slave", slave_msg, master_ts=None)
    
    args, _ = exporter.parallel_q.put.call_args
    task = args[0]
    full_path = str(task[2])
    
    # Expect fall back to own timestamp: 200000000000
    expected_substr = "slave_200000000000"
    assert expected_substr in full_path, f"Expected '{expected_substr}' in '{full_path}'"
