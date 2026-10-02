from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

from drone_cslam.gt_recorder_node import GazeboGroundTruthRecorder


def test_missing_entity_without_status_message_retries():
    fake = SimpleNamespace(_pending={"drone1": True}, _record_failure=Mock())
    future = Future()
    future.set_result(SimpleNamespace(success=False))
    GazeboGroundTruthRecorder._handle_response(fake, "drone1", future)
    assert fake._pending["drone1"] is False
    fake._record_failure.assert_called_once()
