#!/usr/bin/env python3

import ast
import json
import os
from collections import deque
from typing import Any, Deque, Dict, Iterable, List, Tuple

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped


class AlignedTopicLogger:
    def __init__(self) -> None:
        rospy.init_node("aligned_topic_logger", anonymous=True)

        self.topic_types = {
            "/smart/cmd_vel": Twist,
            "/smart/rear_pose": PoseStamped,
            "/smart/center_pose": PoseStamped,
            "/smart/velocity": TwistStamped,
        }

        self.selected_topics = self._normalize_topics(
            rospy.get_param("~topics", list(self.topic_types.keys()))
        )
        self.log_rate = float(rospy.get_param("~log_rate", 10.0))
        self.slop = float(rospy.get_param("~slop", 0.05))
        self.max_age = float(rospy.get_param("~max_age", 5.0))
        self.buffer_size = int(rospy.get_param("~buffer_size", 200))

        self.buffers: Dict[str, Deque[Tuple[rospy.Time, Any]]] = {
            topic: deque(maxlen=self.buffer_size) for topic in self.selected_topics
        }
        self.subscribers = [
            rospy.Subscriber(topic, self.topic_types[topic], self._make_callback(topic), queue_size=10)
            for topic in self.selected_topics
        ]

        self.output_file = self._resolve_output_file(rospy.get_param("~output_file", ""))
        self._log_handle = open(self.output_file, "a", buffering=1)
        rospy.loginfo("Logging topics %s to %s", self.selected_topics, self.output_file)

        period = 1.0 / self.log_rate if self.log_rate > 0 else 0.1
        self._timer = rospy.Timer(rospy.Duration(period), self._on_timer)
        rospy.on_shutdown(self._close)

    def _normalize_topics(self, topics_param: Any) -> List[str]:
        topics_raw: Iterable[str]
        if isinstance(topics_param, str):
            try:
                topics_raw = ast.literal_eval(topics_param)
            except Exception:
                topics_raw = [part.strip() for part in topics_param.split(",") if part.strip()]
        else:
            topics_raw = topics_param

        if not isinstance(topics_raw, (list, tuple)):
            rospy.logwarn("~topics param is not a list; using defaults.")
            return list(self.topic_types.keys())

        filtered = [topic for topic in topics_raw if topic in self.topic_types]
        if not filtered:
            rospy.logwarn("No valid topics found in ~topics=%s, falling back to defaults.", topics_param)
            filtered = list(self.topic_types.keys())
        return filtered

    def _resolve_output_file(self, output_param: str) -> str:
        # 默认写到工作空间根目录下的 logs/
        workspace_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        default_dir = os.path.join(workspace_root, "logs")
        os.makedirs(default_dir, exist_ok=True)

        if not output_param:
            timestamp = int(rospy.Time.now().to_sec())
            return os.path.join(default_dir, f"aligned_topics_{timestamp}.jsonl")

        # 如果传入的是目录，自动补一个文件名
        if output_param.endswith(os.path.sep):
            output_param = os.path.join(output_param, "aligned_topics.jsonl")

        if not os.path.isabs(output_param):
            output_param = os.path.join(default_dir, output_param)

        output_dir = os.path.dirname(output_param)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)
        return output_param

    def _make_callback(self, topic: str):
        def _cb(msg: Any) -> None:
            stamp = self._extract_stamp(msg)
            self.buffers[topic].append((stamp, msg))
        return _cb

    def _extract_stamp(self, msg: Any) -> rospy.Time:
        header = getattr(msg, "header", None)
        if header and hasattr(header, "stamp") and header.stamp != rospy.Time():
            return header.stamp
        return rospy.Time.now()

    def _trim_old(self, buffer: deque, reference: rospy.Time) -> None:
        while buffer and (reference - buffer[0][0]).to_sec() > self.max_age:
            buffer.popleft()

    def _get_latest_within(self, topic: str, reference: rospy.Time) -> Tuple[Any, Any]:
        buffer = self.buffers.get(topic)
        if not buffer:
            return None, None

        self._trim_old(buffer, reference)
        for stamp, msg in reversed(buffer):
            if abs((reference - stamp).to_sec()) <= self.slop:
                return stamp, msg
        return None, None

    def _ros_to_dict(self, value: Any) -> Any:
        if isinstance(value, rospy.Time):
            return value.to_sec()
        if isinstance(value, rospy.Duration):
            return value.to_sec()
        if isinstance(value, (list, tuple)):
            return [self._ros_to_dict(item) for item in value]
        if hasattr(value, "__slots__"):
            return {slot: self._ros_to_dict(getattr(value, slot)) for slot in value.__slots__}
        return value

    def _on_timer(self, _event) -> None:
        now = rospy.Time.now()
        record: Dict[str, Any] = {"log_time": now.to_sec()}

        for topic in self.selected_topics:
            stamp, msg = self._get_latest_within(topic, now)
            if msg is None:
                record[topic] = None
            else:
                record[topic] = {"stamp": stamp.to_sec(), "data": self._ros_to_dict(msg)}

        try:
            self._log_handle.write(json.dumps(record, sort_keys=True) + "\n")
        except Exception as exc:
            rospy.logwarn("Failed to write log line: %s", exc)

    def _close(self) -> None:
        try:
            if self._log_handle:
                self._log_handle.flush()
                self._log_handle.close()
        except Exception:
            pass

    def spin(self) -> None:
        rospy.spin()


if __name__ == "__main__":
    logger = AlignedTopicLogger()
    logger.spin()
