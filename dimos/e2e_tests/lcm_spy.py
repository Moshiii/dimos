# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections.abc import Callable, Iterator
from contextlib import contextmanager
import math
import pickle
import threading
from typing import Any, cast

from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.e2e_tests.scene_contract import PlanarBounds
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.helpers import resolve_msg_type
from dimos.msgs.protocol import DimosMsg
from dimos.utils.testing.waiting import wait_until


class LcmSpy:
    messages: dict[str, list[bytes]]
    _messages_lock: threading.Lock
    _saved_topics: set[str]
    _saved_topics_lock: threading.Lock
    _topic_listeners: dict[str, list[Callable[[bytes], None]]]
    _topic_listeners_lock: threading.Lock

    def __init__(self) -> None:
        self.messages = {}
        self._messages_lock = threading.Lock()
        self._saved_topics = set()
        self._saved_topics_lock = threading.Lock()
        self._topic_listeners = {}
        self._topic_listeners_lock = threading.Lock()
        self._transports: dict[str, PubSubTransport[Any]] = {}
        self._unsubscribers: dict[str, Callable[[], None]] = {}
        self._publishers: dict[str, PubSubTransport[Any]] = {}
        self._transports_lock = threading.Lock()

    def start(self) -> None:
        pass

    def stop(self) -> None:
        with self._transports_lock:
            unsubscribers = tuple(self._unsubscribers.values())
            transports = tuple(self._transports.values())
            publishers = tuple(self._publishers.values())
            self._unsubscribers.clear()
            self._transports.clear()
            self._publishers.clear()
        for unsubscribe in unsubscribers:
            unsubscribe()
        for transport in (*transports, *publishers):
            transport.stop()

    def msg(self, topic: str, data: bytes) -> None:
        with self._saved_topics_lock:
            if topic in self._saved_topics:
                with self._messages_lock:
                    self.messages.setdefault(topic, []).append(data)

        with self._topic_listeners_lock:
            listeners = self._topic_listeners.get(topic)
            if listeners:
                for listener in listeners:
                    listener(data)

    def publish(self, topic: str, msg: Any) -> None:
        with self._transports_lock:
            transport = self._publishers.get(topic)
            if transport is None:
                name, msg_type = _parse_topic(topic, type(msg))
                transport = make_transport(name, msg_type)
                self._publishers[topic] = transport
        transport.broadcast(None, msg)

    def save_topic(self, topic: str) -> None:
        with self._saved_topics_lock:
            self._saved_topics.add(topic)
        self._ensure_subscription(topic)

    def register_topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> None:
        with self._topic_listeners_lock:
            self._topic_listeners.setdefault(topic, []).append(listener)
        self._ensure_subscription(topic)

    def unregister_topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> None:
        with self._topic_listeners_lock:
            self._topic_listeners[topic].remove(listener)

    @contextmanager
    def topic_listener(self, topic: str, listener: Callable[[bytes], None]) -> Iterator[None]:
        self.register_topic_listener(topic, listener)
        try:
            yield
        finally:
            self.unregister_topic_listener(topic, listener)

    def wait_for_saved_topic(self, topic: str, timeout: float = 30.0) -> None:
        def condition() -> bool:
            with self._messages_lock:
                return topic in self.messages

        wait_until(
            condition,
            timeout=timeout,
            message=f"Timeout waiting for topic {topic}",
        )

    def wait_for_saved_topic_content(
        self, topic: str, content_contains: bytes, timeout: float = 30.0
    ) -> None:
        def condition() -> bool:
            with self._messages_lock:
                return any(content_contains in msg for msg in self.messages.get(topic, []))

        wait_until(
            condition,
            timeout=timeout,
            message=f"Timeout waiting for '{topic}' to contain '{content_contains!r}'",
        )

    def wait_for_message_pickle_result(
        self,
        topic: str,
        predicate: Callable[[Any], bool],
        fail_message: str,
        timeout: float = 30.0,
    ) -> None:
        event = threading.Event()

        def listener(msg: bytes) -> None:
            data = pickle.loads(msg)
            if predicate(data["res"]):
                event.set()

        with self.topic_listener(topic, listener):
            wait_until(
                event.is_set,
                timeout=timeout,
                message=fail_message,
            )

    def wait_for_message_result(
        self,
        topic: str,
        type: type[DimosMsg],
        predicate: Callable[[Any], bool],
        fail_message: str,
        timeout: float = 30.0,
    ) -> None:
        event = threading.Event()

        def listener(msg: bytes) -> None:
            data = type.lcm_decode(msg)
            if predicate(data):
                event.set()

        with self.topic_listener(topic, listener):
            wait_until(
                event.is_set,
                timeout=timeout,
                message=fail_message,
            )

    def wait_for_saved_message_result(
        self,
        topic: str,
        type: type[DimosMsg],
        predicate: Callable[[Any], bool],
        fail_message: str,
        timeout: float = 30.0,
    ) -> None:
        """Wait for a matching message saved since ``save_topic`` was called."""

        def condition() -> bool:
            with self._messages_lock:
                messages = tuple(self.messages.get(topic, ()))
            return any(predicate(type.lcm_decode(message)) for message in messages)

        wait_until(condition, timeout=timeout, message=fail_message)

    def wait_until_odom_position(
        self, x: float, y: float, threshold: float = 1, timeout: float = 60
    ) -> None:
        def predicate(msg: PoseStamped) -> bool:
            pos = msg.position
            distance = math.sqrt((pos.x - x) ** 2 + (pos.y - y) ** 2)
            return distance < threshold

        self.wait_for_message_result(
            "/odom#geometry_msgs.PoseStamped",
            PoseStamped,
            predicate,
            f"Failed to get to position x={x}, y={y}",
            timeout,
        )

    def wait_until_odom_near_bounds(
        self,
        bounds: PlanarBounds,
        max_distance: float,
        timeout: float = 60.0,
    ) -> None:
        def predicate(msg: PoseStamped) -> bool:
            return bounds.distance_to(msg.position.x, msg.position.y) <= max_distance

        self.wait_for_message_result(
            "/odom#geometry_msgs.PoseStamped",
            PoseStamped,
            predicate,
            f"Robot did not get within {max_distance} m of semantic target bounds {bounds}",
            timeout,
        )

    def _ensure_subscription(self, topic: str) -> None:
        with self._transports_lock:
            if topic in self._transports:
                return
            name, msg_type = _parse_topic(topic)
            transport = make_transport(name, msg_type)
            unsubscribe = transport.subscribe(lambda msg: self.msg(topic, _encode_message(msg)))
            self._transports[topic] = transport
            self._unsubscribers[topic] = unsubscribe


def _parse_topic(topic: str, default_type: type[Any] | None = None) -> tuple[str, type[Any] | None]:
    if "#" not in topic:
        return topic, default_type if hasattr(default_type, "lcm_encode") else None
    name, type_name = topic.rsplit("#", 1)
    msg_type = resolve_msg_type(type_name)
    if msg_type is None:
        raise ValueError(f"Unknown message type {type_name!r} in topic {topic!r}")
    return name, msg_type


def _encode_message(message: Any) -> bytes:
    if hasattr(message, "lcm_encode"):
        return cast("bytes", message.lcm_encode())
    return pickle.dumps(message)
