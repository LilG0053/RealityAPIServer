import json
from typing import Any


def build_subscription(topic: str, subscription_id: str = "realityapi-position") -> dict[str, Any]:
    """Build a rosbridge subscription for a topic."""
    return {
        "op": "subscribe",
        "id": subscription_id,
        "topic": topic,
        "queue_length": 1,
        "throttle_rate": 0,
    }


def decode_publish_message(raw: str) -> dict[str, Any] | None:
    """Decode a rosbridge text frame and return publish data."""
    try:
        message = json.loads(raw)
    except json.JSONDecodeError:
        return None

    if not isinstance(message, dict) or message.get("op") != "publish":
        return None
    return message
