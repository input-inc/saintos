"""
Tests for SAINT.OS Control Flow

Tests the full control message flow through WebSocket handler,
simulating what happens when the web UI sends control commands.
"""

import asyncio
import json
import pytest
import sys
import os

# Add the parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from saint_server.webserver.state_manager import StateManager
from saint_server.webserver.websocket_handler import WebSocketHandler


def announce(data: dict) -> str:
    """Convert announcement dict to JSON string."""
    return json.dumps(data)


def run_async(coro):
    """Helper to run async functions in tests."""
    return asyncio.get_event_loop().run_until_complete(coro)


class MockWebSocket:
    """Mock WebSocket for testing."""

    def __init__(self):
        self.messages = []
        self.closed = False

    async def send_json(self, message):
        self.messages.append(message)

    async def close(self):
        self.closed = True


class MockClient:
    """Mock WebSocket client for testing."""

    def __init__(self, client_id="test_client"):
        self.id = client_id
        self.ws = MockWebSocket()
        self.subscriptions = set()


class TestControlHandler:
    """Test the control message handler."""

    def setup_method(self):
        """Setup test fixtures."""
        self.state_manager = StateManager(server_name="test-server")
        self.ws_handler = WebSocketHandler(self.state_manager)

        # Setup a test node with pin configuration
        self.state_manager.update_node_from_announcement(announce({
            "node_id": "test_node",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))
        self.state_manager.adopt_node("test_node", "tracks", "Test Node")
        self.state_manager.save_node_pin_config("test_node", {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
            "6": {"mode": "pwm", "logical_name": "right_motor"},
        })

        self.client = MockClient()

    def test_set_pin_value_updates_state(self):
        """Test that set_pin_value updates the state manager."""
        async def _test():
            result = await self.ws_handler._handle_control(
                self.client,
                "set_pin_value",
                {"node_id": "test_node", "gpio": 5, "value": 75.0}
            )

            assert result["status"] == "ok"

            # Verify state was updated
            state = self.state_manager.get_runtime_state("test_node")
            pin5 = next((p for p in state["pins"] if p["gpio"] == 5), None)
            assert pin5 is not None
            assert pin5["desired"] == 75.0

        run_async(_test())

    def test_get_runtime_state_returns_desired(self):
        """Test that get_runtime_state returns desired values set by control."""
        async def _test():
            # First, set a value via control
            await self.ws_handler._handle_control(
                self.client,
                "set_pin_value",
                {"node_id": "test_node", "gpio": 5, "value": 42.0}
            )

            # Then get runtime state
            result = await self.ws_handler._handle_control(
                self.client,
                "get_runtime_state",
                {"node_id": "test_node"}
            )

            assert result["status"] == "ok"
            assert result["data"] is not None

            pin5 = next((p for p in result["data"]["pins"] if p["gpio"] == 5), None)
            assert pin5 is not None
            assert pin5["desired"] == 42.0

        run_async(_test())

    def test_multiple_set_values_persist(self):
        """Test that multiple set_pin_value calls all persist."""
        async def _test():
            # Set multiple values
            await self.ws_handler._handle_control(
                self.client,
                "set_pin_value",
                {"node_id": "test_node", "gpio": 5, "value": 25.0}
            )

            # Wait to avoid throttle
            await asyncio.sleep(0.1)

            await self.ws_handler._handle_control(
                self.client,
                "set_pin_value",
                {"node_id": "test_node", "gpio": 6, "value": 75.0}
            )

            # Get state
            result = await self.ws_handler._handle_control(
                self.client,
                "get_runtime_state",
                {"node_id": "test_node"}
            )

            pin5 = next((p for p in result["data"]["pins"] if p["gpio"] == 5), None)
            pin6 = next((p for p in result["data"]["pins"] if p["gpio"] == 6), None)

            assert pin5["desired"] == 25.0
            assert pin6["desired"] == 75.0

        run_async(_test())

    def test_control_missing_params(self):
        """Test control with missing parameters."""
        async def _test():
            result = await self.ws_handler._handle_control(
                self.client,
                "set_pin_value",
                {"node_id": "test_node"}  # Missing gpio and value
            )

            assert result["status"] == "error"

        run_async(_test())


class TestBroadcastFlow:
    """Test state broadcast functionality."""

    def setup_method(self):
        """Setup test fixtures."""
        self.state_manager = StateManager(server_name="test-server")
        self.ws_handler = WebSocketHandler(self.state_manager)

        # Setup test node
        self.state_manager.update_node_from_announcement(announce({
            "node_id": "test_node",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))
        self.state_manager.adopt_node("test_node", "tracks", "Test Node")
        self.state_manager.save_node_pin_config("test_node", {
            "5": {"mode": "pwm", "logical_name": "motor"},
        })

    def test_broadcast_to_subscribers(self):
        """Test that state is broadcast to subscribed clients."""
        async def _test():
            # Create two mock clients
            client1 = MockClient("client1")
            client1.subscriptions.add("pin_state/test_node")

            client2 = MockClient("client2")
            client2.subscriptions.add("pin_state/test_node")

            client3 = MockClient("client3")  # Not subscribed

            # Add clients to handler
            self.ws_handler.clients = {
                "client1": client1,
                "client2": client2,
                "client3": client3,
            }

            # Broadcast state
            await self.ws_handler._broadcast_pin_state("test_node")

            # Check that subscribed clients received the message
            assert len(client1.ws.messages) == 1
            assert len(client2.ws.messages) == 1
            assert len(client3.ws.messages) == 0  # Not subscribed

            # Verify message content
            msg = client1.ws.messages[0]
            assert msg["type"] == "state"
            assert msg["node"] == "pin_state/test_node"
            assert "data" in msg

        run_async(_test())

    def test_control_triggers_broadcast(self):
        """Test that set_pin_value triggers a broadcast."""
        async def _test():
            # Create a subscribed client
            subscribed_client = MockClient("subscriber")
            subscribed_client.subscriptions.add("pin_state/test_node")

            self.ws_handler.clients = {"subscriber": subscribed_client}

            # Send control command from another client
            controller = MockClient("controller")

            await self.ws_handler._handle_control(
                controller,
                "set_pin_value",
                {"node_id": "test_node", "gpio": 5, "value": 60.0}
            )

            # Check that the subscribed client received a broadcast
            assert len(subscribed_client.ws.messages) >= 1

            # Find the state message
            state_msgs = [m for m in subscribed_client.ws.messages if m.get("type") == "state"]
            assert len(state_msgs) >= 1

            # Verify the broadcast contains the updated value
            msg = state_msgs[-1]
            assert msg["node"] == "pin_state/test_node"
            pin5 = next((p for p in msg["data"]["pins"] if p["gpio"] == 5), None)
            assert pin5["desired"] == 60.0

        run_async(_test())


class TestEndToEndSimulation:
    """End-to-end simulation of the user's workflow."""

    def setup_method(self):
        """Setup test fixtures."""
        self.state_manager = StateManager(server_name="test-server")
        self.ws_handler = WebSocketHandler(self.state_manager)

        # Setup test node
        self.state_manager.update_node_from_announcement(announce({
            "node_id": "robot_001",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))
        self.state_manager.adopt_node("robot_001", "tracks", "Robot")
        self.state_manager.save_node_pin_config("robot_001", {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
            "6": {"mode": "pwm", "logical_name": "right_motor"},
        })

    def test_control_page_then_node_detail(self):
        """
        Simulate: User adjusts control in Control tab,
        then navigates to Node detail page.
        The Node detail page should see the same values.
        """
        async def _test():
            # Client 1: Control page - set values
            control_client = MockClient("control_page")

            result1 = await self.ws_handler._handle_control(
                control_client,
                "set_pin_value",
                {"node_id": "robot_001", "gpio": 5, "value": 80.0}
            )
            assert result1["status"] == "ok"

            await asyncio.sleep(0.1)  # Avoid throttle

            result2 = await self.ws_handler._handle_control(
                control_client,
                "set_pin_value",
                {"node_id": "robot_001", "gpio": 6, "value": 45.0}
            )
            assert result2["status"] == "ok"

            # Client 2: Node detail page - request runtime state
            detail_client = MockClient("node_detail_page")

            result3 = await self.ws_handler._handle_control(
                detail_client,
                "get_runtime_state",
                {"node_id": "robot_001"}
            )

            assert result3["status"] == "ok"
            assert result3["data"] is not None

            # Verify the Node detail page sees the values set by Control page
            pins = result3["data"]["pins"]
            pin5 = next((p for p in pins if p["gpio"] == 5), None)
            pin6 = next((p for p in pins if p["gpio"] == 6), None)

            assert pin5 is not None, "Pin 5 should exist in runtime state"
            assert pin6 is not None, "Pin 6 should exist in runtime state"

            assert pin5["desired"] == 80.0, f"Expected 80.0, got {pin5.get('desired')}"
            assert pin6["desired"] == 45.0, f"Expected 45.0, got {pin6.get('desired')}"

        run_async(_test())

    def test_value_updates_reflected_in_subsequent_requests(self):
        """Test that value changes are immediately visible to other clients."""
        async def _test():
            client = MockClient("test_client")

            # Set initial value
            await self.ws_handler._handle_control(
                client,
                "set_pin_value",
                {"node_id": "robot_001", "gpio": 5, "value": 10.0}
            )

            # Get state - should be 10
            result1 = await self.ws_handler._handle_control(
                client,
                "get_runtime_state",
                {"node_id": "robot_001"}
            )
            pin5_v1 = next((p for p in result1["data"]["pins"] if p["gpio"] == 5), None)
            assert pin5_v1["desired"] == 10.0

            await asyncio.sleep(0.1)

            # Update value
            await self.ws_handler._handle_control(
                client,
                "set_pin_value",
                {"node_id": "robot_001", "gpio": 5, "value": 90.0}
            )

            # Get state again - should be 90
            result2 = await self.ws_handler._handle_control(
                client,
                "get_runtime_state",
                {"node_id": "robot_001"}
            )
            pin5_v2 = next((p for p in result2["data"]["pins"] if p["gpio"] == 5), None)
            assert pin5_v2["desired"] == 90.0

        run_async(_test())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
