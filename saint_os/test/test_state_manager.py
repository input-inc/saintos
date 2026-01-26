"""
Tests for SAINT.OS State Manager

Tests for runtime state management, pin control state synchronization,
and state broadcasting.
"""

import json
import pytest
import time
import sys
import os

# Add the parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from saint_server.webserver.state_manager import (
    StateManager,
    NodeInfo,
    NodePinConfig,
    PinConfiguration,
    NodeRuntimeState,
    PinRuntimeState,
)


def announce(data: dict) -> str:
    """Convert announcement dict to JSON string."""
    return json.dumps(data)


class TestStateManagerBasics:
    """Test basic state manager functionality."""

    def test_create_state_manager(self):
        """Test that StateManager can be created."""
        sm = StateManager(server_name="test-server")
        assert sm is not None
        assert sm.state.server_online is True

    def test_adopt_node(self):
        """Test node adoption."""
        sm = StateManager(server_name="test-server")

        # First, simulate a node announcement to create unadopted node
        sm.update_node_from_announcement(announce({
            "node_id": "test_node_001",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))

        # Check node is unadopted
        assert "test_node_001" in sm.state.unadopted_nodes

        # Adopt the node
        result = sm.adopt_node("test_node_001", "tracks", "Test Node")
        assert result['success'] is True

        # Check node is now adopted
        assert "test_node_001" in sm.state.adopted_nodes
        assert "test_node_001" not in sm.state.unadopted_nodes

        node = sm.state.adopted_nodes["test_node_001"]
        assert node.role == "tracks"
        assert node.display_name == "Test Node"


class TestPinConfiguration:
    """Test pin configuration functionality."""

    def setup_method(self):
        """Setup test fixtures."""
        self.sm = StateManager(server_name="test-server")

        # Create and adopt a test node
        self.sm.update_node_from_announcement(announce({
            "node_id": "test_node_001",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))
        self.sm.adopt_node("test_node_001", "tracks", "Test Node")

    def test_save_pin_config(self):
        """Test saving pin configuration."""
        pin_configs = {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
            "6": {"mode": "pwm", "logical_name": "right_motor"},
            "26": {"mode": "adc", "logical_name": "battery_voltage"},
        }

        result = self.sm.save_node_pin_config("test_node_001", pin_configs)
        assert result['success'] is True

        # Verify configuration was saved
        node = self.sm.state.adopted_nodes["test_node_001"]
        assert node.pin_config is not None
        assert 5 in node.pin_config.pins
        assert node.pin_config.pins[5].mode == "pwm"
        assert node.pin_config.pins[5].logical_name == "left_motor"

    def test_get_pin_config(self):
        """Test retrieving pin configuration."""
        # Save config first
        pin_configs = {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
        }
        self.sm.save_node_pin_config("test_node_001", pin_configs)

        # Retrieve config
        config = self.sm.get_node_pin_config("test_node_001")
        assert config is not None
        assert "pins" in config
        assert "5" in config["pins"]
        assert config["pins"]["5"]["mode"] == "pwm"


class TestRuntimeState:
    """Test runtime state management."""

    def setup_method(self):
        """Setup test fixtures."""
        self.sm = StateManager(server_name="test-server")

        # Create and adopt a test node with pin config
        self.sm.update_node_from_announcement(announce({
            "node_id": "test_node_001",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))
        self.sm.adopt_node("test_node_001", "tracks", "Test Node")

        # Add pin configuration
        pin_configs = {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
            "6": {"mode": "pwm", "logical_name": "right_motor"},
            "7": {"mode": "digital_out", "logical_name": "led"},
            "26": {"mode": "adc", "logical_name": "battery_voltage"},
        }
        self.sm.save_node_pin_config("test_node_001", pin_configs)

    def test_get_or_create_runtime_state(self):
        """Test that get_or_create_runtime_state creates state from pin config."""
        runtime_state = self.sm.get_or_create_runtime_state("test_node_001")

        assert runtime_state is not None
        assert runtime_state.node_id == "test_node_001"
        # Should have pins from pin_config
        assert 5 in runtime_state.pins
        assert 6 in runtime_state.pins
        assert runtime_state.pins[5].mode == "pwm"
        assert runtime_state.pins[5].logical_name == "left_motor"

    def test_get_runtime_state_creates_if_missing(self):
        """Test that get_runtime_state creates state if it doesn't exist."""
        # Ensure no runtime state exists
        node = self.sm.state.adopted_nodes["test_node_001"]
        node.runtime_state = None

        # get_runtime_state should create it
        state_dict = self.sm.get_runtime_state("test_node_001")

        assert state_dict is not None
        assert state_dict["node_id"] == "test_node_001"
        assert len(state_dict["pins"]) > 0

    def test_update_pin_desired(self):
        """Test updating desired pin value."""
        # Update desired value
        result = self.sm.update_pin_desired("test_node_001", 5, 75.0)
        assert result is True

        # Verify the value was stored
        runtime_state = self.sm.get_or_create_runtime_state("test_node_001")
        assert runtime_state.pins[5].desired_value == 75.0

    def test_get_runtime_state_includes_desired(self):
        """Test that get_runtime_state returns desired values."""
        # Set a desired value
        self.sm.update_pin_desired("test_node_001", 5, 50.0)
        self.sm.update_pin_desired("test_node_001", 6, 75.0)

        # Get runtime state as dict
        state_dict = self.sm.get_runtime_state("test_node_001")

        assert state_dict is not None

        # Find pin 5 in the pins list
        pin5 = next((p for p in state_dict["pins"] if p["gpio"] == 5), None)
        assert pin5 is not None
        assert pin5["desired"] == 50.0

        # Find pin 6 in the pins list
        pin6 = next((p for p in state_dict["pins"] if p["gpio"] == 6), None)
        assert pin6 is not None
        assert pin6["desired"] == 75.0

    def test_desired_value_persists_across_calls(self):
        """Test that desired values persist across multiple get calls."""
        # Set desired values
        self.sm.update_pin_desired("test_node_001", 5, 42.0)

        # Call get_runtime_state multiple times
        state1 = self.sm.get_runtime_state("test_node_001")
        state2 = self.sm.get_runtime_state("test_node_001")
        state3 = self.sm.get_runtime_state("test_node_001")

        # All should have the same desired value
        pin5_1 = next((p for p in state1["pins"] if p["gpio"] == 5), None)
        pin5_2 = next((p for p in state2["pins"] if p["gpio"] == 5), None)
        pin5_3 = next((p for p in state3["pins"] if p["gpio"] == 5), None)

        assert pin5_1["desired"] == 42.0
        assert pin5_2["desired"] == 42.0
        assert pin5_3["desired"] == 42.0

    def test_update_pin_actual_from_firmware(self):
        """Test updating actual values from firmware feedback."""
        # Simulate firmware feedback
        pins_data = [
            {"gpio": 5, "mode": "pwm", "value": 74.5, "name": "left_motor"},
            {"gpio": 6, "mode": "pwm", "value": 50.0, "name": "right_motor"},
        ]

        result = self.sm.update_pin_actual("test_node_001", pins_data)
        assert result is True

        # Check actual values were updated
        state_dict = self.sm.get_runtime_state("test_node_001")
        pin5 = next((p for p in state_dict["pins"] if p["gpio"] == 5), None)
        assert pin5 is not None
        assert pin5["actual"] == 74.5

    def test_synced_status(self):
        """Test that synced status is calculated correctly."""
        # Set desired value
        self.sm.update_pin_desired("test_node_001", 5, 75.0)

        # No actual value yet - should not be synced
        state_dict = self.sm.get_runtime_state("test_node_001")
        pin5 = next((p for p in state_dict["pins"] if p["gpio"] == 5), None)
        assert pin5["synced"] is False

        # Update actual value to match
        self.sm.update_pin_actual("test_node_001", [
            {"gpio": 5, "mode": "pwm", "value": 75.0}
        ])

        # Should now be synced
        state_dict = self.sm.get_runtime_state("test_node_001")
        pin5 = next((p for p in state_dict["pins"] if p["gpio"] == 5), None)
        assert pin5["synced"] is True

    def test_runtime_state_for_nonexistent_node(self):
        """Test that get_runtime_state returns None for non-existent node."""
        state = self.sm.get_runtime_state("nonexistent_node")
        assert state is None

    def test_update_desired_for_nonexistent_node(self):
        """Test that update_pin_desired fails for non-existent node."""
        result = self.sm.update_pin_desired("nonexistent_node", 5, 50.0)
        assert result is False


class TestGetControllablePins:
    """Test getting controllable pins."""

    def setup_method(self):
        """Setup test fixtures."""
        self.sm = StateManager(server_name="test-server")

        # Create and adopt a test node
        self.sm.update_node_from_announcement(announce({
            "node_id": "test_node_001",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))
        self.sm.adopt_node("test_node_001", "tracks", "Test Node")

        # Add mixed pin configuration
        pin_configs = {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
            "6": {"mode": "servo", "logical_name": "pan_servo"},
            "7": {"mode": "digital_out", "logical_name": "led"},
            "8": {"mode": "digital_in", "logical_name": "button"},
            "26": {"mode": "adc", "logical_name": "battery_voltage"},
        }
        self.sm.save_node_pin_config("test_node_001", pin_configs)

    def test_get_controllable_pins(self):
        """Test that only controllable pins are returned."""
        pins = self.sm.get_controllable_pins("test_node_001")

        # Should return pwm, servo, digital_out but NOT digital_in or adc
        assert len(pins) == 3

        gpios = [p["gpio"] for p in pins]
        assert 5 in gpios  # pwm
        assert 6 in gpios  # servo
        assert 7 in gpios  # digital_out
        assert 8 not in gpios  # digital_in - not controllable
        assert 26 not in gpios  # adc - not controllable


class TestStateManagerIntegration:
    """Integration tests for state manager with simulated workflow."""

    def test_full_control_workflow(self):
        """Test the full workflow: adopt -> configure -> control -> feedback."""
        sm = StateManager(server_name="test-server")

        # 1. Node announces itself
        sm.update_node_from_announcement(announce({
            "node_id": "robot_001",
            "hw": "RP2040",
            "fw": "1.0.0",
        }))

        # 2. Adopt the node
        result = sm.adopt_node("robot_001", "tracks", "Main Robot")
        assert result['success'] is True

        # 3. Configure pins
        pin_configs = {
            "5": {"mode": "pwm", "logical_name": "left_motor"},
            "6": {"mode": "pwm", "logical_name": "right_motor"},
        }
        result = sm.save_node_pin_config("robot_001", pin_configs)
        assert result['success'] is True

        # 4. User sets desired values (from Control page)
        sm.update_pin_desired("robot_001", 5, 50.0)
        sm.update_pin_desired("robot_001", 6, 75.0)

        # 5. Another client requests runtime state (from Node detail page)
        state = sm.get_runtime_state("robot_001")
        assert state is not None

        pin5 = next((p for p in state["pins"] if p["gpio"] == 5), None)
        pin6 = next((p for p in state["pins"] if p["gpio"] == 6), None)

        # Should see the desired values set by the first client
        assert pin5["desired"] == 50.0
        assert pin6["desired"] == 75.0

        # 6. Firmware sends back actual values
        sm.update_pin_actual("robot_001", [
            {"gpio": 5, "mode": "pwm", "value": 49.8},
            {"gpio": 6, "mode": "pwm", "value": 74.5},
        ])

        # 7. State should now show both desired and actual
        state = sm.get_runtime_state("robot_001")
        pin5 = next((p for p in state["pins"] if p["gpio"] == 5), None)

        assert pin5["desired"] == 50.0
        assert pin5["actual"] == 49.8
        assert pin5["synced"] is True  # Within tolerance

    def test_state_persists_across_page_navigation(self):
        """
        Simulate the user's issue: set value in Control page,
        navigate to Node detail, verify state is preserved.
        """
        sm = StateManager(server_name="test-server")

        # Setup node
        sm.update_node_from_announcement(announce({"node_id": "node_001", "hw": "RP2040", "fw": "1.0.0"}))
        sm.adopt_node("node_001", "tracks", "Test")
        sm.save_node_pin_config("node_001", {"5": {"mode": "pwm", "logical_name": "motor"}})

        # Simulate: User on Control page sets value to 80%
        sm.update_pin_desired("node_001", 5, 80.0)

        # Verify value is set
        state1 = sm.get_runtime_state("node_001")
        pin_before = next((p for p in state1["pins"] if p["gpio"] == 5), None)
        assert pin_before["desired"] == 80.0, "Desired value should be 80.0 after setting"

        # Simulate: User navigates to Node detail page
        # This would call get_runtime_state again
        state2 = sm.get_runtime_state("node_001")
        pin_after = next((p for p in state2["pins"] if p["gpio"] == 5), None)

        # The desired value should still be 80.0
        assert pin_after["desired"] == 80.0, (
            f"Desired value should persist as 80.0, but got {pin_after.get('desired')}"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
