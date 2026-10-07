"""Play from the animation editor uses the timeline on screen.

start_animation used to load the animation from disk by id, so Play
ignored every unsaved edit and the operator heard and saw the last save.
The editor now sends its unsaved copy as a draft; the server plays that
copy and leaves the saved file alone.
"""
from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from saint_server.animation.models import Animation
from saint_server.webserver.state_manager import StateManager


class _RecordingRegistry:
    def __init__(self):
        self.started = []

    async def start(self, anim, loop=None, depth=0):
        self.started.append(anim)


def _run(coro):
    # A private loop, not asyncio.run(): on Python 3.9 asyncio.run()
    # leaves no current event loop behind, which breaks later tests that
    # call asyncio.get_event_loop().
    loop = asyncio.new_event_loop()
    try:
        return loop.run_until_complete(coro)
    finally:
        loop.close()


def _manager(tmp_path):
    sm = StateManager(server_name="test-server", config_dir=str(tmp_path))
    sm._animation_registry = _RecordingRegistry()
    sm.animation_store.save(Animation(id="wave", name="Wave", duration=2.0))
    return sm


def test_draft_is_played_instead_of_the_saved_copy(tmp_path):
    sm = _manager(tmp_path)
    draft = Animation(id="wave", name="Wave", duration=5.0).to_dict()

    result = _run(sm.start_animation("wave", draft=draft))

    assert result["success"] is True
    assert sm._animation_registry.started[0].duration == 5.0
    # Playing a draft is not saving it.
    assert sm.animation_store.get("wave").duration == 2.0


def test_without_a_draft_the_saved_copy_plays(tmp_path):
    sm = _manager(tmp_path)
    _run(sm.start_animation("wave"))
    assert sm._animation_registry.started[0].duration == 2.0


def test_draft_for_a_different_animation_is_refused(tmp_path):
    sm = _manager(tmp_path)
    draft = Animation(id="other", name="Other", duration=5.0).to_dict()
    result = _run(sm.start_animation("wave", draft=draft))
    assert result["success"] is False
    assert sm._animation_registry.started == []
