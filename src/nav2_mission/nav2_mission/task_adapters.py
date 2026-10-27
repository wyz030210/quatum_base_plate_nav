"""Replaceable system boundaries for unified task control (no ROS dependency)."""
from dataclasses import dataclass, field
from typing import Protocol


@dataclass(frozen=True)
class LocalizationState:
    valid: bool
    frame_id: str
    received_at: float
    source_age_sec: float
    error: str = ''
    source: str = ''


@dataclass(frozen=True)
class StopState:
    ready: bool
    stopped: bool
    stop_requested: bool
    received_at: float
    feedback_age_sec: float
    manual_override: bool = False
    emergency_stop: bool = False
    fault: str = ''


@dataclass(frozen=True)
class ExecutionUpdate:
    phase: str
    done: bool = False
    succeeded: bool = False
    error: str = ''
    details: dict = field(default_factory=dict)


class Operation(Protocol):
    """Own all pending goal responses until terminal, including after cancel."""

    def poll(self, now: float) -> ExecutionUpdate:
        ...

    def cancel(self) -> None:
        ...


class TaskHandler(Protocol):
    """Task-specific validation and execution; manager owns lifecycle and safety."""

    def prepare(self, request: dict) -> dict:
        ...

    def begin(self, task_id: str, config: dict, now: float) -> Operation:
        ...
