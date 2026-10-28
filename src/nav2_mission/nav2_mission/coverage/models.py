"""Small dependency-free models shared by coverage components."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class CoverageState(str, Enum):
    IDLE = "IDLE"
    VALIDATING_REGION = "VALIDATING_REGION"
    COMPUTING_PATH = "COMPUTING_PATH"
    NAVIGATING_TO_START = "NAVIGATING_TO_START"
    FOLLOWING_COVERAGE_PATH = "FOLLOWING_COVERAGE_PATH"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELED = "CANCELED"


class EditMode(str, Enum):
    OUTER_DRAFT = "OUTER_DRAFT"
    OUTER_CLOSED = "OUTER_CLOSED"
    HOLE_DRAFT = "HOLE_DRAFT"


@dataclass(frozen=True)
class Point2D:
    x: float
    y: float


@dataclass(frozen=True)
class RegionSnapshot:
    outer: Tuple[Point2D, ...]
    holes: Tuple[Tuple[Point2D, ...], ...]


@dataclass
class LastResult:
    status: str = "NONE"
    error_code: Optional[str] = None
    message: str = ""
    report_path: Optional[str] = None
