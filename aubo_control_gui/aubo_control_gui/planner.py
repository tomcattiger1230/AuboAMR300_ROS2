"""Deterministic joint-space planning helpers."""
from __future__ import annotations
import math
from dataclasses import dataclass
from typing import Iterable, Sequence
JOINT_COUNT = 6
class PlanningError(ValueError): pass
@dataclass(frozen=True)
class JointPlan:
    start: tuple[float, ...]
    target: tuple[float, ...]
    waypoints: tuple[tuple[float, ...], ...]
    intermediate_count: int
def _joints(values: Iterable[float], name: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != JOINT_COUNT: raise PlanningError(f"{name}必须包含 {JOINT_COUNT} 个关节值")
    if not all(math.isfinite(value) for value in result): raise PlanningError(f"{name}包含无效数值")
    return result
def plan_joint_path(start: Sequence[float], target: Sequence[float], intermediate_count: int = 0) -> JointPlan:
    """Build a path in radians, checking the i16 URDF's +/-2pi limits."""
    begin, end = _joints(start, "初始位置"), _joints(target, "目标位置")
    if intermediate_count < 0 or intermediate_count > 32: raise PlanningError("中间点数量必须在 0 到 32 之间")
    if any(abs(value) > 2.0*math.pi+1e-9 for value in (*begin,*end)): raise PlanningError("关节目标超出 URDF 限位 +/-360 deg")
    segments = intermediate_count + 1
    points = tuple(tuple(a+(b-a)*step/segments for a,b in zip(begin,end)) for step in range(1,segments+1))
    return JointPlan(begin,end,points,intermediate_count)
def start_matches_actual(start: Sequence[float], actual: Sequence[float], tolerance: float) -> bool:
    begin,current = _joints(start,"初始位置"),_joints(actual,"当前位置")
    return max(abs(a-b) for a,b in zip(begin,current)) <= tolerance

def plan_cartesian_path(start: Sequence[float], target: Sequence[float], intermediate_count: int = 0) -> tuple[tuple[float,...],...]:
    """Interpolate XYZ positions in metres for Cartesian fallback motion."""
    begin=tuple(float(v) for v in start); end=tuple(float(v) for v in target)
    if len(begin)!=3 or len(end)!=3 or not all(math.isfinite(v) for v in (*begin,*end)):
        raise PlanningError("末端位置必须包含有效的 X、Y、Z")
    if intermediate_count<0 or intermediate_count>32: raise PlanningError("中间点数量必须在 0 到 32 之间")
    segments=intermediate_count+1
    return tuple(tuple(a+(b-a)*step/segments for a,b in zip(begin,end)) for step in range(1,segments+1))
