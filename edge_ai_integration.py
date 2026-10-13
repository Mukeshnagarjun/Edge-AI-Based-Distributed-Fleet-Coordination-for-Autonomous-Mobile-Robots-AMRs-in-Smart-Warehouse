"""
Edge-AI integration adapter for the SIH AMR fleet controller.

Purpose:
    Connect the validated balanced Edge-AI risk predictor to the existing
    fleet-controller decision loop without replacing the controller's
    existing conflict-resolution rules.

Architecture:
    P2P/local state -> Edge-AI risk -> advisory action -> existing controller

Important:
    The underlying model is trained on synthetic rule-generated labels.
    This adapter is suitable for prototype/demo integration, not production
    safety certification.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from typing import Dict, Literal

from edge_ai_risk import FEATURES, LABELS, build_model, predict_risk

Action = Literal["MOVE", "WAIT", "REPLAN"]


@dataclass(frozen=True)
class RiskAssessment:
    risk_level: str
    confidence: float
    probabilities: Dict[str, float]
    action: Action
    reason: str


@lru_cache(maxsize=1)
def get_edge_ai_model():
    """Train the compact local model once and reuse it for later decisions."""
    model, *_ = build_model(seed=42)
    return model


def assess_risk(
    *,
    distance_to_peer: float,
    relative_direction: float,
    same_destination: float,
    head_on: float,
    next_cell_occupied: float,
    nearby_amr_count: float,
    waiting_steps: float,
    recent_conflicts: float,
    conflict_detected: bool = False,
) -> RiskAssessment:
    """Run Edge-AI inference and convert risk into a controller advisory.

    The existing controller remains responsible for hard movement rules.
    The Edge-AI layer only recommends MOVE, WAIT, or REPLAN.
    """

    values = {
        "distance_to_peer": float(distance_to_peer),
        "relative_direction": float(relative_direction),
        "same_destination": float(same_destination),
        "head_on": float(head_on),
        "next_cell_occupied": float(next_cell_occupied),
        "nearby_amr_count": float(nearby_amr_count),
        "waiting_steps": float(waiting_steps),
        "recent_conflicts": float(recent_conflicts),
    }

    # Validate inputs early so bad telemetry cannot silently enter the model.
    missing = [name for name in FEATURES if name not in values]
    if missing:
        raise ValueError(f"Missing Edge-AI feature(s): {', '.join(missing)}")

    model = get_edge_ai_model()
    risk_level, confidence, probabilities = predict_risk(model, values)

    # Advisory policy:
    # LOW    -> keep moving normally.
    # MEDIUM -> wait when a real conflict is already detected; otherwise move.
    # HIGH   -> request an early local replan.
    if risk_level == "HIGH":
        action: Action = "REPLAN"
        reason = "High predicted coordination risk; request early local replanning."
    elif risk_level == "MEDIUM" and conflict_detected:
        action = "WAIT"
        reason = "Medium predicted risk with an active conflict; wait for peer resolution."
    else:
        action = "MOVE"
        reason = "Predicted risk does not require additional intervention."

    return RiskAssessment(
        risk_level=risk_level,
        confidence=confidence,
        probabilities=probabilities,
        action=action,
        reason=reason,
    )


def print_assessment(assessment: RiskAssessment) -> None:
    """Human-readable output for the SIH terminal demo."""
    print("\nEDGE-AI DECISION")
    print(f"Risk level : {assessment.risk_level}")
    print(f"Confidence : {assessment.confidence:.2f}%")
    print(f"Action     : {assessment.action}")
    print(f"Reason     : {assessment.reason}")
    print("Probabilities:")
    for label in LABELS:
        print(f"  {label:<8}: {assessment.probabilities.get(label, 0.0):.2f}%")


if __name__ == "__main__":
    print("=" * 70)
    print("EDGE-AI FLEET-CONTROLLER INTEGRATION TEST")
    print("=" * 70)

    scenarios = [
        (
            "OPEN AISLE",
            dict(
                distance_to_peer=5,
                relative_direction=0,
                same_destination=0,
                head_on=0,
                next_cell_occupied=0,
                nearby_amr_count=0,
                waiting_steps=0,
                recent_conflicts=0,
                conflict_detected=False,
            ),
        ),
        (
            "CONGESTION",
            dict(
                distance_to_peer=2,
                relative_direction=1,
                same_destination=0,
                head_on=0,
                next_cell_occupied=0,
                nearby_amr_count=2,
                waiting_steps=2,
                recent_conflicts=1,
                conflict_detected=True,
            ),
        ),
        (
            "HEAD-ON CONFLICT",
            dict(
                distance_to_peer=1,
                relative_direction=2,
                same_destination=1,
                head_on=1,
                next_cell_occupied=1,
                nearby_amr_count=2,
                waiting_steps=3,
                recent_conflicts=3,
                conflict_detected=True,
            ),
        ),
    ]

    for name, kwargs in scenarios:
        print("\n" + "-" * 70)
        print(name)
        assessment = assess_risk(**kwargs)
        print_assessment(assessment)

    print("\nIntegration adapter test complete.")
