"""
Edge-AI AMR Movement Risk Predictor
Balanced synthetic prototype for SIH demonstration.

IMPORTANT:
- Labels are synthetic, rule-generated prototype labels.
- This is not production-validated warehouse AI.
- Keep this module separate from fleet_controller.py until validated.
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report, confusion_matrix
from sklearn.model_selection import train_test_split

FEATURES = [
    "distance_to_peer",
    "relative_direction",
    "same_destination",
    "head_on",
    "next_cell_occupied",
    "nearby_amr_count",
    "waiting_steps",
    "recent_conflicts",
]

LABELS = ["LOW", "MEDIUM", "HIGH"]


@dataclass
class Scenario:
    name: str
    expected: str
    values: Dict[str, float]


def risk_score(x: Dict[str, float]) -> float:
    """Transparent synthetic labeling rule used only to create prototype data."""
    score = 0.0

    # Smaller distance => higher risk.
    score += max(0.0, min(4.0, 4.0 - x["distance_to_peer"])) * 1.0

    # Direction encoding used by the prototype:
    # 0 = separated, 1 = converging, 2 = head-on.
    score += x["relative_direction"] * 1.4
    score += x["same_destination"] * 2.2
    score += x["head_on"] * 5.0
    score += x["next_cell_occupied"] * 3.0
    score += min(3.0, x["nearby_amr_count"]) * 0.9
    score += min(4.0, x["waiting_steps"]) * 0.5
    score += min(4.0, x["recent_conflicts"]) * 0.8

    return score


def label_from_score(score: float) -> str:
    """Map the transparent synthetic score to three risk bands."""
    if score < 4.5:
        return "LOW"
    if score < 8.5:
        return "MEDIUM"
    return "HIGH"


def generate_balanced_dataset(samples_per_class: int = 1600, seed: int = 42) -> Tuple[np.ndarray, np.ndarray]:
    """Generate a deliberately balanced synthetic dataset.

    Rejection sampling is used so that every class contains exactly the
    requested number of examples.
    """
    rng = random.Random(seed)
    buckets: Dict[str, List[List[float]]] = {label: [] for label in LABELS}

    while min(len(v) for v in buckets.values()) < samples_per_class:
        row = {
            "distance_to_peer": rng.uniform(0.5, 8.0),
            "relative_direction": rng.choice([0, 1, 2]),
            "same_destination": rng.choice([0, 1]),
            "head_on": rng.choice([0, 1]),
            "next_cell_occupied": rng.choice([0, 1]),
            "nearby_amr_count": rng.randint(0, 3),
            "waiting_steps": rng.randint(0, 6),
            "recent_conflicts": rng.randint(0, 5),
        }

        label = label_from_score(risk_score(row))
        if len(buckets[label]) < samples_per_class:
            buckets[label].append([row[f] for f in FEATURES])

    X = np.array([row for label in LABELS for row in buckets[label]], dtype=float)
    y = np.array([label for label in LABELS for _ in buckets[label]])
    return X, y


def build_model(seed: int = 42) -> Tuple[RandomForestClassifier, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    X, y = generate_balanced_dataset(samples_per_class=1600, seed=seed)

    X_train, X_test, y_train, y_test = train_test_split(
        X,
        y,
        test_size=0.20,
        random_state=seed,
        stratify=y,
    )

    model = RandomForestClassifier(
        n_estimators=250,
        max_depth=10,
        min_samples_leaf=2,
        class_weight="balanced",
        random_state=seed,
        n_jobs=-1,
    )
    model.fit(X_train, y_train)
    return model, X_train, X_test, y_train, y_test


def predict_risk(model: RandomForestClassifier, values: Dict[str, float]) -> Tuple[str, float, Dict[str, float]]:
    vector = np.array([[values[f] for f in FEATURES]], dtype=float)
    probabilities = model.predict_proba(vector)[0]
    predicted = model.predict(vector)[0]
    probability_map = {str(cls): float(prob) * 100.0 for cls, prob in zip(model.classes_, probabilities)}
    confidence = float(np.max(probabilities)) * 100.0
    return str(predicted), confidence, probability_map


def main() -> None:
    print("=" * 70)
    print("EDGE-AI AMR MOVEMENT RISK PREDICTOR — BALANCED PROTOTYPE")
    print("=" * 70)

    model, X_train, X_test, y_train, y_test = build_model(seed=42)
    y_pred = model.predict(X_test)

    accuracy = accuracy_score(y_test, y_pred) * 100.0
    print("\nTraining compact local Random Forest...")
    print(f"Training samples : {len(X_train)}")
    print(f"Test samples     : {len(X_test)}")
    print(f"Prototype accuracy: {accuracy:.2f}%")
    print("NOTE: accuracy is measured against balanced synthetic rule-generated labels.")

    print("\n" + "=" * 70)
    print("TEST SET CLASS DISTRIBUTION")
    print("=" * 70)
    for label in LABELS:
        print(f"{label:>8} : {int(np.sum(y_test == label))}")

    scenarios = [
        Scenario(
            "SCENARIO 1 — OPEN AISLE / LOW RISK",
            "LOW",
            {
                "distance_to_peer": 5.0,
                "relative_direction": 0.0,
                "same_destination": 0.0,
                "head_on": 0.0,
                "next_cell_occupied": 0.0,
                "nearby_amr_count": 0.0,
                "waiting_steps": 0.0,
                "recent_conflicts": 0.0,
            },
        ),
        Scenario(
            "SCENARIO 2 — CONGESTION / MEDIUM RISK",
            "MEDIUM",
            {
                "distance_to_peer": 2.0,
                "relative_direction": 1.0,
                "same_destination": 0.0,
                "head_on": 0.0,
                "next_cell_occupied": 0.0,
                "nearby_amr_count": 2.0,
                "waiting_steps": 2.0,
                "recent_conflicts": 1.0,
            },
        ),
        Scenario(
            "SCENARIO 3 — HEAD-ON CONFLICT / HIGH RISK",
            "HIGH",
            {
                "distance_to_peer": 1.0,
                "relative_direction": 2.0,
                "same_destination": 1.0,
                "head_on": 1.0,
                "next_cell_occupied": 1.0,
                "nearby_amr_count": 2.0,
                "waiting_steps": 3.0,
                "recent_conflicts": 3.0,
            },
        ),
    ]

    for s in scenarios:
        predicted, confidence, probs = predict_risk(model, s.values)
        print("\n" + "=" * 70)
        print(s.name)
        print("=" * 70)
        print(f"Expected risk : {s.expected}")
        print(f"Predicted risk: {predicted}")
        print(f"Confidence    : {confidence:.2f}%")
        print("\nFeature values:")
        for key in FEATURES:
            print(f"  {key:<22}: {s.values[key]:.1f}")
        print("\nClass probabilities:")
        for label in LABELS:
            print(f"  {label:<8}: {probs.get(label, 0.0):.2f}%")

    print("\n" + "=" * 70)
    print("MODEL CLASSIFICATION REPORT")
    print("=" * 70)
    print(classification_report(y_test, y_pred, labels=LABELS, zero_division=0))

    print("=" * 70)
    print("CONFUSION MATRIX [rows=true, columns=predicted]")
    print("=" * 70)
    print("          LOW  MED  HIGH")
    cm = confusion_matrix(y_test, y_pred, labels=LABELS)
    for label, row in zip(LABELS, cm):
        print(f"{label:>5}    {row[0]:>4} {row[1]:>4} {row[2]:>5}")

    print("\nEDGE-AI BALANCED MODULE TEST COMPLETE")


if __name__ == "__main__":
    main()
