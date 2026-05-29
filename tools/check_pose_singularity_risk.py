"""Check configured joint poses for obvious singularity risk on UR5.

This is a heuristic checker, not a full Jacobian-based singularity analysis.
It flags poses that are commonly problematic for UR robots:
- elbow nearly straight (J3 near 0 deg)
- wrist alignment around J5 near 0 deg
- shoulder/elbow combinations that tend to reduce IK robustness
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from dotenv import load_dotenv

load_dotenv(ROOT / ".env")

import config


POSES = {
    "HOME_JOINTS": config.HOME_JOINTS,
    "SCAN_APPROACH_JOINTS": config.SCAN_APPROACH_JOINTS,
    "SCAN_POSE_JOINTS": config.SCAN_POSE_JOINTS,
}


def deg_list(q):
    return [math.degrees(v) for v in q]


def classify_pose(q_deg):
    findings = []
    severity = "ok"

    j2 = q_deg[1]
    j3 = q_deg[2]
    j5 = q_deg[4]

    if abs(j3) <= 5:
        findings.append(f"J3 gan 0 deg ({j3:.2f}) -> nguy co elbow singularity rat cao")
        severity = "high"
    elif abs(j3) <= 10:
        findings.append(f"J3 kha gan 0 deg ({j3:.2f}) -> nguy co elbow singularity")
        severity = "medium"

    if abs(j5) <= 5:
        findings.append(f"J5 gan 0 deg ({j5:.2f}) -> nguy co wrist singularity")
        severity = "high"
    elif abs(abs(j5) - 180) <= 5:
        findings.append(f"J5 gan +/-180 deg ({j5:.2f}) -> nguy co wrist singularity")
        severity = "high" if severity != "high" else severity

    if abs(j2 + j3) <= 5:
        findings.append(
            f"J2 + J3 gan 0 deg ({j2 + j3:.2f}) -> tay gan duoi thang theo mat phang shoulder-elbow"
        )
        if severity == "ok":
            severity = "medium"

    if not findings:
        findings.append("Khong thay dau hieu singularity ro rang theo heuristic")

    return severity, findings


def main():
    print("=== POSE SINGULARITY RISK CHECK ===")
    print("Heuristic only: uu tien phat hien pose co J3 gan 0 deg.\n")

    for name, q in POSES.items():
        q_deg = deg_list(q)
        severity, findings = classify_pose(q_deg)
        print(name)
        print(f"  joints_deg: {[round(v, 2) for v in q_deg]}")
        print(f"  severity: {severity}")
        for item in findings:
            print(f"  - {item}")
        print()


if __name__ == "__main__":
    main()
