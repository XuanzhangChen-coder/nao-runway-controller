"""Command-line synthetic demonstration of the public controller."""

from __future__ import annotations

import argparse
import math

from .controller import RunwayController
from .synthetic import simulate_convergence


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=160)
    parser.add_argument("--heading-deg", type=float, default=11.0)
    parser.add_argument("--lateral-mm", type=float, default=260.0)
    parser.add_argument(
        "--dropout-start",
        type=int,
        default=50,
        help="First step in a ten-frame synthetic vision dropout.",
    )
    args = parser.parse_args()

    dropouts = range(args.dropout_start, args.dropout_start + 10)
    history = simulate_convergence(
        RunwayController(),
        steps=args.steps,
        initial_heading_rad=math.radians(args.heading_deg),
        initial_lateral_mm=args.lateral_mm,
        dropout_steps=dropouts,
    )

    print("step  source          heading_deg  lateral_mm  rot_cmd  y_cmd")
    stride = max(1, args.steps // 8)
    for index, state in enumerate(history):
        if index % stride and index != len(history) - 1:
            continue
        command = state.diagnostics.command
        print(
            f"{index:4d}  {state.diagnostics.source.value:14s} "
            f"{math.degrees(state.heading_error_rad):11.3f} "
            f"{state.lateral_error_mm:11.2f} "
            f"{command.rotation_ratio:8.4f} {command.lateral_ratio:7.4f}"
        )

    final = history[-1]
    print(
        "\nSynthetic final error: "
        f"heading={math.degrees(final.heading_error_rad):.3f} deg, "
        f"lateral={final.lateral_error_mm:.2f} mm"
    )
    print("This is a deterministic algorithm check, not a real-robot benchmark.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
