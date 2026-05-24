#!/usr/bin/env python3

import argparse
import itertools
from pathlib import Path

PARAM_VALUES = {
    "theta_bins": [36, 72, 144],
    "reverse_penalty": [1.0, 2.0, 5.0],
    "steer_penalty": [0.0, 1.0, 4.0],
    "gear_switch_penalty": [0.0, 1.0, 4.0],
    "steer_change_penalty": [0.0, 1.0, 4.0],
    "analytic_expansion_distance": [4, 8, 16],
    "analytic_expansion_interval": [5, 25, 100],
}

BASELINE = {
    "theta_bins": 72,
    "reverse_penalty": 2.0,
    "steer_penalty": 1.0,
    "gear_switch_penalty": 1.0,
    "steer_change_penalty": 1.0,
    "analytic_expansion_distance": 8,
    "analytic_expansion_interval": 25,
}

PENALTY_KEYS = [
    "reverse_penalty",
    "steer_penalty",
    "gear_switch_penalty",
    "steer_change_penalty",
]


COMBOS = {
    "coarse_low_cost_fast_analytic": {
        "theta_bins": 36,
        "reverse_penalty": 1.0,
        "steer_penalty": 0.0,
        "gear_switch_penalty": 0.0,
        "steer_change_penalty": 0.0,
        "analytic_expansion_distance": 16,
        "analytic_expansion_interval": 5,
    },
    "coarse_high_cost_rare_analytic": {
        "theta_bins": 36,
        "reverse_penalty": 5.0,
        "steer_penalty": 4.0,
        "gear_switch_penalty": 4.0,
        "steer_change_penalty": 4.0,
        "analytic_expansion_distance": 4,
        "analytic_expansion_interval": 100,
    },
    "fine_low_cost_fast_analytic": {
        "theta_bins": 144,
        "reverse_penalty": 1.0,
        "steer_penalty": 0.0,
        "gear_switch_penalty": 0.0,
        "steer_change_penalty": 0.0,
        "analytic_expansion_distance": 16,
        "analytic_expansion_interval": 5,
    },
    "fine_high_cost_rare_analytic": {
        "theta_bins": 144,
        "reverse_penalty": 5.0,
        "steer_penalty": 4.0,
        "gear_switch_penalty": 4.0,
        "steer_change_penalty": 4.0,
        "analytic_expansion_distance": 4,
        "analytic_expansion_interval": 100,
    },
    "baseline_costs_fast_analytic": {
        "theta_bins": 72,
        "reverse_penalty": 2.0,
        "steer_penalty": 1.0,
        "gear_switch_penalty": 1.0,
        "steer_change_penalty": 1.0,
        "analytic_expansion_distance": 16,
        "analytic_expansion_interval": 5,
    },
    "baseline_costs_rare_analytic": {
        "theta_bins": 72,
        "reverse_penalty": 2.0,
        "steer_penalty": 1.0,
        "gear_switch_penalty": 1.0,
        "steer_change_penalty": 1.0,
        "analytic_expansion_distance": 4,
        "analytic_expansion_interval": 100,
    },
    "low_reverse_high_steer": {
        "theta_bins": 72,
        "reverse_penalty": 1.0,
        "steer_penalty": 4.0,
        "gear_switch_penalty": 1.0,
        "steer_change_penalty": 4.0,
        "analytic_expansion_distance": 8,
        "analytic_expansion_interval": 25,
    },
    "high_reverse_low_steer": {
        "theta_bins": 72,
        "reverse_penalty": 5.0,
        "steer_penalty": 0.0,
        "gear_switch_penalty": 1.0,
        "steer_change_penalty": 0.0,
        "analytic_expansion_distance": 8,
        "analytic_expansion_interval": 25,
    },
    "switching_free": {
        "theta_bins": 72,
        "reverse_penalty": 2.0,
        "steer_penalty": 1.0,
        "gear_switch_penalty": 0.0,
        "steer_change_penalty": 1.0,
        "analytic_expansion_distance": 8,
        "analytic_expansion_interval": 25,
    },
    "switching_expensive": {
        "theta_bins": 72,
        "reverse_penalty": 2.0,
        "steer_penalty": 1.0,
        "gear_switch_penalty": 4.0,
        "steer_change_penalty": 1.0,
        "analytic_expansion_distance": 8,
        "analytic_expansion_interval": 25,
    },
}


def value_token(value):
    return str(value).replace(".", "p")


def format_value(value):
    if isinstance(value, float):
        return f"{value:.1f}"
    return str(value)


def replace_yaml_value(text, key, value):
    lines = []
    replaced = False
    for line in text.splitlines():
        stripped = line.lstrip()
        indent = line[:len(line) - len(stripped)]
        if stripped.startswith(f"{key}:"):
            comment_index = line.find("#")
            comment = ""
            if comment_index != -1:
                comment = "  " + line[comment_index:].strip()
            lines.append(f"{indent}{key}: {format_value(value)}{comment}")
            replaced = True
        else:
            lines.append(line)
    if not replaced:
        raise ValueError(f"missing key in base yaml: {key}")
    return "\n".join(lines) + "\n"


def apply_overrides(base_text, overrides, max_iterations):
    text = base_text
    merged = dict(overrides)
    merged["max_iterations"] = max_iterations
    for key, value in merged.items():
        text = replace_yaml_value(text, key, value)
    return text


def single_groups():
    groups = [("baseline", dict(BASELINE))]
    for key, values in PARAM_VALUES.items():
        for value in values:
            if value == BASELINE[key]:
                continue
            groups.append((f"{key}_{value_token(value)}", {key: value}))
    return groups


def combo_groups():
    groups = single_groups()
    seen = {name for name, _ in groups}
    for name, overrides in COMBOS.items():
        if name not in seen:
            groups.append((name, dict(overrides)))
            seen.add(name)
    return groups


def penalty_groups():
    groups = []
    for values in itertools.product(*(PARAM_VALUES[key] for key in PENALTY_KEYS)):
        overrides = dict(zip(PENALTY_KEYS, values))
        name = "penalty_" + "_".join(f"{key}_{value_token(overrides[key])}" for key in PENALTY_KEYS)
        groups.append((name, overrides))
    return groups


def full_groups():
    keys = list(PARAM_VALUES)
    groups = []
    for values in itertools.product(*(PARAM_VALUES[key] for key in keys)):
        overrides = dict(zip(keys, values))
        name = "combo_" + "_".join(f"{key}_{value_token(overrides[key])}" for key in keys)
        groups.append((name, overrides))
    return groups


def build_groups(mode):
    if mode == "single":
        return single_groups()
    if mode == "combo":
        return combo_groups()
    if mode == "penalty":
        return penalty_groups()
    if mode == "full":
        return full_groups()
    raise ValueError(f"unsupported mode: {mode}")


def parse_args():
    parser = argparse.ArgumentParser(description="Generate Hybrid A* testbench parameter configs")
    parser.add_argument("--base", default="config/default.yaml")
    parser.add_argument("--output-dir", default="config/testbench/generated")
    parser.add_argument("--max-iterations", type=int, required=True)
    parser.add_argument("--mode", choices=["single", "combo", "penalty", "full"], default="combo")
    return parser.parse_args()


def main():
    args = parse_args()
    base_path = Path(args.base)
    output_dir = Path(args.output_dir)
    base_text = base_path.read_text(encoding="utf-8")
    output_dir.mkdir(parents=True, exist_ok=True)

    group_entries = []
    for name, overrides in build_groups(args.mode):
        config_path = output_dir / f"{name}.yaml"
        config_path.write_text(
            apply_overrides(base_text, overrides, args.max_iterations),
            encoding="utf-8",
        )
        group_entries.append(f"{name} {config_path.as_posix()}")

    groups_path = output_dir / "groups.txt"
    groups_path.write_text("\n".join(group_entries) + "\n", encoding="utf-8")
    print(f"generated {len(group_entries)} groups")
    print(f"groups: {groups_path}")


if __name__ == "__main__":
    main()
