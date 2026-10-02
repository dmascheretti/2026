"""Check that every parameter in params.yaml has value, unit, source and status.

Usage: python3 model/check_params.py
Exits with code 1 and lists the problems if any entry is incomplete.
"""
import sys
from pathlib import Path

import yaml

PARAMS_FILE = Path(__file__).parent / "params.yaml"
REQUIRED_FIELDS = ["value", "unit", "source", "status"]
ALLOWED_STATUS = ["constant", "datasheet", "literature", "assumed", "own_id"]


def check(params):
    problems = []
    sources = params["sources"]
    for group_name, group in params.items():
        if group_name == "sources":
            continue
        for name, entry in group.items():
            full_name = group_name + "." + name
            for field in REQUIRED_FIELDS:
                if field not in entry:
                    problems.append(full_name + ": missing '" + field + "'")
            if entry.get("source") not in sources:
                problems.append(full_name + ": unknown source '" + str(entry.get("source")) + "'")
            if entry.get("status") not in ALLOWED_STATUS:
                problems.append(full_name + ": bad status '" + str(entry.get("status")) + "'")
    return problems


def main():
    with open(PARAMS_FILE) as f:
        params = yaml.safe_load(f)
    problems = check(params)
    for problem in problems:
        print(problem)
    if problems:
        sys.exit(1)
    print("params.yaml OK")


if __name__ == "__main__":
    main()
