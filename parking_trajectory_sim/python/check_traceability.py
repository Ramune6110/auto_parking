"""Apply equation anchors and validate documentation/code IDs. Traceability: IMPL-TRACE-01."""

from argparse import ArgumentParser
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
DOCUMENT_PREFIXES = {
    "docs/fundamentals/coordinate_system.md": "COORD",
    "docs/fundamentals/path_generation.md": "PATH",
    "docs/fundamentals/trajectory_generation.md": "TRAJ",
    "docs/fundamentals/kinematic_bicycle_model.md": "BICYCLE",
    "docs/fundamentals/steering_actuator.md": "ACT",
    "docs/fundamentals/linear_error_model.md": "LINERR",
    "docs/controllers/feedforward.md": "FF",
    "docs/controllers/yaw_feedback.md": "YAW",
    "docs/controllers/lateral_feedback.md": "LAT",
    "docs/controllers/pure_pursuit.md": "PP",
    "docs/controllers/stanley.md": "STANLEY",
    "docs/controllers/lqr.md": "LQR",
    "docs/controllers/longitudinal_velocity_control.md": "LON",
    "docs/controllers/longitudinal_constraints.md": "LONLIM",
    "docs/controllers/stop_position_control.md": "STOP",
    "docs/controllers/longitudinal_lateral_coordination.md": "COOP",
    "docs/controllers/linear_mpc.md": "MPC",
}

# These equations have a direct executable implementation and must not lose
# their reverse link when source files are reorganized.
REQUIRED_IMPLEMENTATION_IDS = {
    "COORD-EQ-05",
    "BICYCLE-EQ-06",
    "ACT-EQ-04",
    "FF-EQ-04",
    "YAW-EQ-08",
    "LAT-EQ-10",
    "PP-EQ-10",
    "STANLEY-EQ-10",
    "LINERR-EQ-19",
    "LQR-EQ-05",
    "LON-EQ-02",
    "LONLIM-EQ-04",
    "STOP-EQ-04",
    "COOP-EQ-05",
    "MPC-EQ-15",
}

DIRECT_CALCULATION_IDS = {
    "src/models/kinematic_bicycle.cpp": {"BICYCLE-EQ-06", "BICYCLE-EQ-09", "BICYCLE-EQ-11", "BICYCLE-EQ-12", "BICYCLE-EQ-13"},
    "src/models/linear_error_model.cpp": {"LINERR-EQ-18", "LINERR-EQ-19"},
    "src/control/discrete_lqr.cpp": {"LQR-EQ-04", "LQR-EQ-05"},
    "src/controllers/lqr_controller.cpp": {"LQR-EQ-06", "LQR-EQ-07", "LQR-EQ-08", "LQR-EQ-12"},
    "src/actuators/steering_actuator.cpp": {"ACT-EQ-02", "ACT-EQ-05", "ACT-EQ-06", "ACT-EQ-08"},
    "src/trajectory/path_generator.cpp": {"PATH-EQ-04", "PATH-EQ-05", "PATH-EQ-10", "PATH-EQ-11", "PATH-EQ-12"},
    "src/trajectory/velocity_profile.cpp": {"TRAJ-EQ-09", "TRAJ-EQ-11", "TRAJ-EQ-12", "TRAJ-EQ-13", "TRAJ-EQ-15", "TRAJ-EQ-17"},
    "src/controllers/longitudinal/velocity_stop_controller.cpp": {"LON-EQ-01", "LON-EQ-02", "LONLIM-EQ-01", "LONLIM-EQ-04", "STOP-EQ-02", "STOP-EQ-04", "STOP-EQ-06"},
    "src/coordination/longitudinal_lateral_coordinator.cpp": {"COOP-EQ-01", "COOP-EQ-02", "COOP-EQ-03", "COOP-EQ-04", "COOP-EQ-05"},
    "src/controllers/linear_mpc_controller.cpp": {"MPC-EQ-01", "MPC-EQ-02", "MPC-EQ-05", "MPC-EQ-06", "MPC-EQ-09", "MPC-EQ-10", "MPC-EQ-11", "MPC-EQ-12", "MPC-EQ-13", "MPC-EQ-14", "MPC-EQ-15"},
}


def equation_id(prefix: str, number: int) -> str:
    return f"{prefix}-EQ-{number:02d}"


def apply_equation_anchors(path: Path, prefix: str) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    output = []
    index = 0
    while index < len(lines):
        if lines[index].strip() != "$$":
            output.append(lines[index])
            index += 1
            continue
        end = index + 1
        while end < len(lines) and lines[end].strip() != "$$":
            end += 1
        block = "\n".join(lines[index:end + 1])
        match = re.search(r"\\tag\{(\d+)\}", block)
        if match:
            identifier = equation_id(prefix, int(match.group(1)))
            anchor = f'<a id="{identifier.lower()}"></a>'
            label = f'**理論ID：`{identifier}`**'
            if output and re.fullmatch(r"\*\*理論ID：`[A-Z]+-EQ-\d{2}`\*\*", output[-1].strip()):
                output[-1] = label
            elif len(output) >= 2 and output[-2].strip() == anchor:
                output[-1] = label
                output.append("")
            elif not output or output[-1].strip() != anchor:
                output.extend([anchor, "", label, ""])
        output.extend(lines[index:end + 1])
        index = end + 1
    path.write_text("\n".join(output) + "\n", encoding="utf-8")


def collect_document_ids():
    identifiers = set()
    errors = []
    for relative_path, prefix in DOCUMENT_PREFIXES.items():
        path = ROOT / relative_path
        text = path.read_text(encoding="utf-8")
        equation_numbers = [int(value) for value in re.findall(r"\\tag\{(\d+)\}", text)]
        for number in equation_numbers:
            identifier = equation_id(prefix, number)
            anchor = f'id="{identifier.lower()}"'
            if anchor not in text:
                errors.append(f"missing anchor {identifier} in {relative_path}")
            if f"**理論ID：`{identifier}`**" not in text:
                errors.append(f"missing visible label {identifier} in {relative_path}")
            identifiers.add(identifier)
    return identifiers, errors


def validate_code_references(document_ids):
    errors = []
    referenced = set()
    source_files = list((ROOT / "include").rglob("*.hpp"))
    source_files += list((ROOT / "src").rglob("*.cpp"))
    source_files += list((ROOT / "tests").rglob("*.cpp"))
    source_files += list((ROOT / "python").rglob("*.py"))
    pattern = re.compile(r"\b(?:COORD|PATH|TRAJ|BICYCLE|ACT|LINERR|FF|YAW|LAT|PP|STANLEY|LQR|LON|LONLIM|STOP|COOP|MPC)-EQ-\d{2}\b")
    marker_pattern = re.compile(r"(?:[A-Z]+-EQ-\d{2}|IMPL-[A-Z0-9-]+)")
    for path in source_files:
        text = path.read_text(encoding="utf-8")
        if not marker_pattern.search(text):
            errors.append(f"missing traceability marker in {path.relative_to(ROOT)}")
        for identifier in pattern.findall(text):
            referenced.add(identifier)
            if identifier not in document_ids:
                errors.append(f"unknown theory ID {identifier} in {path.relative_to(ROOT)}")
    for identifier in sorted(REQUIRED_IMPLEMENTATION_IDS - referenced):
        errors.append(f"required implementation link is missing: {identifier}")
    for relative_path, required_ids in DIRECT_CALCULATION_IDS.items():
        lines = (ROOT / relative_path).read_text(encoding="utf-8").splitlines()[1:]
        calculation_text = "\n".join(lines)
        for identifier in sorted(required_ids):
            if identifier not in calculation_text:
                errors.append(
                    f"missing calculation-level link {identifier} in {relative_path}")
    return referenced, errors


def validate_document_catalog():
    errors = []
    index_path = ROOT / "docs/index.md"
    index_text = index_path.read_text(encoding="utf-8")
    listed = set()
    for target in re.findall(r"\[[^]]+\]\(([^)#]+)", index_text):
        path = (index_path.parent / target).resolve()
        if path.suffix == ".md":
            listed.add(path)
    all_documents = {
        path.resolve() for path in (ROOT / "docs").rglob("*.md")
        if path.name != "index.md"
    }
    for path in sorted(all_documents - listed):
        errors.append(f"document is missing from index.md: {path.relative_to(ROOT)}")

    illustrated_documents = (
        "docs/controllers/longitudinal_velocity_control.md",
        "docs/controllers/longitudinal_constraints.md",
        "docs/controllers/stop_position_control.md",
        "docs/controllers/longitudinal_lateral_coordination.md",
    )
    for relative_path in illustrated_documents:
        text = (ROOT / relative_path).read_text(encoding="utf-8")
        title = next((line for line in text.splitlines() if line.startswith("# ")), "")
        if re.match(r"#\s+段階\d+", title):
            errors.append(f"roadmap stage number must not be in title: {relative_path}")
        if "```mermaid" not in text:
            errors.append(f"control document has no Mermaid figure: {relative_path}")
    return errors


def main() -> int:
    parser = ArgumentParser()
    parser.add_argument("--apply-anchors", action="store_true")
    args = parser.parse_args()
    if args.apply_anchors:
        for relative_path, prefix in DOCUMENT_PREFIXES.items():
            apply_equation_anchors(ROOT / relative_path, prefix)

    document_ids, errors = collect_document_ids()
    errors.extend(validate_document_catalog())
    referenced_ids, code_errors = validate_code_references(document_ids)
    errors.extend(code_errors)
    if not referenced_ids:
        errors.append("no theory IDs are referenced from source code")
    if errors:
        print("Traceability validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    print(
        f"Traceability OK: {len(document_ids)} equations, "
        f"{len(referenced_ids)} equation IDs referenced from code.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
