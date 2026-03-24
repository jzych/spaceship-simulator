#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

project_key=""
branch=""
output_path=""
sonar_base_url="https://sonarcloud.io"

usage() {
    cat <<'EOF'
Usage: export-sonar-findings.sh [--project-key KEY] [--branch NAME] [--output PATH] [--sonar-base-url URL]
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --project-key)
            project_key="$2"
            shift 2
            ;;
        --branch)
            branch="$2"
            shift 2
            ;;
        --output)
            output_path="$2"
            shift 2
            ;;
        --sonar-base-url)
            sonar_base_url="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ -z "${branch}" ]]; then
    branch="$(git -C "${repo_root}" branch --show-current)"
fi

if [[ -z "${branch}" ]]; then
    echo "Unable to resolve the current Git branch. Pass --branch explicitly." >&2
    exit 1
fi

if [[ -z "${output_path}" ]]; then
    output_path="${repo_root}/sonar-findings.md"
fi

if [[ -z "${project_key}" ]]; then
    project_key="$(python3 - "${repo_root}" <<'PY'
import pathlib
import re
import sys

repo_root = pathlib.Path(sys.argv[1])
pattern = re.compile(r"sonar\.projectKey=([A-Za-z0-9._:-]+)")

candidates = []
sonar_properties = repo_root / "sonar-project.properties"
if sonar_properties.exists():
    candidates.append(sonar_properties)

workflow_dir = repo_root / ".github" / "workflows"
if workflow_dir.exists():
    candidates.extend(sorted(workflow_dir.glob("*.yml")))
    candidates.extend(sorted(workflow_dir.glob("*.yaml")))

for candidate in candidates:
    match = pattern.search(candidate.read_text(encoding="utf-8"))
    if match:
        print(match.group(1))
        sys.exit(0)

sys.exit(1)
PY
)"
fi

if [[ -z "${project_key}" ]]; then
    echo "Unable to resolve the Sonar project key. Pass --project-key or add sonar.projectKey to local Sonar config." >&2
    exit 1
fi

curl_args=(-fsSL)
if [[ -n "${SONAR_TOKEN:-}" ]]; then
    curl_args+=(-H "Authorization: Bearer ${SONAR_TOKEN}")
fi

pull_request_json_file="$(mktemp)"
issues_json_file="$(mktemp)"
trap 'rm -f "${pull_request_json_file}" "${issues_json_file}"' EXIT

escaped_project_key="$(python3 -c 'import sys, urllib.parse; print(urllib.parse.quote(sys.argv[1], safe=""))' "${project_key}")"
curl "${curl_args[@]}" "${sonar_base_url}/api/project_pull_requests/list?project=${escaped_project_key}" > "${pull_request_json_file}"
pull_request_key="$(python3 - "${branch}" "${pull_request_json_file}" <<'PY'
import json
import pathlib
import sys

branch = sys.argv[1]
json_path = pathlib.Path(sys.argv[2])
data = json.loads(json_path.read_text(encoding="utf-8"))
for pull_request in data.get("pullRequests", []):
    if pull_request.get("branch") == branch:
        print(pull_request["key"])
        break
PY
)"

analysis_target_label="Branch"
analysis_target_value="${branch}"

if [[ -n "${pull_request_key}" ]]; then
    analysis_target_label="Pull Request"
    analysis_target_value="${pull_request_key}"
    issues_url="${sonar_base_url}/api/issues/search?componentKeys=${escaped_project_key}&pullRequest=${pull_request_key}&resolved=false&ps=500"
else
    escaped_branch="$(python3 -c 'import sys, urllib.parse; print(urllib.parse.quote(sys.argv[1], safe=""))' "${branch}")"
    issues_url="${sonar_base_url}/api/issues/search?componentKeys=${escaped_project_key}&branch=${escaped_branch}&resolved=false&ps=500"
fi

curl "${curl_args[@]}" "${issues_url}" > "${issues_json_file}"
python3 - "${output_path}" "${project_key}" "${analysis_target_label}" "${analysis_target_value}" "${branch}" "${issues_json_file}" <<'PY'
import json
import pathlib
import sys
from datetime import datetime

output_path = pathlib.Path(sys.argv[1])
project_key = sys.argv[2]
analysis_target_label = sys.argv[3]
analysis_target_value = sys.argv[4]
branch = sys.argv[5]
issues_json_path = pathlib.Path(sys.argv[6])

data = json.loads(issues_json_path.read_text(encoding="utf-8"))
issues = data.get("issues", [])

def value_or_na(value):
    if value is None or value == "":
        return "(n/a)"
    return str(value)

lines = [
    "# SonarCloud Findings",
    "",
    f"Project: {project_key}",
    f"{analysis_target_label}: {analysis_target_value}",
    f"Branch: {branch}",
    f"Generated: {datetime.now().isoformat(timespec='seconds')}",
    f"Total issues: {len(issues)}",
    "",
]

if not issues:
    lines.extend([
        "No open SonarCloud issues were found for this target.",
        "",
    ])
else:
    for issue in issues:
        lines.extend([
            f"## [{value_or_na(issue.get('severity'))}] {value_or_na(issue.get('rule'))}",
            f"- File: {value_or_na(issue.get('component'))}",
            f"- Line: {value_or_na(issue.get('line'))}",
            f"- Message: {value_or_na(issue.get('message'))}",
            f"- Type: {value_or_na(issue.get('type'))}",
            f"- Status: {value_or_na(issue.get('status'))}",
            f"- Issue key: {value_or_na(issue.get('key'))}",
            "",
        ])

output_path.write_text("\n".join(lines), encoding="utf-8")
print(f"Wrote {output_path} with {len(issues)} issues for {analysis_target_label} {analysis_target_value}.")
PY