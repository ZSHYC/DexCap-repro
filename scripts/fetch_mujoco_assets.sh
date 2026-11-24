#!/usr/bin/env bash
set -euo pipefail

# Official MuJoCo Menagerie, pinned so model geometry and limits cannot drift.
readonly REVISION=eea4c7431b63da8278b0ff31638e0b0e941cec0e
readonly REPOSITORY=https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie
readonly TREE=https://api.github.com/repos/google-deepmind/mujoco_menagerie/git/trees/$REVISION
readonly PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly DESTINATION="$PROJECT_ROOT/data/mujoco_menagerie"
if [[ -f "$DESTINATION/SOURCE" \
      && "$(cat "$DESTINATION/SOURCE")" == "MuJoCo Menagerie $REVISION" \
      && -f "$DESTINATION/LICENSE" \
      && -f "$DESTINATION/franka_emika_panda/panda_nohand.xml" \
      && -f "$DESTINATION/franka_emika_panda/assets/link0.stl" ]]; then
  exit 0
fi
if [[ -e "$DESTINATION" ]]; then
  printf 'Existing asset directory is not the requested Menagerie revision: %s\n' "$DESTINATION" >&2
  exit 1
fi
temporary_checkout="$(mktemp -d)"
trap 'rm -rf "$temporary_checkout"' EXIT

while IFS= read -r path; do
  mkdir -p "$temporary_checkout/$(dirname "$path")"
  curl --fail --silent --show-error "$REPOSITORY/$REVISION/$path" \
    --output "$temporary_checkout/$path" &
  while (( $(jobs -rp | wc -l) >= 8 )); do wait -n; done
done < <(
  curl --fail --silent --show-error "$TREE?recursive=1" \
    | jq -r '.tree[] | select(.type == "blob") | .path
             | select(. == "LICENSE"
                      or . == "franka_emika_panda/LICENSE"
                      or . == "franka_emika_panda/README.md"
                      or . == "franka_emika_panda/panda_nohand.xml"
                      or test("^franka_emika_panda/assets/link"))'
)
wait

printf 'MuJoCo Menagerie %s\n' "$REVISION" > "$temporary_checkout/SOURCE"
test -f "$temporary_checkout/LICENSE"
test -f "$temporary_checkout/franka_emika_panda/panda_nohand.xml"
test -f "$temporary_checkout/franka_emika_panda/assets/link0.stl"
mkdir -p "$(dirname "$DESTINATION")"
mv "$temporary_checkout" "$DESTINATION"
