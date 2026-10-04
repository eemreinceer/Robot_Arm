#!/usr/bin/env bash

set -euo pipefail

readonly EXPECTED_NAME="Emre Inceer"
readonly EXPECTED_EMAIL="inceer22@gmail.com"
revision_range="${1:-HEAD}"

if ! commit_list="$(git rev-list --reverse "$revision_range")"; then
  printf 'ERROR: invalid revision range: %s\n' "$revision_range" >&2
  exit 2
fi

if [ -z "$commit_list" ]; then
  printf 'commit identity PASS: no commits in %s\n' "$revision_range"
  exit 0
fi

failed=0
while IFS= read -r commit; do
  record="$(git show -s --format='%H%x1f%an%x1f%ae%x1f%cn%x1f%ce' "$commit")"
  IFS=$'\x1f' read -r sha author_name author_email committer_name committer_email \
    <<<"$record"

  if [ "$author_name" != "$EXPECTED_NAME" ] || [ "$author_email" != "$EXPECTED_EMAIL" ]; then
    printf 'FAIL %s: author is %s <%s>\n' "$sha" "$author_name" "$author_email" >&2
    failed=1
  fi
  if [ "$committer_name" != "$EXPECTED_NAME" ] || [ "$committer_email" != "$EXPECTED_EMAIL" ]; then
    printf 'FAIL %s: committer is %s <%s>\n' \
      "$sha" "$committer_name" "$committer_email" >&2
    failed=1
  fi
  if git show -s --format='%B' "$commit" | grep -Eiq '^Co-authored-by:'; then
    printf 'FAIL %s: Co-authored-by trailer is not allowed in public history\n' \
      "$sha" >&2
    failed=1
  fi
done <<<"$commit_list"

if [ "$failed" -ne 0 ]; then
  exit 1
fi

printf 'commit identity PASS: %s\n' "$revision_range"
