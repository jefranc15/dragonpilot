#!/usr/bin/env bash
set -euo pipefail

REPO="${REPO:-/data/openpilot}"
REMOTE="${REMOTE:-origin}"
BASE_BRANCH="${BASE_BRANCH:-dnga-v25c-test}"
TARGET_BRANCH="${TARGET_BRANCH:-nightly}"

STAMP="$(date +%Y%m%d_%H%M%S)"
WORKTREE="/tmp/dragonpilot-${TARGET_BRANCH}-${STAMP}"
PATCH="/tmp/${BASE_BRANCH}-to-device-${STAMP}.patch"
UNTRACKED_LIST="/tmp/${BASE_BRANCH}-untracked-${STAMP}.txt"
SKIPPED_LIST="/tmp/${BASE_BRANCH}-skipped-${STAMP}.txt"

cleanup() {
  if [ -d "$WORKTREE" ]; then
    git -C "$REPO" worktree remove --force "$WORKTREE" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

cd "$REPO"

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "ERROR: $REPO is not a Git repository."
  exit 1
fi

REMOTE_URL="$(git remote get-url "$REMOTE")"
echo "Repository: $REMOTE_URL"
echo "Base:       $REMOTE/$BASE_BRANCH"
echo "Target:     $TARGET_BRANCH"
echo

git fetch "$REMOTE" --prune

if ! git show-ref --verify --quiet "refs/remotes/$REMOTE/$BASE_BRANCH"; then
  echo "ERROR: Remote base branch does not exist:"
  echo "  $REMOTE/$BASE_BRANCH"
  echo
  echo "Available remote branches containing 'dnga' or 'v25':"
  git branch -r | grep -Ei 'dnga|v25' || true
  exit 1
fi

BASE_REF="$REMOTE/$BASE_BRANCH"

# Capture every tracked difference between the current comma checkout and the
# selected base branch. This includes:
# - commits present on the current checkout but absent from the base
# - staged changes
# - unstaged changes
# - added/deleted/renamed tracked files
git diff --binary --full-index "$BASE_REF" -- . > "$PATCH"

# Capture non-ignored untracked files too, but exclude runtime/generated data.
# Source/config files such as new .py, .h, .dbc, .json and scripts are included.
: > "$UNTRACKED_LIST"
: > "$SKIPPED_LIST"

while IFS= read -r -d '' file; do
  case "$file" in
    .git/*|*/.git/*|\
    *__pycache__/*|*.pyc|*.pyo|\
    dnga_logs/*|*/dnga_logs/*|logs/*|*/logs/*|log/*|*/log/*|\
    *.csv|*.bz2|*.rlog|*.hevc|*.h264|*.mp4|*.mov|*.avi|\
    *.zip|*.tar|*.tgz|*.7z|*.gz|*.xz|\
    *.log|*.tmp|*.swp|*.swo)
      printf '%s\n' "$file" >> "$SKIPPED_LIST"
      continue
      ;;
  esac

  # Avoid accidentally committing models, recordings, or other large binaries.
  size="$(wc -c < "$file" 2>/dev/null || echo 0)"
  if [ "$size" -gt 10485760 ]; then
    printf '%s\n' "$file" >> "$SKIPPED_LIST"
    continue
  fi

  printf '%s\n' "$file" >> "$UNTRACKED_LIST"
done < <(git ls-files --others --exclude-standard -z)

echo "Tracked differences versus $BASE_REF:"
git diff --stat "$BASE_REF" -- . || true
echo

if [ -s "$UNTRACKED_LIST" ]; then
  echo "New non-ignored files that will also be included:"
  sed 's/^/  + /' "$UNTRACKED_LIST"
  echo
fi

if [ -s "$SKIPPED_LIST" ]; then
  echo "Runtime/generated or large untracked files excluded:"
  sed 's/^/  - /' "$SKIPPED_LIST"
  echo
fi

# Build the target branch separately so the running comma checkout, current
# branch, index, and local modifications remain untouched.
git worktree add --detach "$WORKTREE" "$BASE_REF"
git -C "$WORKTREE" switch -C "$TARGET_BRANCH"

if [ -s "$PATCH" ]; then
  git -C "$WORKTREE" apply --binary --index "$PATCH"
fi

if [ -s "$UNTRACKED_LIST" ]; then
  while IFS= read -r file; do
    mkdir -p "$WORKTREE/$(dirname "$file")"
    cp -p "$REPO/$file" "$WORKTREE/$file"
    git -C "$WORKTREE" add -- "$file"
  done < "$UNTRACKED_LIST"
fi

echo "Staged nightly changes:"
git -C "$WORKTREE" status --short
echo

if git -C "$WORKTREE" diff --cached --quiet; then
  echo "No differences found versus $BASE_REF; nothing to upload."
  exit 0
fi

# Compile changed Python files only. This checks syntax using the same Python
# installed on the comma device.
mapfile -d '' CHANGED_PY < <(
  git -C "$WORKTREE" diff --cached --name-only --diff-filter=ACMR -z -- '*.py'
)

if [ "${#CHANGED_PY[@]}" -gt 0 ]; then
  echo "Compiling changed Python files..."
  for file in "${CHANGED_PY[@]}"; do
    PYTHONPATH="$WORKTREE:$WORKTREE/cereal" \
      python -m py_compile "$WORKTREE/$file"
  done
fi

CURRENT_NAME="$(git -C "$REPO" config user.name || true)"
CURRENT_EMAIL="$(git -C "$REPO" config user.email || true)"
git -C "$WORKTREE" config user.name "${CURRENT_NAME:-John Franco}"
git -C "$WORKTREE" config user.email \
  "${CURRENT_EMAIL:-jefranc15@users.noreply.github.com}"

git -C "$WORKTREE" commit \
  -m "DNGA: upload current device changes as V2.5S nightly"

# nightly is intentionally reconstructed as:
#   origin/dnga-v25c-test + every current device difference.
# --force-with-lease protects against overwriting an unexpected remote update.
if git ls-remote --exit-code --heads "$REMOTE" "$TARGET_BRANCH" \
    >/dev/null 2>&1; then
  echo "Remote $TARGET_BRANCH exists; updating with force-with-lease."
  git -C "$WORKTREE" push --force-with-lease \
    "$REMOTE" "$TARGET_BRANCH:$TARGET_BRANCH"
else
  git -C "$WORKTREE" push -u "$REMOTE" \
    "$TARGET_BRANCH:$TARGET_BRANCH"
fi

COMMIT="$(git -C "$WORKTREE" rev-parse HEAD)"
echo
echo "UPLOAD COMPLETE"
echo "Repository: $REMOTE_URL"
echo "Branch:     $TARGET_BRANCH"
echo "Base:       $BASE_REF"
echo "Commit:     $COMMIT"
echo
echo "The active comma checkout was not switched, reset, or cleaned."
