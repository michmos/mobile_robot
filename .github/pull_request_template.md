## Summary
<!-- What this PR changes and why. Name the issue it resolves with "Fixes #N". PRs target `dev` but the default branch is `main`, so "Fixes #N" does not auto-close the issue: close it by hand after merge. -->

## Depends on
<!-- PRs that must be merged before this one, or the branch this PR is stacked on. Check that a stacked-on PR is still open: a PR based on an already merged branch never reaches `dev`. Write "None" if it is independent. -->

## Test coverage
<!-- Tests added or changed and what they cover, per test file/case. Name what is NOT covered and why. Do not weaken or skip tests to make them pass. Write "No test changes" with a reason if none. -->

## Critical changes
<!-- Anything risky or needing reviewer attention: breaking changes, wire protocol changes (firmware and hardware plugin must be updated together), parameter/launch/interface changes, Docker image or CI changes (CI only sees them after merge to dev and image.yml has run). Write "None" if there are none. -->

## Decisions made
<!-- Design choices, alternatives considered and why they were rejected. Write "None" if there were no real choices. -->

## Proof of work
<!-- The exact commands run and their results (e.g. colcon build/test summary counts, repeated test runs, firmware compile), and where they ran (local Docker image or CI). Paste the relevant output summary, not the full log. -->

## Open items
<!-- Known gaps, follow-up work, issues opened while working on this, and cleanup after merge (e.g. issues to close by hand, branches/worktrees to delete). Write "None" if there are none. -->

## Non-technical summary
<!-- 1-3 plain-language sentences for someone without a programming background: what changed for the robot or the project, and why it matters. -->

## Checklist
<!-- Tick what applies; strike through or explain items that don't apply. -->
- [ ] Branch is based on current `dev` and the PR targets `dev`
- [ ] Builds and all tests pass (locally per `TESTING.md` and in CI)
- [ ] New or changed behavior is covered by tests; no test was weakened, skipped or loosened to pass
- [ ] `clang-format -i` run on all touched C++ files
- [ ] New dependencies added to `package.xml` and, if needed in CI, `docker/Dockerfile.ros2`
- [ ] Wire protocol changes: firmware and hardware plugin updated together
- [ ] `OVERVIEW.md` updated for bigger changes
- [ ] Every section above is filled in
