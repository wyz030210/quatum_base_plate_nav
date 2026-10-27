# Frozen property_inspection source

`property_inspection-20261027.tar.gz` is the complete reproducible source tree,
including the untracked coverage modules and ignored source tests. The initial
archive was lost with the interrupted temporary worktree; this archive was
recaptured from the unchanged original source during recovery. The manifest
records this recovery, source worktree changes, source
commit, target actual baseline/status, relevant Python/ROS package versions and
SHA256 of every archived file and the archive. Generated caches and local
`.claude` settings are explicitly excluded and listed in the manifest.

Extract into a temporary directory, validate the manifest, then run the six
existing pure-module suites. No source checkout or hardware is needed:

```bash
python3 docs/source_snapshots/verify_and_test.py
```

The snapshot retains simulation coordinates, launches and behavior trees as
provenance only. They are not installed as target vehicle execution defaults.
