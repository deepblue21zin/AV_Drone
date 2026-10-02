#!/usr/bin/env bash
# Read-only inspection of the stopped AV_Drone sim container's writable layer.
# No deletion, truncation, container restart, chmod, or chown is performed.
# An administrator must run this if the Docker storage directory is inaccessible.
set -euo pipefail

inspect_root=/var/lib/docker/overlay2/d2fbedb258a011d7c81853e2156333da51db2aa646a78c554167819004d03ae0/diff
inspect_sort_dir=/home3/deepblue/work/AV_Drone/artifacts/tmp_recovery_20260911.jhPJbR
export LC_ALL=C

if [[ ! -r "$inspect_root" || ! -x "$inspect_root" ]]; then
  printf 'Cannot read the sim storage directory: %s\n' "$inspect_root" >&2
  printf 'Ask an administrator to run this read-only script. Do not change Docker directory permissions.\n' >&2
  exit 2
fi
if [[ ! -d "$inspect_sort_dir" || ! -w "$inspect_sort_dir" ]]; then
  printf 'The home3 sort workspace is not writable: %s\n' "$inspect_sort_dir" >&2
  exit 2
fi

printf 'Container: av_drone-sim-1\nStorage directory: %s\n' "$inspect_root"
printf '\nAllocated disk bytes (du; excludes filesystem-wide metadata):\n'
du -x -B1 -s "$inspect_root"

printf '\nLargest directories: allocated_bytes<TAB>path\n'
du -x -B1 --max-depth=6 "$inspect_root" |
  sort -nr -S 8M -T "$inspect_sort_dir" |
  awk 'NR <= 40'

printf '\nLargest files by logical size: logical_bytes<TAB>allocated_512_byte_blocks<TAB>path\n'
find "$inspect_root" -xdev -type f -printf '%s\t%b\t%p\n' |
  sort -nr -S 8M -T "$inspect_sort_dir" |
  awk 'NR <= 50'

printf '\nAllocated blocks are in units of 512 bytes; logical size can exceed allocated space for sparse files.\n'
printf 'No sim files or containers were modified.\n'
