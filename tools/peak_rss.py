"""Run a command and report the peak resident set size of the processes it spawned.

R50: CI prints this per shard, so the suite's memory is a number in the log rather than
a guess, and the run that killed a 16 GB runner mid-suite with no test failing would show
up as a figure instead of a shutdown signal.

What it measures, exactly: `ru_maxrss` over all reaped children, which is the high-water
mark of **the single largest process**, not the sum of the processes alive at one moment.
The suite forks worker processes, so the sum at peak is higher than this number - the
figure is a floor on the shard's footprint, not the footprint. It is still the right thing
to print: it is cheap, it needs no sampling thread, and a shard whose largest process
alone approaches the runner's memory is already in trouble.

Units differ by platform: Linux reports kilobytes, macOS bytes. Both are handled, because
CI is Linux and the measurements on this project were taken on macOS.

    python tools/peak_rss.py -- pytest -q --shard 0/2
"""

from __future__ import annotations

import resource
import subprocess
import sys

#: `ru_maxrss` unit per platform: Linux gives KiB, macOS (and the BSDs) bytes.
TO_MIB = 1 / 1024 if sys.platform.startswith("linux") else 1 / (1024 * 1024)


def main(argv: list[str]) -> int:
    if argv[:1] == ["--"]:
        argv = argv[1:]
    if not argv:
        print(__doc__.strip().splitlines()[-1].strip(), file=sys.stderr)
        return 2
    code = subprocess.call(argv)
    peak = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss * TO_MIB
    # One line, greppable, on stdout even when the command failed: a shard that died of
    # memory is exactly the case where the number matters most.
    print(f"peak-rss-mib {peak:.0f} (largest single process; exit {code})")
    return code


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
