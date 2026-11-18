"""The strict YAML loader the community linter uses (R138).

`yaml.SafeLoader` with anchors and aliases refused. Safe loading stops a file from building
Python objects; it does nothing about one that **expands**, and this is the measurement that
says where the guard belongs:

- 229 bytes of nested aliases is 16,369 nodes at twelve levels, and each further level
  doubles it;
- but `yaml.safe_load` itself is cheap - PyYAML hands an alias the node it already composed,
  so the loaded object is a graph with shared references, not a copy per alias. The blow-up
  happens in whatever **walks** the result, which for a room from someone else is the
  linter and the validators behind it;
- and the loaders cannot refuse aliases, because `yaml.safe_dump` emits an anchor whenever
  the same object appears twice. A room dict whose two parts share one `pos` list
  round-trips through an alias; refusing them broke `tests/test_cube.py`, which is how this
  module ended up here instead of in `room.py`.

So: strict on the path that takes a file from someone else (`community.check_raw`), plain
`safe_load` everywhere the repository reads its own files.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


class NoAliasLoader(yaml.SafeLoader):
    """SafeLoader that refuses anchors and aliases (see the module docstring)."""

    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise yaml.composer.ComposerError(
                None, None, "YAML aliases (*name) are not allowed in shared rooms",
                self.peek_event().start_mark)
        return super().compose_node(parent, index)


def loads(text: str) -> Any:
    """Parse YAML text with the strict loader."""
    return yaml.load(text, Loader=NoAliasLoader)


def load(path: str | Path) -> Any:
    """Read and parse a YAML file with the strict loader."""
    return loads(Path(path).read_text())
