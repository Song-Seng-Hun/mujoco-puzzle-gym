"""R51: the docs say where each body's assets came from, and do not overstate one.

The second body's walking policy is a community checkpoint, not a vendor release. That
distinction is easy to lose in a later edit - "the official Go2 policy" reads perfectly
well and would be a false claim about someone else's work - so it is a test, not a
convention. The test does two things: it checks the disclaimer is present in all three
languages, and it checks the word "official" is never used about that policy.
"""

import json
import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
#: (doc, the sentence that must be there, the word that must not describe the policy)
LANGS = {
    "docs/bodies.md": ("공식 배포가 아닙니다", "공식"),
    "docs/bodies.en.md": ("not an official Unitree release", "official"),
    "docs/bodies.ja.md": ("公式な配布ではありません", "公式"),
}
READMES = {
    "README.md": "공식 배포가 아니라 커뮤니티",
    "docs/README.en.md": "not an official Unitree release but a community checkpoint",
    "docs/README.ja.md": "公式な配布ではなく、コミュニティ",
}
LOCKS = ("src/mujoco_puzzle_gym/upstream.lock.json",
         "src/mujoco_puzzle_gym/upstream.go2.lock.json")


@pytest.mark.parametrize("doc", list(LANGS))
def test_the_policy_is_called_a_community_checkpoint(doc):
    assert LANGS[doc][0] in (ROOT / doc).read_text(), doc


@pytest.mark.parametrize("doc", list(READMES))
def test_the_readme_says_it_too(doc):
    assert READMES[doc] in (ROOT / doc).read_text(), doc


#: Words that mean "the walking policy" in each language. The word "official" may be used
#: about other things - that no official reach figure is published, for instance - so the
#: rule is scoped to lines that are about a policy.
POLICY_WORDS = {"docs/bodies.md": ("\uc815\ucc45",),
                "docs/bodies.en.md": ("policy", "checkpoint"),
                "docs/bodies.ja.md": ("\u30dd\u30ea\u30b7\u30fc", "\u30c1\u30a7\u30c3\u30af\u30dd\u30a4\u30f3\u30c8")}


@pytest.mark.parametrize("doc", list(LANGS))
def test_official_is_never_used_about_that_policy(doc):
    """A line that says the policy *is* official fails here, whichever way it is phrased.

    Scoped to lines about a policy on purpose: the same word is needed elsewhere to say
    that no official reach figure is published for the arm, and a test that forbade the
    word outright would have made that sentence unwriteable instead of catching a claim.
    """
    text, (denial, word) = (ROOT / doc).read_text(), LANGS[doc]
    stray = [line.strip() for line in text.splitlines()
             if word in line and denial not in line
             and any(w in line for w in POLICY_WORDS[doc])
             and "velstand" not in line]
    assert not stray, f"{doc}: {stray}"


@pytest.mark.parametrize("lock", LOCKS)
def test_every_pinned_source_carries_a_licence_and_an_attribution(lock):
    data = json.loads((ROOT / lock).read_text())
    licence = data["license"]
    assert licence.get("attribution"), lock
    assert any("licen" in k or k in ("meshes", "mjcf_and_code", "mjcf_and_meshes", "policies")
               for k in licence), lock
    for key, value in licence.items():
        if key == "attribution":
            continue
        assert re.search(r"Apache-2\.0|BSD-3-Clause|CC BY", value), (lock, key, value)


@pytest.mark.parametrize("lock", LOCKS)
def test_the_docs_name_the_revision_each_body_is_pinned_to(lock):
    """A revision in the lock but not in the docs is a provenance claim nobody can check."""
    data = json.loads((ROOT / lock).read_text())
    text = (ROOT / "docs/bodies.md").read_text()
    for source in data["sources"].values():
        ref = source.get("commit") or source.get("revision")
        assert ref[:7] in text, (lock, ref)
