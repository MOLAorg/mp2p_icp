"""scikit-build-core metadata provider of the wheel "dependencies".

The C++ code in the wheel links to the MRPT libraries of the mrpt3 wheel it
was built against, so the mrpt3 requirement follows the mrpt3 version in the
build environment: the same minor series, and at least that patch version
(e.g. "mrpt3>=3.6.2,<3.7"). Other requirements come from the "requires"
setting (see pyproject.toml).
"""

from __future__ import annotations

import re
from importlib.metadata import version


def dynamic_metadata(field, settings):
    if field != "dependencies":
        raise ValueError(f"This provider only fills 'dependencies', not '{field}'")
    mrpt3 = version("mrpt3")
    m = re.match(r"(\d+)\.(\d+)\.(\d+)", mrpt3)
    if not m:
        raise ValueError(f"Unexpected mrpt3 version: '{mrpt3}'")
    major, minor, patch = (int(x) for x in m.groups())
    return [*settings.get("requires", []), f"mrpt3>={major}.{minor}.{patch},<{major}.{minor + 1}"]
