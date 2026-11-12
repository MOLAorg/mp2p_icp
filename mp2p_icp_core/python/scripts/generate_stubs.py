#!/usr/bin/env python3
"""Regenerates the type stubs of the extension module, `mp2p_icp/_core.pyi`.

Run it after changing the C++ bindings, with the built `mp2p_icp` package and
the `mrpt` Python modules importable (e.g. after `source install/setup.bash`):

    pip install pybind11-stubgen==3.0.0
    python3 mp2p_icp_core/python/scripts/generate_stubs.py

Stubs depend on the pybind11 version: generate them with the one of the
mrpt3 wheel (mrpt.sdk.PYBIND11_VERSION), e.g. with the mp2p-icp wheel
installed (see packaging/pip/README-dev.md), not with an older system one.
"""

import importlib
import os
import re
import subprocess
import sys
import tempfile

PKG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mp2p_icp")


def use_public_names(text: str) -> str:
    """'mrpt.X._bindings.Name' into 'mrpt.X.Name', if the package exports it."""

    def repl(m):
        mod = importlib.import_module("mrpt." + m.group(1))
        return f"mrpt.{m.group(1)}.{m.group(2)}" if hasattr(mod, m.group(2)) else m.group(0)

    text = re.sub(r"\bmrpt\.(\w+)\._bindings\.(\w+)\b", repl, text)
    for mod in set(re.findall(r"^import mrpt\.(\w+)\._bindings$", text, re.MULTILINE)):
        if f"mrpt.{mod}._bindings." not in text:
            text = re.sub(
                rf"^import mrpt\.{mod}\._bindings$", f"import mrpt.{mod}", text, flags=re.M
            )
    return text


def main() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run(
            [
                sys.executable,
                "-m",
                "pybind11_stubgen",
                "--numpy-array-remove-parameters",
                "-o",
                tmp,
                "mp2p_icp._core",
            ],
            check=True,
            env=dict(os.environ, PYTHONWARNINGS="ignore"),
        )
        with open(os.path.join(tmp, "mp2p_icp", "_core.pyi"), encoding="utf-8") as f:
            text = use_public_names(f.read())
    with open(os.path.join(PKG_DIR, "_core.pyi"), "w", encoding="utf-8") as f:
        f.write(text)
    open(os.path.join(PKG_DIR, "py.typed"), "w").close()
    print("Updated:", os.path.normpath(os.path.join(PKG_DIR, "_core.pyi")))


if __name__ == "__main__":
    main()
