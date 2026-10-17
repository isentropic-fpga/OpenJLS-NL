#!/usr/bin/env python3
"""Verification matrix -> HTML table for the NL report landing page.

Wraps OpenJLS/Verification/OSVVM/matrix_html.py: links point to the OpenJLS-NL
repo, links into the OpenJLS submodule point to the upstream repo, and the ASCII
"ok" status becomes the check mark upstream expects.
Usage: matrix_html.py <verification_requirements.md>
"""
import builtins
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "OpenJLS", "Verification", "OSVVM"))
import matrix_html as up  # noqa: E402

UPSTREAM = up.REPO
up.REPO = "https://github.com/isentropic-fpga/OpenJLS-NL/blob/main/"
up.ROOT = ROOT
_cell = up.cell


def cell(text, base):
    def sub(m):
        url = m.group(2)
        if not re.match(r"[a-z]+://", url):
            path = os.path.normpath(os.path.join(base, url))
            if path == "OpenJLS" or path.startswith("OpenJLS/"):
                url = UPSTREAM + path[len("OpenJLS/"):]
        return f"[{m.group(1)}]({url})"
    return _cell(re.sub(r"\[([^\]]*)\]\(([^)]*)\)", sub, text), base)


def ascii_open(*args, **kwargs):
    text = builtins.open(*args, **kwargs).read()
    return io.StringIO(re.sub(r"(?<=\| )ok(?= \|)", "\u2713", text))


up.cell = cell
up.open = ascii_open

if __name__ == "__main__":
    up.main()
