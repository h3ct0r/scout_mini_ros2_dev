# MIT License

# Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from pathlib import Path

from ros2_unbag.core.routines.base import ExportRoutine, ExportMode, ExportMetadata


def setup_function(_):
    # Reset registries before each test
    ExportRoutine.reset_registry()


def test_catch_all_registration_and_queries(tmp_path: Path):
    calls = []

    @ExportRoutine.set_catch_all(["text/custom"], mode=ExportMode.MULTI_FILE)
    def export_multi(msg, path: Path, fmt: str, metadata: ExportMetadata):
        # Record a call and write a file to prove invocation
        calls.append(("multi", fmt, metadata.index))
        p = Path(str(path) + ".multi")
        p.write_text("multi")

    @ExportRoutine.set_catch_all(["text/custom"], mode=ExportMode.SINGLE_FILE)
    def export_single(msg, path: Path, fmt: str, metadata: ExportMetadata):
        calls.append(("single", fmt, metadata.index))
        p = Path(str(path) + ".out")
        p.write_text("ok")

    # Only the base format is advertised
    assert ExportRoutine.get_formats("any/msg") == ["text/custom"]

    # Default (no suffix) resolves to multi
    assert ExportRoutine.get_mode("any/msg", "text/custom") == ExportMode.MULTI_FILE

    # Explicit suffix resolves accordingly
    assert ExportRoutine.get_mode("any/msg", "text/custom@single_file") == ExportMode.SINGLE_FILE
    assert ExportRoutine.get_mode("any/msg", "text/custom@multi_file") == ExportMode.MULTI_FILE

    # Handler lookup works with and without suffixes
    multi_handler = ExportRoutine.get_handler("any/msg", "text/custom")
    single_handler = ExportRoutine.get_handler("any/msg", "text/custom@single_file")
    assert callable(multi_handler)
    assert callable(single_handler)

    # Invoke through handlers to ensure canonical fmt is passed into routines
    md1 = ExportMetadata(index=0, max_index=1)
    multi_handler(msg=object(), path=tmp_path / "file1", fmt="text/custom", metadata=md1, topic="/a")
    md2 = ExportMetadata(index=1, max_index=1)
    single_handler(msg=object(), path=tmp_path / "file2", fmt="text/custom@single_file", metadata=md2, topic="/b")

    assert calls == [
        ("multi", "text/custom", 0),
        ("single", "text/custom", 1),
    ]

    assert (tmp_path / "file1.multi").exists()
    assert (tmp_path / "file2.out").exists()


def test_single_mode_default_access(tmp_path: Path):
    ExportRoutine.reset_registry()
    calls = []

    @ExportRoutine.set_catch_all(["text/only"], mode=ExportMode.SINGLE_FILE)
    def export_single(msg, path: Path, fmt: str, metadata: ExportMetadata):
        calls.append(fmt)

    assert ExportRoutine.get_formats("any/msg") == ["text/only"]
    assert ExportRoutine.get_mode("any/msg", "text/only") == ExportMode.SINGLE_FILE
    assert ExportRoutine.get_mode("any/msg", "text/only@single_file") == ExportMode.SINGLE_FILE

    handler = ExportRoutine.get_handler("any/msg", "text/only")
    handler(msg=object(), path=tmp_path / "x", fmt="text/only", metadata=ExportMetadata(index=0, max_index=0))
    assert calls == ["text/only"]
