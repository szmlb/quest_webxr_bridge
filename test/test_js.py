"""Run the browser logic unit tests (test/js/*.test.js) with Node.js."""

import os
import shutil
import subprocess

import pytest

JS_DIR = os.path.join(os.path.dirname(__file__), 'js')


@pytest.mark.skipif(shutil.which('node') is None, reason='node not installed')
def test_browser_logic():
    files = sorted(os.path.join(JS_DIR, f) for f in os.listdir(JS_DIR) if f.endswith('.test.js'))
    res = subprocess.run(['node', '--test'] + files, capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
