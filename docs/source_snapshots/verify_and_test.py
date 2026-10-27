"""Verify frozen source bytes and run the retained pure-module regressions."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

root = Path(__file__).resolve().parent
archive = root / 'property_inspection-20261027.tar.gz'
manifest = json.loads((root / 'property_inspection-20261027.manifest.json').read_text())
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive_sha256']
with tempfile.TemporaryDirectory(prefix='property-inspection-source-') as temp:
    with tarfile.open(archive) as source:
        source.extractall(temp, filter='data')
    package = Path(temp) / 'property_inspection'
    for entry in manifest['files']:
        assert hashlib.sha256((package / entry['path']).read_bytes()).hexdigest() == entry['sha256']
    suites = ['config_loader', 'inspection_executor', 'mission_reporter',
              'mission_runner', 'coverage_region', 'coverage_metrics']
    env = dict(os.environ, PYTHONPATH=str(package) + os.pathsep + os.environ.get('PYTHONPATH', ''))
    result = subprocess.run([sys.executable, '-m', 'pytest', '-q',
                             *[str(package / 'test' / f'test_{suite}.py') for suite in suites]],
                            env=env)
    raise SystemExit(result.returncode)
