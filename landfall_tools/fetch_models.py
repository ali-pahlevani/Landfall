# Copyright 2026 Ali Pahlevani
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Download the learned models of models.yaml and check their SHA-256 (scripts/fetch_models.sh).

    fetch_models.py [--models-dir DIR] [--parity] [FILE ...]

DIR: --models-dir, else $LANDFALL_MODELS_DIR, else ~/.local/share/landfall/models (as the
relocalizer looks them up). FILE: model files of models.yaml (default: all). --parity also
fetches the parity references of the C++ tests. A file already present with the right SHA-256
is kept; nothing is installed unless its checksum matches.
"""

import argparse
import hashlib
import os
import pathlib
import sys
import urllib.request

import yaml

MANIFEST = pathlib.Path(__file__).resolve().parent / 'models.yaml'


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def fetch(url, expected, target):
    if target.is_file() and sha256(target) == expected:
        print(f'{target.name}: present')
        return
    partial = target.with_name(target.name + '.part')
    print(f'{target.name}: downloading {url}')
    with urllib.request.urlopen(url, timeout=60) as response, open(partial, 'wb') as out:
        while True:
            block = response.read(1 << 20)
            if not block:
                break
            out.write(block)
    actual = sha256(partial)
    if actual != expected:
        partial.unlink()
        raise SystemExit(f'{target.name}: SHA-256 {actual}, expected {expected}; not installed')
    partial.replace(target)
    print(f'{target.name}: ok')


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--models-dir', type=pathlib.Path)
    parser.add_argument('--parity', action='store_true')
    parser.add_argument('files', nargs='*')
    args = parser.parse_args()

    manifest = yaml.safe_load(MANIFEST.read_text())
    models = {m['file']: m for m in manifest['models']}
    wanted = args.files or list(models)
    unknown = [f for f in wanted if f not in models]
    if unknown:
        raise SystemExit(f'not in models.yaml: {", ".join(unknown)} (known: {", ".join(models)})')
    directory = args.models_dir or pathlib.Path(
        os.environ.get('LANDFALL_MODELS_DIR') or
        pathlib.Path.home() / '.local/share/landfall/models')
    directory.mkdir(parents=True, exist_ok=True)

    missing = []
    for name in wanted:
        model = models[name]
        files = [(model['file'], model.get('url'), model.get('sha256'))]
        if args.parity:
            files.append((model['parity'], None, model.get('parity_sha256')))
        for file, url, expected in files:
            if not expected:
                missing.append(file)
                continue
            fetch(url or f'{manifest["release"]}/{file}', expected, directory / file)
    if missing:
        print(f'not published yet (docs/TODO.md T-007): {", ".join(missing)}', file=sys.stderr)
        return 1
    print(f'models in {directory}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
