#!/usr/bin/env python3
"""Export the pinned JD-AMR baseline without opening hardware or starting ROS."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def prepare(source, output, contract):
    """Export committed files only; reject an existing destination."""
    if output.exists():
        raise ValueError(f'output already exists: {output}')
    revision = contract['source']['commit']
    subprocess.run(
        ['git', '-C', str(source), 'cat-file', '-e', f'{revision}^{{commit}}'],
        check=True)
    # git archive excludes local edits and avoids changing the source checkout.
    archive = subprocess.run(
        ['git', '-C', str(source), 'archive', '--format=tar', revision],
        check=True, stdout=subprocess.PIPE).stdout
    output.mkdir(parents=True)
    baseline = output / 'baseline'
    baseline.mkdir()
    subprocess.run(['tar', '-xf', '-', '-C', str(baseline)],
                   input=archive, check=True)
    files = {
        str(path.relative_to(baseline)): hashlib.sha256(
            path.read_bytes()).hexdigest()
        for path in sorted(baseline.rglob('*'))
        if path.is_file() and not path.is_symlink()
    }
    report = {
        'status': 'BASELINE_EXPORTED_NOT_PHYSICAL_VALIDATION',
        'source': contract['source'],
        'files_sha256': files,
        'target_design': contract['target_design'],
        'physical_motion_enabled': False,
        'required_before_physical_navigation': [
            'confirm motor/controller reuse and actual lidar model',
            'verify servo IDs, encoder signs and firmware watchdog',
            'measure wheel geometry, effective separation and sensor TF',
            'replace JD-AMR URDF with measured target robot description',
            'update both costmaps and collision monitor for transport envelope',
            'measure stopping distance at initial low speed',
            'validate map/keepout and new station clearance',
        ],
    }
    (output / 'manifest.json').write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    contract = json.loads(
        (ROOT / 'config/navigation/jdamr_migration.json').read_text())
    result = prepare(args.source.resolve(), args.output.resolve(), contract)
    print(json.dumps({'status': result['status'],
                      'commit': result['source']['commit'],
                      'file_count': len(result['files_sha256'])}))


if __name__ == '__main__':
    main()
