#!/usr/bin/env python3
"""Create a source-only, checksummed bundle. Never connects to or runs a robot."""
import argparse
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile
import time

PACKAGES = ('omx_turtlebot3_control_cpp', 'omx_turtlebot3_runtime',
            'omx_turtlebot3_policy', 'aruco_eef_tracker', 'DynamixelSDK', 'mp_control', 'turtlebot3_control')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[2]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    run_id = time.strftime('%Y%m%d_%H%M%S')+'_'+str(time.time_ns())[-9:]
    destination = args.output_dir/f'omx_v6_cpp_source_{run_id}.tar.gz'
    manifest = dict(schema_version=1, created_unix_ns=time.time_ns(),
        checkout_commit=subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip(),
        includes_uncommitted_workspace_changes=True,
        reference_commit='795d0ee0cac048b4289238cb4bdc16ed6a32fd35',
        architecture='source_only_rebuild_on_target', files=[])
    with destination.open('xb') as output, tarfile.open(fileobj=output, mode='w:gz') as archive:
        for package in PACKAGES:
            for path in sorted((source/package).rglob('*')):
                if not path.is_file() or any(p in ('.git', '__pycache__', '.pytest_cache', '.ruff_cache') for p in path.relative_to(source).parts):
                    continue
                if path.is_symlink():
                    raise RuntimeError(f'Unexpected source symlink: {path}')
                name = 'src/'+path.relative_to(source).as_posix()
                content = path.read_bytes()
                manifest['files'].append(dict(path=name, size=len(content), sha256=hashlib.sha256(content).hexdigest()))
                info = archive.gettarinfo(str(path), arcname=name)
                archive.addfile(info, io.BytesIO(content))
        metadata = json.dumps(manifest, indent=2).encode()+b'\n'
        info = tarfile.TarInfo('bundle_manifest.json')
        info.size, info.mtime = len(metadata), int(time.time())
        archive.addfile(info, io.BytesIO(metadata))
    # Verify archive contents against the explicit list before calling it ready.
    with tarfile.open(destination, 'r:gz') as archive:
        files = {m.name: m for m in archive.getmembers() if m.isfile()}
        if set(files) != {v['path'] for v in manifest['files']} | {'bundle_manifest.json'}:
            raise RuntimeError('Bundle file list mismatch')
        for entry in manifest['files']:
            content = archive.extractfile(entry['path']).read()
            if len(content) != entry['size'] or hashlib.sha256(content).hexdigest() != entry['sha256']:
                raise RuntimeError('Bundle checksum mismatch: '+entry['path'])
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    destination.with_suffix(destination.suffix+'.sha256').write_text(digest+'  '+destination.name+'\n')
    print(json.dumps(dict(bundle=str(destination), sha256=digest,
                         source_files=len(manifest['files']), verified=True), indent=2))


if __name__ == '__main__':
    main()
