#!/usr/bin/env python3
"""Apply/check Ev4sio's PhysX symbol isolation after an upstream update.

Public C entry points need an explicit prefix: a C++ namespace does not
change their linker names. Internal C exports are handled separately in
ev4sio_symbols.json so ordinary C++ methods keep their public API names.
Only Git-tracked text is processed; generated build products are excluded.
"""
import argparse
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).with_name('ev4sio_symbols.json')
NAMESPACES = ('physx', 'Sc', 'Gu', 'Cm', 'Dy', 'Bp', 'Sq', 'IG', 'Ext')
CODE_SUFFIXES = {'.h', '.hpp', '.cpp', '.c', '.cu', '.cuh', '.inl', '.hlsl', '.glsl', '.natvis', '.rc'}
SDK_SUFFIXES = CODE_SUFFIXES | {'.py', '.cmake', '.txt', '.xml', '.def', '.map', '.md', '.html'}
COMMENTS = re.compile(r'//[^\n]*|/\*.*?\*/', re.S)
C_EXPORT = re.compile(r'\bPX_C_EXPORT\b[^;{}]*?\b([A-Za-z_]\w*)\s*\(')


def source_files():
    paths = subprocess.check_output(['git', '-C', str(ROOT), 'ls-files', '-z']).decode().split('\0')
    for name in paths:
        path = ROOT / name
        if not path.is_file() or path in (Path(__file__).resolve(), MANIFEST):
            continue
        suffixes = SDK_SUFFIXES if name.startswith(('physx/', 'blast/', 'flow/')) else CODE_SUFFIXES
        if path.suffix.lower() in suffixes:
            yield name, path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Report remaining unprefixed SDK symbols without changing files.')
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text(encoding='utf-8')) if MANIFEST.exists() else {'c_functions': [], 'internal_cpp_linkage': {}}
    symbols = set(manifest['c_functions'])
    files = []
    for name, path in source_files():
        raw = path.read_bytes()
        if b'\0' in raw:
            continue
        text = raw.decode('utf-8', errors='surrogateescape')
        files.append((name, path, raw, text))
        if name.startswith(('physx/include/', 'physx/source/')):
            declarations = re.sub(r'^\s*#.*$', '', COMMENTS.sub('', text), flags=re.M)
            for match in C_EXPORT.finditer(declarations):
                symbol = match.group(1)
                if symbol.startswith('ev4sio_'):
                    symbol = symbol[7:]
                if symbol.startswith('Px'):
                    symbols.add(symbol)
                elif symbol not in manifest['internal_cpp_linkage'].get(name, []):
                    raise SystemExit('Review internal C entry point before prefixing: %s: %s' % (name, symbol))

    # The version token and the PDF's C++ global entry points also form part of
    # the fork's source-level interface, even when absent from PX_C_EXPORT.
    symbols.add('PX_PHYSICS_VERSION')
    if not symbols:
        raise SystemExit('No PhysX symbols found.')
    token = re.compile(r'\b(?:' + '|'.join(re.escape(s) for s in sorted(symbols, key=len, reverse=True)) + r')\b')
    namespaces = '|'.join(NAMESPACES)
    declaration = re.compile(r'\b(namespace\s+)(' + namespaces + r')\b')
    qualifier = re.compile(r'\b(' + namespaces + r')(?=\s*::)')
    qualified_tail = re.compile(r'(::\s*)(' + namespaces + r')\b')
    alias = re.compile(r'(\bnamespace\s+\w+\s*=\s*(?:::)?)(physx)(\s*;)')
    changed = []
    for name, path, raw, text in files:
        updated = declaration.sub(lambda m: m[1] + 'ev4sio_' + m[2], text)
        updated = qualifier.sub(lambda m: 'ev4sio_' + m[1], updated)
        updated = qualified_tail.sub(lambda m: m[1] + 'ev4sio_' + m[2], updated)
        updated = alias.sub(lambda m: m[1] + 'ev4sio_' + m[2] + m[3], updated)
        updated = token.sub(lambda m: 'ev4sio_' + m[0], updated)
        # These internal declarations are C++-only interfaces. Removing the C
        # linkage lets the renamed namespace isolate them without renaming
        # unrelated PxPhysics methods such as createTriangleMesh().
        for symbol in manifest['internal_cpp_linkage'].get(name, []):
            pattern = re.compile(r'\bPX_C_EXPORT\s+(?=[^;{}]*?\b' + re.escape(symbol) + r'\s*\()')
            updated = pattern.sub('', updated)
        if updated != text:
            before, after = text.splitlines(keepends=True), updated.splitlines(keepends=True)
            if len(before) == len(after):
                updated = ''.join(re.sub(r'[ \t]+(?=\r?\n|$)', '', new) if old != new else new
                                  for old, new in zip(before, after))
            changed.append(name)
            if not args.check:
                path.write_bytes(updated.encode('utf-8', errors='surrogateescape'))

    print('%d C symbols; %d files %s' % (len(symbols) - 1, len(changed), 'need prefixing' if args.check else 'updated'))
    if args.check:
        for name in changed[:30]:
            print(name)
        raise SystemExit(bool(changed))
    manifest['c_functions'] = sorted(symbols - {'PX_PHYSICS_VERSION'})
    MANIFEST.write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
