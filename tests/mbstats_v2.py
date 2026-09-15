#!/usr/bin/env python3
"""Parser for libvc1 macroblock-stats-v2 scoped CSV traces."""
import csv


def read_macroblock_stats(path):
    config = {}
    data_lines = []
    with open(path, newline='') as f:
        for raw in f:
            if raw.startswith('#'):
                text = raw[1:].strip()
                if '=' in text:
                    k, v = text.split('=', 1)
                    config[k.strip()] = v.strip()
                continue
            if raw.strip():
                data_lines.append(raw)
    if not data_lines:
        raise RuntimeError(f'{path}: no CSV rows')
    rows = list(csv.reader(data_lines))
    header = rows[0]
    if not header or header[0] != 'record':
        raise RuntimeError(f'{path}: missing scoped record header')
    frames = []
    macroblocks = []
    current = None
    for vals in rows[1:]:
        if not vals:
            continue
        if len(vals) > len(header):
            raise RuntimeError(f'{path}: row wider than header ({len(vals)} > {len(header)})')
        row = dict(zip(header, vals))
        kind = row.get('record', '')
        if kind == 'frame':
            current = row
            frames.append(row)
        elif kind == 'mb':
            if current is None:
                raise RuntimeError(f'{path}: macroblock row precedes first frame row')
            row['_frame'] = current
            macroblocks.append(row)
        else:
            raise RuntimeError(f'{path}: unknown record type {kind!r}')
    return config, header, frames, macroblocks


def frame_value(mb, key):
    return mb['_frame'][key]
