"""Read gzip, plain JSONL, and interrupted .part logs by content, not suffix.

Only complete JSON records preceding a gzip error are yielded; corrupted tails
are never guessed. Pass a dict as report to inspect partial/error counts.
"""
import gzip
import json
import zlib


def iter_records(path, report=None):
    report = report if report is not None else {}
    report.update(records=0, invalid_lines=0, incomplete_lines=0, error=None)
    with open(path, 'rb') as probe:
        compressed = probe.read(2) == b'\x1f\x8b'
    report['encoding'] = 'gzip' if compressed else 'plain_jsonl'
    opener = gzip.open if compressed else open
    try:
        with opener(path, 'rb') as stream:
            for line in stream:
                if not line.endswith(b'\n'):
                    report['incomplete_lines'] += 1
                    continue
                try:
                    record = json.loads(line)
                except (ValueError, UnicodeError):
                    report['invalid_lines'] += 1
                    continue
                if not isinstance(record, dict):
                    report['invalid_lines'] += 1
                    continue
                report['records'] += 1
                yield record
    except (EOFError, gzip.BadGzipFile, zlib.error) as exc:
        report['error'] = str(exc)
    report['complete'] = not bool(report['error'] or report['invalid_lines'] or report['incomplete_lines'])


def iter_samples(path, report=None):
    for record in iter_records(path, report):
        if record.get('type') == 'sample':
            yield record
