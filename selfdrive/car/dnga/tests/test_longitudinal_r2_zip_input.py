"""Offline ZIP input tests for the R2 native controller runner (no cereal needed)."""
import ast
import bz2
import re
import tempfile
import unittest
import zipfile
from pathlib import Path


def load_zip_reader():
  src = (Path(__file__).resolve().parent / 'r2_exact_controller_replay.py').read_text()
  root = ast.parse(src)
  defs = [node for node in root.body
          if isinstance(node, ast.FunctionDef)
          and node.name in ('route_label', 'native_sort', 'iter_rlogs')]
  scope = {'Path': Path, 'zipfile': zipfile, 'bz2': bz2, 're': re}
  exec(compile(ast.Module(body=defs, type_ignores=[]), '<zip-reader>', 'exec'), scope)
  return scope['iter_rlogs']


class TestR2ZipReader(unittest.TestCase):
  def test_reads_and_orders_original_segment_layout_without_extracting(self):
    read = load_zip_reader()
    with tempfile.TemporaryDirectory() as tmp:
      archive = Path(tmp) / 'realdata1.zip'
      with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('2026-11-05--11-32-49--9/qlog.bz2', bz2.compress(b'NO'))
        z.writestr('2026-11-05--11-32-49--10/rlog.bz2', bz2.compress(b'TEN'))
        z.writestr('2026-11-05--11-32-49--2/rlog.bz2', bz2.compress(b'TWO'))
      received = list(read([archive]))
      self.assertEqual([str(p.parent.name) for p, _ in received],
                       ['2026-11-05--11-32-49--2', '2026-11-05--11-32-49--10'])
      self.assertEqual([bytes(raw) for _, raw in received], [b'TWO', b'TEN'])
      self.assertFalse((Path(tmp) / '2026-11-05--11-32-49--2').exists())

  def test_bad_archive_fails_explicitly(self):
    read = load_zip_reader()
    with tempfile.TemporaryDirectory() as tmp:
      archive = Path(tmp) / 'bad.zip'
      with zipfile.ZipFile(archive, 'w') as z:
        z.writestr('note.txt', 'not an rlog')
      with self.assertRaisesRegex(ValueError, 'no YYYY-MM-DD'):
        list(read([archive]))


if __name__ == '__main__':
  unittest.main()
