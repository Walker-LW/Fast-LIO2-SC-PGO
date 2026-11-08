"""Regression checks for tool-version dates versus experiment identities."""
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch
import manage


class RunIdentityTests(unittest.TestCase):
    def test_explicit_new_date_uses_existing_tool(self):
        with tempfile.TemporaryDirectory() as d:
            base=Path(d)
            with patch.object(manage,'BASE',base), patch.object(manage,'HERE',base/'existing-tool-20261107'), \
                 patch.object(manage,'ID',manage.ID), patch.object(manage,'RUN',manage.RUN):
                manage.select_run_id('scpgo-corridor02-geometry-20261108-001')
                self.assertEqual(manage.RUN,base/'results/scpgo/scpgo-corridor02-geometry-20261108-001')
                self.assertFalse(manage.RUN.exists())

    def test_prepared_copy_uses_its_manifest_and_rejects_other_id(self):
        with tempfile.TemporaryDirectory() as d:
            base=Path(d);run=base/'results/scpgo/scpgo-corridor02-geometry-20261108-001'
            (run/'config').mkdir(parents=True)
            (run/'config/manifest.json').write_text(json.dumps({'id':run.name}))
            with patch.object(manage,'BASE',base), patch.object(manage,'HERE',run), \
                 patch.object(manage,'ID',manage.ID), patch.object(manage,'RUN',manage.RUN):
                manage.select_run_id()
                self.assertEqual(manage.RUN,run)
                with self.assertRaises(AssertionError):manage.select_run_id('scpgo-corridor02-geometry-20261107-001')

    def test_invalid_and_empty_explicit_ids_are_rejected(self):
        with tempfile.TemporaryDirectory() as d, patch.object(manage,'HERE',Path(d)):
            for value in ['', 'scpgo-corridor01-geometry-20261108-001', 'scpgo-corridor02-geometry-20261308-001']:
                with self.subTest(value=value), self.assertRaises((AssertionError,ValueError)):
                    manage.select_run_id(value)


if __name__=='__main__':unittest.main()
