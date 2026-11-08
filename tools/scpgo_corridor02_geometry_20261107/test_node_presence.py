"""Regression: an unknown ROS node can still yield a successful info command."""
import subprocess
import unittest
from unittest.mock import patch
import manage


class NodePresenceTests(unittest.TestCase):
    def test_absent_node_is_false_even_when_cli_succeeds(self):
        with patch.object(manage.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'/rosout\n','')) as run:
            self.assertFalse(manage.node_present('/laserMapping'))
            self.assertEqual(run.call_args.args[0][-2:],['rosnode','list'])

    def test_registered_node_is_true(self):
        with patch.object(manage.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'/rosout\n/laserMapping\n','')):
            self.assertTrue(manage.node_present('/laserMapping'))

    def test_master_failure_is_not_misclassified_as_absent(self):
        with patch.object(manage.subprocess,'run',side_effect=subprocess.CalledProcessError(1,['rosnode','list'])):
            with self.assertRaises(subprocess.CalledProcessError):manage.node_present('/laserMapping')


if __name__=='__main__':unittest.main()
