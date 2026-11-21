#!/usr/bin/env python3
"""Accounting invariants independent of the generated SDF layout."""

# Resolve the checkout independently of the current working directory.
from pathlib import Path as _ProjectPath
import sys as _project_sys
_PROJECT = next(p for p in _ProjectPath(__file__).resolve().parents if (p / "project.toml").is_file())
_project_sys.path.insert(0, str(_PROJECT / "tools/lib"))
_project_sys.path.insert(0, str(_PROJECT / "tools/modeling"))
from project_paths import PROJECT_ROOT, WORKSPACE, tool_path
import unittest,json,copy
from unittest.mock import patch
import numpy as np
from equipment_ledger import load,rows,dry_links,auxiliary_w
from mass_properties import aggregate
from fuel_mass import combine,configuration,tank_properties
class LedgerTests(unittest.TestCase):
 def test_profile_increment(self):
  self.assertAlmostEqual(sum(r['mass_kg'] for r in rows('speed15_concept'))-sum(r['mass_kg'] for r in rows()),4*76+4*2+30+1.5*(1.5**3-1))
 def test_unique_and_physical(self):
  for profile in ['baseline','speed15_concept']:
   for case in ['light','nominal','heavy']:
    p=aggregate(rows(profile,case));self.assertTrue(np.linalg.eigvalsh(p['inertia_at_cg_kg_m2']).min()>0)
 def test_reject_duplicate(self):
  data=load();data['parts'].append(copy.deepcopy(data['parts'][0]))
  with patch('equipment_ledger.load',return_value=data):
   with self.assertRaises(ValueError):rows()
 def test_fuel_exact_once(self):
  cfg=configuration();dry=dry_links()['hull']
  self.assertAlmostEqual(cfg['dry_hull']['mass_kg'],dry['mass_kg'])
  for l in [0,6,30,60]:self.assertAlmostEqual(combine(dry,tank_properties(cfg,l))['mass_kg']-dry['mass_kg'],.84*l)
 def test_power_modes(self):
  self.assertLess(auxiliary_w('normal'),auxiliary_w('survey'));self.assertLess(auxiliary_w('survey'),auxiliary_w('peak'))
  self.assertAlmostEqual(auxiliary_w('peak'),sum(r['power']['rated_w'] for r in rows() if r['power']['class']=='auxiliary'))
 def test_battery_capacity_matches_count(self):
  self.assertEqual(sum('E163' in r['name'] for r in rows()),2)
  self.assertEqual(sum('E163' in r['name'] for r in rows('speed15_concept')),6)
if __name__=='__main__':unittest.main()
