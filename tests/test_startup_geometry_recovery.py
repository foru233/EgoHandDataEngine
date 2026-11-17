"""Proposal-only tests: load the detached source; never alter installed modules."""
import copy
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import sys
import numpy as np
import pytest
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'tests')]
from egohand.vio.recovery_coordinator import RecoveryCoordinator as Original
from egohand.vio.recovery_contract import application_feedback
from egohand.vio.localization_state import LocalizationPipeline,consumer_map_pose
from egohand.vio.hybrid_quality import make_quality_gate
from egohand.vio.native_bridge import ImuMeasurement
from test_short_vio_recovery import frame
# Exercise the integrated implementation with its feature explicitly off/on.
class LegacyPolicy(Original):
 pass
Proposed=Original
Original=LegacyPolicy


def system(cls=Proposed,enabled=True,stable=.02):
 opts=dict(application_admission=True,short_recovery=True,allow_initialized_anchor=True,timeout_action='independent_new_map')
 if cls is Proposed:opts['allow_startup_relocalization']=enabled
 c=cls(**opts)
 p=LocalizationPipeline('stereo_inertial',make_quality_gate('stereo_inertial',stable_seconds=stable,allow_verified_stereo_bootstrap=True),recovery_commit=lambda out:c.finish_application_frame(application_feedback(out)))
 def run(r,candidate_evidence=None):
  c.begin_frame([ImuMeasurement(r['timestamp_ns'],(0,0,9.81),(0,0,0))])
  r=copy.deepcopy(r)
  if candidate_evidence is not None:
   c.check_candidate(np.array(candidate_evidence['T_world_from_body']).reshape(4,4),r['timestamp_ns'],r['map_epoch'],candidate_evidence)
  r['recovery_preaccept']=c.finish_frame(r,r.get('body_state_candidate'))
  return p.process(r,[])
 return c,p,run

def seed(run,reset=0):
 r=frame(1_000_000_000,reset=reset);r['imu_initialized']=False;r['body_state_candidate']=None
 assert not run(r)['pose_valid']
 run(frame(1_010_000_000,state='RECENTLY_LOST',reset=reset))

def candidate(c,reset=0,**changes):
 r=frame(1_020_000_000,geometry=True,reset=reset);o=r['recovery_observation'];o['phase']='relocalization';o.update(changes)
 T=np.array(r['body_state_candidate']['T_world_from_body'])
 return r,c.check_candidate(T,r['timestamp_ns'],r['map_epoch'],o)

def test_default_off_stays_rejected():
 c,p,run=system(enabled=False);seed(run)
 assert candidate(c)[1]['independent_visual_verification']=='unavailable'

@pytest.mark.parametrize('fault',['stale_timestamp','wrong_reset','wrong_generation','wrong_atlas','weak_stereo','weak_inliers','cells','reprojection','body','quarantine','sensor','no_snapshot'])
def test_candidate_guards(fault):
 c,p,run=system()
 if fault=='no_snapshot':
  r=frame(1_000_000_000);r['reference_frame']['frame_bound']=False;run(r)
 else:seed(run)
 changes={
 'stale_timestamp':{'timestamp_ns':1},'wrong_reset':{'reset_id':4},
 'wrong_generation':{'reference_generation':9},'wrong_atlas':{'atlas_generation':10},
 'weak_stereo':{'stereo_inliers':29},'weak_inliers':{'geometric_inliers':49},
 'cells':{'occupied_cells':5},'reprojection':{'stereo_p95_normalized':3.1},
 'body':{'T_world_from_body':(np.eye(4)*2).reshape(-1).tolist()}}
 if fault=='quarantine':c._quarantine('pose_jump','unavailable')
 if fault=='sensor':c.invalid_reason='imu_non_monotonic'
 assert candidate(c,**changes.get(fault,{}))[1]['independent_visual_verification']=='unavailable'

def test_final_geometry_still_waits_stability_and_separate_segment():
 c,p,run=system()
 run(frame(900_000_000));old=run(frame(920_000_000));assert old['pose_valid']
 seed(run,reset=1)
 r,decision=candidate(c,reset=1);assert decision['startup_geometry_candidate']
 ready=run(r,candidate_evidence=r['recovery_observation']);assert not ready['pose_valid'] and not c.quarantined
 assert ready['recovery_preaccept']['visual_release']['kind']=='independent_startup_stereo_map_geometry'
 assert ready['quality_gate']['reason_code']=='stable_window_pending'
 ready=run(frame(1_040_000_000,geometry=True,reset=1))
 assert ready['pose_valid'] and c.trusted_visual_identity is not None and c.startup_visual_identity is None
 assert ready['local_motion']['T_session_from_segment'] is None
 assert ready['local_motion']['segment_id']!=old['local_motion']['segment_id']
 assert ready['local_motion']['cross_segment_merge_permitted'] is False

@pytest.mark.parametrize('fault',['no_geometry','unbound','frame','atlas','native_lost'])
def test_provisional_acceptance_never_outputs_without_final_geometry(fault):
 c,p,run=system();seed(run);r,decision=candidate(c)
 assert decision['status']=='consistent'
 evidence=copy.deepcopy(r['recovery_observation'])
 if fault=='no_geometry':r.pop('recovery_observation')
 elif fault=='unbound':r['recovery_observation']['pose_binding_verified']=False
 elif fault=='frame':r['recovery_observation']['frame_id']=1
 elif fault=='atlas':r['recovery_observation']['atlas_generation']=99
 elif fault=='native_lost':r['tracking_state']='RECENTLY_LOST';r['pose_valid']=False;r['matrix']=None
 out=run(r,candidate_evidence=evidence)
 assert not out['pose_valid'] and consumer_map_pose(out,world_hands=True) is None


def test_reset_retires_startup_identity():
 c,p,run=system();seed(run);assert c.startup_visual_identity is not None
 c.reset_for_new_map();assert c.startup_visual_identity is None
 assert candidate(c)[1]['independent_visual_verification']=='unavailable'
