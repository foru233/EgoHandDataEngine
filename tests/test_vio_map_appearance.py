"""Synthetic data checks; no native tracking accuracy or world continuity claim."""
import base64
import copy
import ctypes
import hashlib
import json
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from egohand.vio.native_bridge import NativeVioBridge, _CMapAppearanceHeader as H, _CMapAppearancePoint as P
from egohand.vio.visual_requalification import (AppearanceMatchLimits, decode_map_appearance,
    match_map_appearance_stereo, _APPEARANCE_DTYPE, _mutual_descriptor_matches)
from egohand.vio.live_subprocess import send
from egohand.vio.recovery_contract import MAX_MESSAGE, RecoveryProtocolError


def fn(call):
    return lambda *args: call(*args)


def fake_bridge(change=None, rc=0):
    bridge = NativeVioBridge.__new__(NativeVioBridge)
    bridge._closed = False
    bridge._handle = 1
    snapshot = dict(snapshot_consistent=True, frame_id=7, timestamp_ns=100, map_id=3,
                    reset_id=1, map_revision=4, map_change_index=5)
    def capture(handle, frame, timestamp, header, size, points, stride, capacity):
        h = ctypes.cast(header, ctypes.POINTER(H)).contents
        for k, v in snapshot.items():
            if k != 'snapshot_consistent': setattr(h, k, v)
        h.schema_version=1; h.struct_size=ctypes.sizeof(H); h.status=1 if rc==0 else 2
        h.flags=1; h.source_kind=1; h.capacity=capacity
        h.point_count=2 if rc==0 else 0; h.sampled_count=2; h.total_map_points=2
        h.image_width=800; h.image_height=600; h.fx=h.fy=400; h.cx=400; h.cy=300; h.bf=26
        for i in range(2):
            points[i].point_id=i; points[i].position[:]=[i,0,3]; points[i].observations=2
        if change: change(h, points)
        return rc
    bridge._dll = SimpleNamespace(egohand_vio_capabilities=fn(lambda:b'map_appearance_v1'),
        egohand_vio_map_appearance_header_size=fn(lambda:ctypes.sizeof(H)),
        egohand_vio_map_appearance_point_size=fn(lambda:ctypes.sizeof(P)),
        egohand_vio_map_appearance_offset=fn(lambda kind,i:getattr((H,P)[kind],(H,P)[kind]._fields_[i][0]).offset),
        egohand_vio_get_map_appearance=fn(capture))
    return bridge, snapshot


def test_native_layout_and_all_offsets_checked():
    assert ctypes.sizeof(H)==136 and ctypes.sizeof(P)==56
    bridge,snapshot=fake_bridge()
    assert bridge.get_map_appearance(snapshot,100)['point_count']==2
    for kind, structure in enumerate((H,P)):
        for index in range(len(structure._fields_)):
            bridge,snapshot=fake_bridge()
            original=bridge._dll.egohand_vio_map_appearance_offset
            bridge._dll.egohand_vio_map_appearance_offset=fn(
                lambda k,i:original(k,i)+int((k,i)==(kind,index)))
            with pytest.raises(RuntimeError,match='offset'):
                bridge.get_map_appearance(snapshot,100)


@pytest.mark.parametrize('key', ['frame_id','timestamp_ns','map_id','reset_id','map_revision','map_change_index'])
def test_native_wrong_identity_rejected(key):
    bridge,snapshot=fake_bridge(lambda h,p:setattr(h,key,getattr(h,key)+1))
    with pytest.raises(RuntimeError,match='identity'):
        bridge.get_map_appearance(snapshot,100)


@pytest.mark.parametrize('change', [lambda h,p:setattr(h,'schema_version',2),
    lambda h,p:setattr(h,'point_count',1025), lambda h,p:setattr(h,'flags',0),
    lambda h,p:setattr(h,'fx',float('nan')), lambda h,p:setattr(h,'sampled_count',1),
    lambda h,p:setattr(p[1],'point_id',p[0].point_id),
    lambda h,p:p[0].position.__setitem__(0,float('inf'))])
def test_native_bad_payload_rejected(change):
    bridge,snapshot=fake_bridge(change)
    with pytest.raises(RuntimeError):bridge.get_map_appearance(snapshot,100)


def test_unavailable_closed_capacity_and_optional_legacy():
    bridge,snapshot=fake_bridge(rc=1)
    assert bridge.get_map_appearance(snapshot,100)['status']=='unavailable'
    bridge._closed=True
    with pytest.raises(RuntimeError,match='closed'):bridge.get_map_appearance(snapshot,100)
    bridge._closed=False
    for capacity in (0,4097,True):
        with pytest.raises(ValueError):bridge.get_map_appearance(snapshot,100,capacity=capacity)
    bridge._dll.egohand_vio_capabilities=fn(lambda:b'result;snapshot')
    bridge._check_abi=lambda:None
    bridge._check_abi()  # Existing default path never requests this extension.
    with pytest.raises(RuntimeError,match='capability'):bridge.get_map_appearance(snapshot,100)


@pytest.fixture
def appearance_sample():
    rng=np.random.default_rng(481)
    left=rng.integers(0,256,(480,640),dtype=np.uint8)
    right=np.zeros_like(left); right[:,:-8]=left[:,8:]
    limits=AppearanceMatchLimits(2000,1.2,8,19,12,.75,55.,1.5,.5,150.)
    orb=cv2.ORB_create(nfeatures=2000,scaleFactor=1.2,nlevels=8,edgeThreshold=19,fastThreshold=12)
    keys,desc=orb.detectAndCompute(left,None)
    data=np.zeros(len(desc),dtype=_APPEARANCE_DTYPE)
    data['id']=np.arange(len(desc)); data['descriptor']=desc; data['observations']=3
    data['position'][:,2]=3.
    bridge,snapshot=fake_bridge()
    value=bridge.get_map_appearance(snapshot,100)
    value.update(point_count=len(data), sampled_count=len(data),total_map_points=len(data),capacity=4096,
        image_width=640,image_height=480,input_image_size=[640,480],input_timestamp_ns=100,
        bridge_instance_id='synthetic-session',settings_sha256='a'*64,
        points_base64=base64.b64encode(data.tobytes()).decode(),
        left_sha256=hashlib.sha256(left.tobytes()).hexdigest(),
        right_sha256=hashlib.sha256(right.tobytes()).hexdigest())
    return dict(appearance=value,left_gray=left,right_gray=right,timestamp_ns=100,
        bridge_instance_id='synthetic-session',settings_sha256='a'*64,limits=limits,
        camera_matrix=np.array([[400.,0,400.],[0,400.,300.],[0,0,1.]]),baseline_m=.065)


def test_independent_pixels_match_and_candidate_cannot_select_matches(appearance_sample):
    first=match_map_appearance_stereo(**appearance_sample)
    assert len(first['point_ids'])>100
    assert not first['candidate_pose_used_for_matching'] and not first['tracking_correspondences_used']
    # Even adversarial pose metadata cannot alter the matcher inputs or result.
    appearance_sample['appearance']['native_T_map_from_camera']=(np.eye(4)*999).tolist()
    second=match_map_appearance_stereo(**appearance_sample)
    assert first['point_ids']==second['point_ids']
    np.testing.assert_array_equal(first['left_pixels'],second['left_pixels'])
    assert not second['production_release_authorized']


@pytest.mark.parametrize('key,value', [('bridge_instance_id','wrong'),('settings_sha256','b'*64),
    ('timestamp_ns',101)])
def test_image_provenance_mismatch_rejected(appearance_sample,key,value):
    appearance_sample[key]=value
    with pytest.raises(ValueError):match_map_appearance_stereo(**appearance_sample)


@pytest.mark.parametrize('key', ['left_gray','right_gray'])
def test_wrong_pixels_rejected(appearance_sample,key):
    appearance_sample[key]=np.roll(appearance_sample[key],3,axis=1)
    with pytest.raises(ValueError,match='image mismatch'):match_map_appearance_stereo(**appearance_sample)


def test_duplicate_points_and_truncation_explicit(appearance_sample):
    value=appearance_sample['appearance']
    value['total_map_points']+=1
    with pytest.raises(ValueError):decode_map_appearance(value,bridge_instance_id='synthetic-session',settings_sha256='a'*64)
    value['flags']=3
    decoded=decode_map_appearance(value,bridge_instance_id='synthetic-session',settings_sha256='a'*64).copy()
    decoded['id'][1]=decoded['id'][0]
    value['points_base64']=base64.b64encode(decoded.tobytes()).decode()
    with pytest.raises(ValueError,match='points'):decode_map_appearance(value,bridge_instance_id='synthetic-session',settings_sha256='a'*64)


def test_repeated_texture_ties_are_rejected(appearance_sample):
    desc=np.zeros((10,32),np.uint8)
    assert _mutual_descriptor_matches(desc,desc,appearance_sample['limits'])=={}


def test_wrong_calibration_rejected(appearance_sample):
    appearance_sample['camera_matrix'][0,0]+=10
    with pytest.raises(ValueError,match='calibration'):match_map_appearance_stereo(**appearance_sample)


def test_explicit_frozen_map_uses_later_frame_without_relabelling_snapshot(appearance_sample):
    frozen=copy.deepcopy(appearance_sample['appearance'])
    binding=copy.deepcopy(frozen)
    binding.update(frame_id=8,timestamp_ns=101,input_timestamp_ns=101,map_revision=5,map_change_index=6)
    appearance_sample.update(frame_binding=binding,timestamp_ns=101)
    result=match_map_appearance_stereo(**appearance_sample)
    assert result['frozen_map_reference'] and result['map_snapshot_frame_id']==7 and result['image_frame_id']==8
    assert not result['coordinate_continuity_verified']
    assert appearance_sample['appearance']==frozen
    binding['atlas_generation']+=1
    with pytest.raises(ValueError,match='identity mismatch'):match_map_appearance_stereo(**appearance_sample)


def test_shuffled_descriptor_associations_destroy_independent_matches(appearance_sample):
    first=match_map_appearance_stereo(**appearance_sample)
    value=appearance_sample['appearance']
    data=np.frombuffer(base64.b64decode(value['points_base64']),dtype=_APPEARANCE_DTYPE).copy()
    # Bitwise corruption: map descriptor content no longer describes input pixels.
    data['descriptor']^=np.uint8(255)
    value['points_base64']=base64.b64encode(data.tobytes()).decode()
    second=match_map_appearance_stereo(**appearance_sample)
    assert len(second['point_ids'])<len(first['point_ids'])/10


def test_ipc_1024_points_fit_without_lifting_limit():
    bridge,snapshot=fake_bridge()
    value=bridge.get_map_appearance(snapshot,100)
    value['points_base64']=base64.b64encode(bytes(1024*56)).decode()
    message={'map_appearance':value,'existing_metadata_allowance':'x'*16000}
    assert len(json.dumps(message).encode())<MAX_MESSAGE
    class Socket:
        def sendall(self,data):self.size=len(data)
    sock=Socket();send(sock,message)
    with pytest.raises(RecoveryProtocolError):send(sock,{'too_big':'x'*MAX_MESSAGE})


def with_reference(value, generation=1):
    report = {k:value[k] for k in ('frame_id','timestamp_ns','map_id','reset_id',
                                   'map_revision','map_change_index','atlas_generation')}
    report.update(schema_version=1, frame_bound=True, snapshot_consistent=True,
        generation=generation, operation_sequence=generation, geometry_sequence=8,
        basis='successful_initializer_commit', coordinate_continuity_verified=False)
    return {**value, 'reference_generation':generation, 'reference_frame':report}


def test_native_appearance_rechecks_reference_after_numeric_copy():
    bridge, snapshot = fake_bridge()
    snapshot.update(with_reference({**snapshot, 'atlas_generation':0}))
    bridge.get_reference_frame = lambda s: copy.deepcopy(s['reference_frame'])
    copied = bridge.get_map_appearance(snapshot,100)
    assert copied['reference_generation'] == 1
    assert copied['reference_frame']['geometry_sequence'] == 8
    # A copy is no longer usable if a new initializer commits during the getter.
    for field in ('generation','operation_sequence','geometry_sequence','atlas_generation','frame_id'):
        changed = {**snapshot['reference_frame'], field:snapshot['reference_frame'][field]+1}
        bridge.get_reference_frame = lambda s, changed=changed: changed
        assert bridge.get_map_appearance(snapshot,100)['reason'] == 'reference_changed_during_appearance_copy'
    bridge.get_reference_frame = lambda s: None
    assert bridge.get_map_appearance(snapshot,100)['status'] == 'unavailable'
    snapshot['reference_generation'] = None
    assert bridge.get_map_appearance(snapshot,100)['reason'] == 'reference_frame_unavailable'


@pytest.mark.parametrize('change', ['changed','missing','invalid','atlas','wrong_frame','wrong_operation'])
def test_frozen_appearance_cannot_cross_known_reference(appearance_sample, change):
    appearance = with_reference(appearance_sample['appearance'])
    binding = with_reference({**appearance, 'frame_id':8, 'timestamp_ns':101,
                              'input_timestamp_ns':101, 'map_revision':5, 'map_change_index':6})
    if change == 'changed': binding = with_reference(binding,2)
    elif change == 'missing':
        del binding['reference_generation']; del binding['reference_frame']
    elif change == 'invalid': binding['reference_generation'] = True
    elif change == 'atlas': binding['atlas_generation'] += 1
    elif change == 'wrong_frame': binding['reference_frame']['frame_id'] = 7
    elif change == 'wrong_operation': binding['reference_frame']['operation_sequence'] = 0
    with pytest.raises(ValueError,match='reference|identity'):
        match_map_appearance_stereo(**{**appearance_sample,'appearance':appearance,
            'frame_binding':binding,'timestamp_ns':101})


def test_known_reference_can_compare_later_content_without_claiming_continuity(appearance_sample):
    appearance = with_reference(appearance_sample['appearance'])
    binding = with_reference({**appearance,'frame_id':8,'timestamp_ns':101,'input_timestamp_ns':101,
                              'map_revision':5,'map_change_index':6})
    result = match_map_appearance_stereo(**{**appearance_sample,'appearance':appearance,
        'frame_binding':binding,'timestamp_ns':101})
    assert result['reference_identity_known'] and result['reference_generation'] == 1
    assert not result['coordinate_continuity_verified'] and not result['production_release_authorized']
