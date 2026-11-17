import numpy as np
from types import SimpleNamespace
from egohand.body.smpl_mapping import SMPL_FROM_BODY,SMPL_PARENT_IDS,SMPL_JOINT_NAMES,mapped_targets
from egohand.body.body_model import BodyPose
from egohand.gui.vio_panel import TrajectoryView
from delivery_views import BodyPoseOverlay,WorldHandOverlay,DISPLAY_WORLD_ROTATION
from egohand.body.smpl_mount import forehead_vertex,HEAD_ANCHOR_VERSION


def test_default_ground_height_is_standing_forehead():
    from egohand.body.smpl_runtime import GroundReference,STANDING_FOREHEAD_HEIGHT_M
    assert STANDING_FOREHEAD_HEIGHT_M == 1.693
    assert GroundReference().height_m == STANDING_FOREHEAD_HEIGHT_M


def test_smpl_mapping_is_proper_rotation_and_preserves_anatomical_left():
    C=SMPL_FROM_BODY
    np.testing.assert_allclose(C@C.T,np.eye(3));assert np.linalg.det(C)==1
    targets=mapped_targets({'LeftShoulder':[-.2,0,0],'RightShoulder':[.2,0,0]})
    assert targets[16][0]>targets[17][0]
    p=np.array([.2,-.7,.8]);np.testing.assert_allclose(p@C.T@C,p)
    assert SMPL_PARENT_IDS[16]==13 and SMPL_PARENT_IDS[13]==9
    assert SMPL_PARENT_IDS[9]==6 and SMPL_PARENT_IDS[6]==3


def test_forehead_mount_uses_upper_front_midline_not_nasal_bridge():
    # Equal weight head points: neither a forward nose tip, skull center,
    # back of head nor an off-center brow may win the camera attachment.
    v=np.tile([0.,.4,0.],(30,1))
    v[:7]=[[0,.6,0],[0,.36,0],[0,.48,.13],[0,.54,.11],
           [.06,.54,.11],[0,.54,-.1],[-.09,.45,0]]
    v[7]=[.09,.45,0]
    assert forehead_vertex(v,np.ones(30))==3
    assert HEAD_ANCHOR_VERSION=='forehead_surface_v2'


def test_unified_surface_and_joint_display_share_rotation_and_clear_invalid(qtbot,tmp_path):
    view=TrajectoryView();qtbot.addWidget(view);overlay=BodyPoseOverlay(view)
    path=tmp_path/'smpl_vertices.npy';v=np.zeros((1,6890,3),np.float32)
    v[0,:3]=[[.1,.2,.3],[.2,.3,.4],[.3,.4,.5]];np.save(path,v)
    pose=BodyPose(1,2,('r',),'estimated_preview',dict(zip(SMPL_JOINT_NAMES,v[0,:24])),
        bones=(('Pelvis','LeftHip'),),diagnostics={'unified_smpl':True,
        'body_mesh_vertices_path':str(path),'body_mesh_index':0,'body_mesh':{'faces':[[0,1,2]]}})
    overlay.update(pose)
    np.testing.assert_allclose(overlay._mesh_item.opts['meshdata'].vertexes(),v[0]@DISPLAY_WORLD_ROTATION.T,atol=1e-7)
    overlay.update(None)
    assert not overlay._mesh_item.visible()


def test_mano_parent_binding_ignores_unrelated_camera_and_detected_wrist(qtbot):
    view=TrajectoryView();qtbot.addWidget(view);overlay=WorldHandOverlay(view)
    local=np.array([[0,0,0],[.1,0,0],[0,.1,0]])
    T=np.eye(4);T[:3,3]=[.3,.4,.5]
    mesh={'coordinate_frame':'body_world','smpl_parent_joint':20,
        'T_world_from_smpl_wrist':T.tolist(),'vertices_wrist_local':local.tolist(),
        'vertices':(local+T[:3,3]).tolist(),'joints20':[[.3,.4,.5]],'faces':[[0,1,2]],'track_id':7}
    camera=np.eye(4);camera[0,3]=10
    hand=SimpleNamespace(track_id=7,joints=[SimpleNamespace(valid=True,xyz_m=np.ones(3)*20)])
    overlay.update_mano_meshes([mesh],camera,[hand])
    actual=overlay.mesh_items[0].opts['meshdata'].vertexes()
    np.testing.assert_allclose(actual,(local+T[:3,3])@DISPLAY_WORLD_ROTATION.T,atol=1e-7)


def test_unified_missing_or_malformed_body_never_calls_custom_estimator(qtbot):
    from client_gui import ClientMainWindow
    w=ClientMainWindow();qtbot.addWidget(w)
    def forbidden(*args,**kwargs):raise AssertionError('Custom skeleton fallback')
    w.body_estimator.update=forbidden
    for body in [None,{'bad':'cache'}]:
        r=SimpleNamespace(diagnostics={'offline_derived_pose':{'unified_smpl':True,'body':body}})
        w._update_body_pose(r);assert w.body_pose is None
    w.close()


def test_current_cached_body_loads_without_starting_a_solver(qtbot,tmp_path,monkeypatch):
    from client_gui import ClientMainWindow
    w=ClientMainWindow();qtbot.addWidget(w)
    def forbidden(*a,**k):raise AssertionError('Cached replay must not optimize')
    monkeypatch.setattr(w.smpl_controller,'request',forbidden)
    path=tmp_path/'vertices.npy';v=np.zeros((1,6890,3),np.float32);np.save(path,v)
    diag={'head_anchor_version':HEAD_ANCHOR_VERSION,'unified_smpl':True}
    body={'frame_id':1,'timestamp_ns':2,'reference_key':None,'status':'estimated_preview',
        'joints':dict(zip(SMPL_JOINT_NAMES,v[0,:24].tolist())),
        'bones':[[SMPL_JOINT_NAMES[p],SMPL_JOINT_NAMES[j]] for j,p in enumerate(SMPL_PARENT_IDS) if p>=0],
        'diagnostics':diag}
    result=SimpleNamespace(frame_id=1,timestamp_ns=2,diagnostics={'offline_derived_pose':{
        'unified_smpl':True,'head_anchor_version':HEAD_ANCHOR_VERSION,'body':body,
        'body_mesh_vertices_path':str(path),'body_mesh_index':0,'body_mesh':{'faces':[[0,1,2]]}}})
    w._update_body_pose(result)
    assert w.body_pose is not None and w.body_overlay._mesh_frame_valid
    result.diagnostics['offline_derived_pose']['body']=None
    w._update_body_pose(result);assert w.body_pose is None
    w.close()


def test_versioned_cache_paths_resolve_inside_moved_result_directory(tmp_path):
    import json
    from egohand.storage.replay import load_recorded_results
    from egohand.core.types import StereoHandResult,serializable
    (tmp_path/'results').mkdir();r=StereoHandResult(1,2,'cal','recorded')
    (tmp_path/'results/frames.jsonl').write_text(json.dumps(serializable(r))+'\n',encoding='utf8')
    (tmp_path/'new_mesh').mkdir()
    (tmp_path/'new_mesh/smpl_mesh.json').write_text(json.dumps({'faces':[[0,1,2]]}),encoding='utf8')
    (tmp_path/'unified_smpl_manifest.json').write_text(json.dumps({'head_anchor_version':HEAD_ANCHOR_VERSION,
        'vertices_file':'new_mesh/smpl_vertices.npy','mesh_file':'new_mesh/smpl_mesh.json'}),encoding='utf8')
    (tmp_path/'derived_pose.jsonl').write_text(json.dumps({'frame_id':1,'timestamp_ns':2,
        'unified_smpl':True,'body_mesh_ref':'new_mesh/smpl_mesh.json'})+'\n',encoding='utf8')
    results,truncated=load_recorded_results(tmp_path);assert not truncated
    d=results[0].diagnostics['offline_derived_pose']
    assert d['body_mesh_vertices_path']==str(tmp_path/'new_mesh/smpl_vertices.npy')
    assert d['body_mesh']['faces']==[[0,1,2]] and d['head_anchor_version']==HEAD_ANCHOR_VERSION


def _pose_result(frame,stamp,y=0.,epoch=0):
    T=np.eye(4);T[1,3]=y
    return SimpleNamespace(frame_id=frame,timestamp_ns=stamp,session_id='recording',hands=[],
        diagnostics={'vio_pose':{'pose_valid':True,'timestamp_ns':stamp,'tracking_state':'OK',
            'map_epoch':epoch,'matrix':T.tolist()}})


def test_editable_ground_uses_initial_camera_height_even_after_seek_or_rising():
    from egohand.body.smpl_runtime import GroundReference
    g=GroundReference(1.1)
    first=_pose_result(0,100,y=.2);standing=_pose_result(60,200,y=-.3)
    g.observe(first)
    assert np.isclose(g.floor_for(standing),1.3)
    g.height_m=.8
    assert np.isclose(g.floor_for(standing),1.)
    assert np.isclose(g.floor_for(first),1.)
    new_map=_pose_result(70,300,y=5.,epoch=1)
    assert np.isclose(g.floor_for(new_map),5.8)
    assert np.isclose(g.floor_for(first),1.)


def test_runtime_request_keeps_same_frame_hand_and_rejects_untrusted_camera():
    from egohand.body.smpl_runtime import request_from_result
    from egohand.core.types import Hand3D
    r=_pose_result(7,123,y=.2);h=Hand3D('Left',1.,3)
    h.joints[0].valid=True;h.joints[0].xyz_m=np.array([.1,.2,.3]);r.hands=[h]
    request=request_from_result(r,1.3)
    assert request['timestamp_ns']==123 and request['frame_id']==7
    np.testing.assert_allclose(request['wrists']['left'],[.1,.4,.3])
    r.diagnostics['vio_pose']['pose_valid']=False
    assert request_from_result(r,1.3) is None


def test_smpl_controller_discards_older_floor_and_keeps_only_latest_request(qtbot,monkeypatch):
    import json
    from egohand.gui.smpl_controller import SmplController
    c=SmplController();received=[];c.completed.connect(received.append)
    monkeypatch.setattr(c,'_ensure',lambda:True)
    c.asset={'faces':[]}
    written=[]
    class Process:
        def write(self,data):written.append(json.loads(data))
        def readAllStandardOutput(self):return self.reply
    c.process=Process()
    c.request({'ground_y_m':1.6});first=c.active['request_id']
    c.request({'ground_y_m':1.2});c.request({'ground_y_m':.9})
    assert len(written)==1
    c.process.reply=(json.dumps({'type':'result','request_id':first,'accepted':True})+'\n').encode()
    c._read();assert received==[] and len(written)==2
    assert written[1]['ground_y_m']==.9
    c.process.reply=(json.dumps({'type':'result','request_id':c.active['request_id'],'accepted':True})+'\n').encode()
    c._read();assert len(received)==1 and not c.pending
    c.timeout.stop();c.process=None;c.close()
