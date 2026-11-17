"""Official SMPL joint mapping used by the offline mesh fitter.

The detector supplies sparse anatomical points.  SMPL remains the authority
for parent links and segment lengths; detector points only select root motion
and joint directions.
"""
import numpy as np

SMPL_INDEX = {
    'Pelvis': 0, 'LeftHip': 1, 'RightHip': 2, 'Chest': 9,
    'LeftKnee': 4, 'RightKnee': 5, 'LeftAnkle': 7, 'RightAnkle': 8,
    'Head': 15, 'CervicalBase': 12, 'LeftShoulder': 16, 'RightShoulder': 17,
    'LeftElbow': 18, 'RightElbow': 19, 'LeftWrist': 20, 'RightWrist': 21,
}
SMPL_JOINT_NAMES=('Pelvis','LeftHip','RightHip','Spine1','LeftKnee','RightKnee',
    'Spine2','LeftAnkle','RightAnkle','Chest','LeftFoot','RightFoot','CervicalBase',
    'LeftCollar','RightCollar','Head','LeftShoulder','RightShoulder',
    'LeftElbow','RightElbow','LeftWrist','RightWrist','LeftHand','RightHand')
SMPL_PARENT_IDS=(-1,0,0,0,1,2,3,4,5,6,7,8,9,9,9,12,13,14,16,17,18,19,20,21)
SMPL_PARENTS={i:p for i,p in enumerate(SMPL_PARENT_IDS) if p>=0}
# Optical/body world: X right, Y down, Z forward. SMPL local anatomical
# left is +X and up is +Y. Both X and Y must change: this is a proper
# rotation (det=+1), rather than the old Y-only mirror (det=-1).
SMPL_FROM_BODY=np.diag([-1.,-1.,1.])


def mapped_targets(joints):
    """Return measured points in SMPL's Y-up fitting convention."""
    return {SMPL_INDEX[name]: np.asarray(joints[name], dtype=float) @ SMPL_FROM_BODY.T
            for name in SMPL_INDEX if name in joints}


def rest_bone_lengths(model):
    """Official SMPL rest lengths for all mapped parent-child pairs."""
    with __import__('torch').no_grad():
        out = model(global_orient=__import__('torch').zeros((1, 3)),
                    body_pose=__import__('torch').zeros((1, 69)),
                    betas=__import__('torch').zeros((1, 10)))
    joints = out.joints[0].detach().cpu().numpy()
    return {child: float(np.linalg.norm(joints[child] - joints[parent]))
            for child, parent in SMPL_PARENTS.items()}
