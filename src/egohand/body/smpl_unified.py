"""Single-pose SMPL forward model. Optional torch runtime, never imported by UI.

The SMPL asset supplies rest joints, parent tree, blend shapes and skinning.
The explicit conservative pose bounds here are application priors, NOT
official SMPL joint limits or observations of untracked torso/legs.
"""
import numpy as np
import torch
from .smpl_mapping import SMPL_FROM_BODY as C,SMPL_JOINT_NAMES,SMPL_PARENT_IDS
from .smpl_mount import forehead_vertex,HEAD_ANCHOR_VERSION

def palm_basis(points):
    """A proper rotation from wrist, index/middle/little MCP observations."""
    p=np.asarray(points,dtype=float)
    if p.shape!=(21,3) or not np.isfinite(p[[0,5,9,17]]).all():return None
    y=p[9]-p[0];y/=max(np.linalg.norm(y),1e-12)
    x=p[5]-p[17];x-=y*np.dot(x,y);n=np.linalg.norm(x)
    if n<1e-5:return None
    x/=n;z=np.cross(x,y)
    return np.column_stack((x,y,z))

class UnifiedSmpl:
    def __init__(self,model):
        self.model=model
        self.parents=model.parents.detach().cpu().numpy().tolist()
        if tuple(self.parents)!=SMPL_PARENT_IDS:raise ValueError('SMPL asset hierarchy mismatch')
        shape=torch.nn.Parameter(torch.zeros((1,10)))
        shape_opt=torch.optim.Adam([shape],lr=.03)
        for _ in range(80):
            shape_opt.zero_grad()
            v=model(global_orient=torch.zeros((1,3)),body_pose=torch.zeros((1,69)),betas=shape).vertices[0]
            height=v[:,1].max()-v[:,1].min()
            ((height-1.75).square()+1e-5*shape.square().sum()).backward();shape_opt.step()
            with torch.no_grad():shape.clamp_(-2.,2.)
        self.betas=shape.detach()
        with torch.no_grad():
            out=model(global_orient=torch.zeros((1,3)),body_pose=torch.zeros((1,69)),betas=self.betas)
        self.rest=out.joints[0,:24].detach()
        v=out.vertices[0].detach().numpy()
        # Explicit uncalibrated forehead anchor on the official template. It
        # avoids falsely equating SMPL's internal head joint with camera origin.
        head=self.rest[15].numpy()
        weights=model.lbs_weights[:,15].numpy()
        self.head_vertex=forehead_vertex(v,weights)
        self.head_offset=torch.tensor(v[self.head_vertex]-head,dtype=torch.float32)
        self.rest_height=float(np.ptp(v[:,1]));self.bounds=self._bounds()
        weights_all=model.lbs_weights.numpy()
        self.feet=[torch.tensor(np.flatnonzero(weights_all[:,a]+weights_all[:,b]>.75),dtype=torch.long)
                   for a,b in [(7,10),(8,11)]]
        self.rest_floor=float(np.mean([v[ids.numpy(),1].min() for ids in self.feet]))
        # Optimization only needs 24 official joints, the forehead vertex and
        # foot surfaces. Evaluate the exact same SMPL blend/skinning equations
        # on those vertices; generate all 6890 vertices once for final output.
        from smplx.lbs import blend_shapes
        self.sparse_ids=torch.unique(torch.cat((*self.feet,torch.tensor([self.head_vertex]))))
        shaped=(model.v_template[None]+blend_shapes(self.betas,model.shapedirs)).detach()
        self.sparse_shaped=shaped[:,self.sparse_ids]
        self.sparse_posedirs=model.posedirs.reshape(207,-1,3)[:,self.sparse_ids].reshape(207,-1).detach()
        self.sparse_weights=model.lbs_weights[self.sparse_ids].detach()
        self.previous=None;self.reference=None;self.ground=None
        self.previous_joints=None;self.previous_head=None

    def _bounds(self):
        bounds=np.tile(np.array([[-1.4,1.4]]),(23,3,1))
        for j in (3,6,9):bounds[j-1]=[[-.35,.35],[-.25,.25],[-.30,.30]]
        for j in (1,2):bounds[j-1]=[[-2.1,.5],[-.5,.5],[-.55,.55]]
        for j in (4,5):bounds[j-1]=[[0.,2.65],[-.06,.06],[-.06,.06]]
        for j in (7,8):bounds[j-1]=[[-.95,.65],[-.20,.20],[-.25,.25]]
        for j in (10,11,22,23):bounds[j-1]=[[-.1,.1]]*3
        bounds[18-1]=[[-.12,.12],[-2.65,0.],[-.12,.12]]
        bounds[19-1]=[[-.12,.12],[0.,2.65],[-.12,.12]]
        for j in (16,17):bounds[j-1]=[[-2.5,2.5],[-1.6,1.6],[-1.8,1.8]]
        return torch.tensor(bounds,dtype=torch.float32)

    def forward(self,orient,pose,trans,head_R=None,head=None,*,sparse=False):
        from smplx.lbs import batch_rodrigues
        local=batch_rodrigues(torch.cat((orient,pose),dim=1).reshape(-1,3))
        rotations=[local[0]]
        local_list=list(local.unbind())
        for j in range(1,24):
            if j==15 and head_R is not None:
                local_list[j]=rotations[self.parents[j]].T@head_R
            rotations.append(rotations[self.parents[j]]@local_list[j])
        local=torch.stack(local_list)
        if sparse:
            from types import SimpleNamespace
            from smplx.lbs import batch_rigid_transform
            joints,A=batch_rigid_transform(local[None],self.rest[None],self.model.parents)
            offsets=((local[None,1:]-torch.eye(3)).reshape(1,-1)@self.sparse_posedirs).reshape(1,-1,3)
            posed=self.sparse_shaped+offsets
            skin=(self.sparse_weights[None]@A.reshape(1,24,16)).reshape(1,-1,4,4)
            homo=torch.cat((posed,torch.ones((1,len(self.sparse_ids),1))),dim=2)
            selected=(skin@homo[...,None])[...,:3,0]+trans[:,None]
            vertices=torch.zeros((1,6890,3)).index_copy(1,self.sparse_ids,selected)
            out=SimpleNamespace(vertices=vertices,joints=joints+trans[:,None])
        else:
            out=self.model(global_orient=local[None,:1],body_pose=local[None,1:],transl=trans,
                           betas=self.betas,pose2rot=False)
        if head is not None:
            # Forehead location is an exact attachment constraint; root
            # translation is derived rather than traded against hand errors.
            delta=head-out.vertices[0,self.head_vertex]
            out.vertices=out.vertices+delta[None,None,:]
            out.joints=out.joints+delta[None,None,:]
            out.transl_effective=trans+delta[None,:]
        out.local_rotations=local
        return out,torch.stack(rotations)

    def fit(self,head_world,head_rotation,wrists,reference,*,iterations=None,ground_y_m=None,hand_directions=None):
        # All observations enter a single proper world<->SMPL rotation.
        head=torch.tensor(np.asarray(head_world)@C.T,dtype=torch.float32)
        head_R=torch.tensor(C@np.asarray(head_rotation)@C.T,dtype=torch.float32)
        restarting=self.previous is None or reference!=self.reference
        neutral=torch.zeros((1,69));neutral[0,47]=-1.15;neutral[0,50]=1.15
        neutral[0,52]=-.25;neutral[0,55]=.25
        if restarting:
            pose0=neutral
            orient0=torch.zeros((1,3))
            # Share large yaw with the torso instead of twisting the neck.
            orient0[0,1]=float(np.arctan2(head_R[0,2],head_R[2,2]))
            trans0=(head-self.rest[15]-self.head_offset).reshape(1,3)
            # Initial feet define a local unobserved floor prior per segment.
            if self.ground is None or reference!=self.reference:
                self.ground=self.rest_floor+float(trans0[0,1])
        else:orient0,pose0,trans0=self.previous
        if ground_y_m is not None:self.ground=-float(ground_y_m)
        # Straight knees have almost no vertical derivative. Seed a symmetric
        # squat from the observed head height so contact optimization cannot
        # stall at the extension bound. Lengths come from this SMPL shape.
        neutral_height=float(self.rest[15,1]+self.head_offset[1])-self.rest_floor
        drop=max(0.,neutral_height-(float(head[1])-self.ground))
        leg_length=float(torch.linalg.vector_norm(self.rest[1]-self.rest[4])+
                         torch.linalg.vector_norm(self.rest[4]-self.rest[7]))
        knee=2.*np.arccos(np.clip(1.-drop/leg_length,.3,1.))
        pose0=pose0.clone()
        # A warm pose already has bent knees and valid contact. Reseeding it
        # every frame injected unobserved leg motion and defeated continuity.
        if restarting:
            for joint,angle in [(1,-knee/2),(2,-knee/2),(4,knee),(5,knee),(7,-knee/2),(8,-knee/2)]:
                pose0[0,(joint-1)*3]=float(angle)
        elif drop>.02 and max(float(pose0[0,9]),float(pose0[0,12]))<.025:
            # Break the straight-leg singularity by a tiny continuous step;
            # never replace a warm leg pose with a complete squat at a height
            # threshold (that produced a 32 cm knee step in frame 1465).
            for joint,angle in [(1,-.015),(2,-.015),(4,.03),(5,.03),(7,-.015),(8,-.015)]:
                pose0[0,(joint-1)*3]+=angle
        camera_height=float(head[1])-self.ground
        if restarting and camera_height<1.0:
            # A deep squat needs hip flexion AND a forward torso. A seed
            # with only knees bent cannot reach a low forehead height.
            torso_height=max(.5,neutral_height-leg_length)
            leg_height=max(.12,leg_length*.22)
            lean=float(np.arccos(np.clip((camera_height-leg_height)/torso_height,.15,1.)))
            root_lean=min(.3,lean*.3);orient0=orient0.clone();orient0[0,0]=root_lean
            for joint in (1,2):pose0[0,(joint-1)*3]=float(-.65*knee-root_lean)
            for joint in (7,8):pose0[0,(joint-1)*3]=float(-.35*knee)
            for joint in (3,6,9):pose0[0,(joint-1)*3]=min(.35,(lean-root_lean)/3.)
            pose0[0,33]=-lean*.5
            # Initialize each observed arm using THIS SMPL shape's two limb
            # lengths and an anatomical elbow plane, then optimize the whole
            # pose. No handcrafted shoulder widths or bone lengths enter.
            from scipy.spatial.transform import Rotation
            with torch.no_grad():seed,seed_R=self.forward(orient0,pose0,trans0,head_R,head,sparse=True)
            for side,target_world in wrists.items():
                shoulder,elbow,wrist=(16,18,20) if side=='left' else (17,19,21)
                origin=seed.joints[0,shoulder].numpy()
                target=np.asarray(target_world)@C.T
                upper=(self.rest[elbow]-self.rest[shoulder]).numpy()
                lower=(self.rest[wrist]-self.rest[elbow]).numpy()
                a=np.linalg.norm(upper);b=np.linalg.norm(lower);vec=target-origin;distance=np.linalg.norm(vec)
                if distance<1e-5:continue
                d=float(np.clip(distance,abs(a-b)+1e-4,a+b-1e-4));axis=vec/distance
                pole=np.array([0.,-1.,0.]);pole-=axis*np.dot(axis,pole)
                if np.linalg.norm(pole)<1e-4:pole=np.cross(axis,[1.,0.,0.])
                pole/=np.linalg.norm(pole)
                along=(a*a-b*b+d*d)/(2*d);off=np.sqrt(max(0.,a*a-along*along))
                desired_upper=axis*along+pole*off
                desired_lower=axis*d-desired_upper
                flex=float(np.arccos(np.clip((d*d-a*a-b*b)/(2*a*b),-1.,1.)))
                elbow_angle=-flex if side=='left' else flex
                er=Rotation.from_rotvec([0.,elbow_angle,0.]).as_matrix()
                world_R=Rotation.align_vectors(np.stack((desired_upper/a,desired_lower/b)),
                    np.stack((upper/a,(er@lower)/b)))[0].as_matrix()
                local_R=seed_R[self.parents[shoulder]].numpy().T@world_R
                pose0[0,(shoulder-1)*3:(shoulder-1)*3+3]=torch.tensor(Rotation.from_matrix(local_R).as_rotvec(),dtype=torch.float32)
                pose0[0,(elbow-1)*3+1]=elbow_angle
        orient=torch.nn.Parameter(orient0.clone());pose=torch.nn.Parameter(pose0.clone());trans=torch.nn.Parameter(trans0.clone())
        opt=torch.optim.Adam([orient,pose,trans],lr=.025 if restarting else .006)
        targets={20 if side=='left' else 21:torch.tensor(np.asarray(p)@C.T,dtype=torch.float32)
                 for side,p in wrists.items()}
        directions={20 if side=='left' else 21:torch.tensor(np.asarray(p)@C.T,dtype=torch.float32)
                    for side,p in (hand_directions or {}).items()}
        for _ in range(iterations or (100 if restarting else 20)):
            opt.zero_grad();out,R=self.forward(orient,pose,trans,head_R,head,sparse=True)
            joints=out.joints[0,:24]
            # The head is rigidly attached to the camera. Penalize the
            # derived local rotation so pelvis/neck share the turn naturally.
            loss=(1.2 if camera_height<1.0 else .12)*(out.local_rotations[15]-torch.eye(3)).square().sum()
            if camera_height<1.0:
                # With a very low camera and two outstretched arms, the
                # unconstrained yaw can flip the torso backwards to shorten
                # reach. Keep its horizontal heading near the observed head.
                heading=R[0][[0,2],2];head_heading=head_R[[0,2],2]
                loss+=1.2*(heading/torch.linalg.vector_norm(heading).clamp_min(.1)-
                    head_heading/torch.linalg.vector_norm(head_heading).clamp_min(.1)).square().sum()
            loss+=.08*(pose.reshape(23,3)[11]).square().sum()
            for joint,target in targets.items():loss+=50*(joints[joint]-target).square().sum()
            for joint,direction in directions.items():
                axis=self.rest[joint+2]-self.rest[joint]
                axis=axis/torch.linalg.vector_norm(axis)
                loss+=.08*(R[joint]@axis-direction).square().sum()
            # Anatomical leg constraints are expressed on the resulting
            # joints, not on individual axis-angle components. This prevents
            # a valid optimizer solution from becoming a backward knee or a
            # folded leg when the head height and hand targets conflict.
            for hip,knee_joint,ankle in ((1,4,7),(2,5,8)):
                upper=joints[hip]-joints[knee_joint]
                lower=joints[ankle]-joints[knee_joint]
                cosine=(upper*lower).sum()/(torch.linalg.vector_norm(upper).clamp_min(.05)*
                    torch.linalg.vector_norm(lower).clamp_min(.05))
                # 25°..175° knee interior angle; penalize both hyperextension
                # and folding beyond the stable seated range.
                loss+=1.8*(torch.relu(cosine-.9063)**2+torch.relu(-.9962-cosine)**2)
            torso=joints[15]-joints[0]
            loss+=.35*torch.relu(.32-torch.linalg.vector_norm(torso))**2
            # Missing limbs are priors, never copied observations from a
            # separately dimensioned handcrafted skeleton.
            loss+=.025*(pose-neutral).square().sum()
            loss+=.4*(pose.reshape(23,3)[[2,5,8]]).square().sum()
            loss+=.12*orient[:,[0,2]].square().sum()
            if not restarting:
                loss+=.6*(pose-pose0).square().sum()+.4*(orient-orient0).square().sum()
                if self.previous_joints is not None and self.previous_head is not None:
                    predicted=self.previous_joints+(head-self.previous_head)
                    # Stabilize inferred limbs in camera-relative position.
                    # Observed wrists keep their stronger same-frame loss.
                    loss+=8.*(joints-predicted).square().sum()
            # Contact is a soft, explicitly unobserved local floor prior.
            sole=torch.stack([out.vertices[0,ids,1].min() for ids in self.feet])
            loss+=2500*(sole-self.ground).square().sum()
            # Contact by a single toe vertex is not a planted foot. Keep the
            # sole orientation close to the ground while allowing foot yaw.
            up=torch.tensor([0.,1.,0.])
            loss+=1.5*((R[10][:,1]-up).square().sum()+(R[11][:,1]-up).square().sum())
            loss.backward();opt.step()
            with torch.no_grad():
                p=pose.reshape(23,3);p.copy_(torch.maximum(torch.minimum(p,self.bounds[:,:,1]),self.bounds[:,:,0]))
                orient[:,0].clamp_(-.55,.55);orient[:,2].clamp_(-.55,.55)
        with torch.no_grad():
            out,R=self.forward(orient,pose,trans,head_R,head)
            vertices=out.vertices[0].numpy()@C
            joints=out.joints[0,:24].numpy()@C
            rotations=np.einsum('ij,njk,kl->nil',C,R.numpy(),C)
            head_error=float(torch.linalg.vector_norm(out.vertices[0,self.head_vertex]-head))
            errors={str(j):float(torch.linalg.vector_norm(out.joints[0,j]-t)) for j,t in targets.items()}
            from scipy.spatial.transform import Rotation
            actual_angles=Rotation.from_matrix(out.local_rotations.numpy()).as_rotvec().astype(np.float32)
            actual_trans=out.transl_effective.numpy()[0]
            foot_error=[float(out.vertices[0,ids,1].min()-self.ground) for ids in self.feet]
            head_angle=float(np.degrees(Rotation.from_matrix(R[15].numpy()@head_R.numpy().T).magnitude()))
            head_local_angle=float(np.linalg.norm(actual_angles[15]))
        self.previous=(orient.detach().clone(),pose.detach().clone(),torch.tensor(actual_trans[None,:]));self.reference=reference
        self.previous_joints=out.joints[0,:24].detach().clone();self.previous_head=head.detach().clone()
        return {'vertices':vertices,'joints':joints,'rotations':rotations,
            'global_orient':actual_angles[0],'body_pose':actual_angles[1:].reshape(69),
            'transl':actual_trans,'betas':self.betas.numpy()[0],
            'head_error_m':head_error,'wrist_errors_m':errors,
            'foot_contact_errors_m':foot_error,'head_angle_error_deg':head_angle,
            'head_local_angle_deg':float(np.degrees(head_local_angle)),
            'head_anchor_version':HEAD_ANCHOR_VERSION,
            'ground_y_m':-self.ground,'head_anchor_vertex':self.head_vertex}
