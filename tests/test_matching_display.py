import numpy as np
from egohand.core.types import HandDetection,Hand3D
from egohand.visualization.image_view import ImageView


def test_accepted_pair_uses_track_colour_not_model_handedness(qtbot):
    view=ImageView('test');qtbot.addWidget(view)
    image=np.zeros((240,320,3),np.uint8)
    uv=np.tile([160.,130.],(21,1));left=HandDetection('Left',.9,uv/[320,240],uv)
    right=HandDetection('Right',.9,uv/[320,240],uv)
    h=Hand3D('Left',.9,1);h.valid_joint_count=10
    view.display(image,[left],[h],side='left',accepted_pairs=[[0,0]])
    a=view.last_bgr[130,160].copy()
    view.display(image,[right],[h],side='right',accepted_pairs=[[0,0]])
    b=view.last_bgr[130,160].copy()
    np.testing.assert_array_equal(a,b)
    view.display(image,[right],[h],side='right',accepted_pairs=[])
    np.testing.assert_array_equal(view.last_bgr[130,160],[61,218,255])
    assert not np.array_equal(a,view.last_bgr[130,160])
