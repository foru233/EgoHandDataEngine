import os, sys
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
sys.path.insert(0, os.path.abspath('userrr'))
from PySide6.QtWidgets import QApplication
import numpy as np
from client_gui import ClientMainWindow
app=QApplication.instance() or QApplication([])
w=ClientMainWindow(); w.resize(1280,760); w.show()
I=np.eye(4).tolist(); w.vio_panel.begin_live()
w.on_vio({'type':'pose','timestamp_ns':1,'frame_id':1,'tracking_state':'OK','pose_valid':True,'imu_initialized':True,'map_epoch':1,'matrix':I})
body=np.eye(4); body[0,3]=.8; body[1,3]=.25
w.on_vio({'type':'pose','timestamp_ns':2,'frame_id':2,'tracking_state':'LOST','pose_valid':False,'imu_initialized':True,'map_epoch':1,'matrix':None,
 'display_pose_contract':{'T_body_from_camera':I,'camera_frame':'gui_left_camera','body_frame':'imu_body'},
 'recovery_preaccept':{'loss_prediction':{'status':'predicted','duration_s':.50,'T_world_from_body':body.tolist()}}})
app.processEvents(); w.grab().save('handoff/client_gui_best_effort_pose_20261114/client_gui_imu_prediction_simulated.png'); w.close()
