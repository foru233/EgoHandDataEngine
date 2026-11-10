"""MANO 在独立进程计算，避免 PyTorch 与 GUI 环境互相覆盖。"""
import json
import logging
from pathlib import Path
import numpy as np
import yaml
from PySide6.QtCore import QObject,QProcess,QTimer
from ..core.pipeline import ROOT

log=logging.getLogger(__name__)

class ManoController(QObject):
    def __init__(self,window):
        super().__init__(window)
        self.window=window;self.process=None;self.busy=False;self.current=None
        self.generation=0;self.last_key=None;self.buffer=b''
        self.cfg=yaml.safe_load((ROOT/'configs'/'mano.yaml').read_text(encoding='utf8'))

    def reset(self):
        self.generation+=1;self.current=None;self.last_key=None
        self.window.skeleton.set_mano_meshes([],[],redraw=True)

    def update(self,result,frame):
        show_pred=self.window.mano_prediction_action.isChecked()
        show_truth=self.window.mano_truth_action.isChecked()
        if not show_pred and not show_truth:
            if self.current is not None:self.reset()
            return
        sequence=f'{self.generation}:{result.session_id}:{result.source_type}:{result.calibration_id}'
        # 同一帧因参数变化可能被重新处理；对象身份区分新旧骨架。
        key=(sequence,result.frame_id,show_pred,show_truth,id(result))
        if self.current is not None and key!=self.current[0]:
            self.window.skeleton.set_mano_meshes([],[],redraw=False)
        self.current=(key,result,frame)
        if key!=self.last_key and not self.busy:self._submit()

    def _submit(self):
        if self.current is None or self.busy:return
        key,result,frame=self.current
        if key==self.last_key:return
        if not self._ensure_process():return
        hands=[]
        for hand in result.hands:
            if result.diagnostics.get('result_kind')=='official_ground_truth':continue
            xyz=np.asarray([j.xyz_m for j in hand.joints],float)
            valid=np.asarray([j.valid for j in hand.joints],bool)&np.isfinite(xyz).all(axis=1)
            hands.append({'side':hand.handedness,'track_id':hand.track_id,
                          'points':np.where(valid[:,None],xyz,0).tolist(),'valid':valid.tolist()})
        request={'sequence':key[0],'frame':key[1],'prediction':key[2],'ground_truth':key[3],'request_id':key[4],
                 'hands':hands,'truth':frame.metadata.get('ground_truth_mano',[]) if frame else []}
        self.last_key=key;self.busy=True
        self.window.statusBar().showMessage(f'MANO：正在计算第 {result.frame_id} 帧；播放较快时可暂停查看')
        self.process.write((json.dumps(request,allow_nan=False)+'\n').encode('utf8'))

    def _ensure_process(self):
        if self.process and self.process.state()!=QProcess.NotRunning:return True
        python=Path(self.cfg['python']); script=ROOT/'scripts'/'mano_worker.py'
        if not python.is_file():
            self.window.statusBar().showMessage('MANO 不可用：请在 configs/mano.yaml 配置 PyTorch Python 路径')
            self.last_key=self.current[0];return False
        self.process=QProcess(self);self.process.setProgram(str(python));self.process.setArguments(['-u',str(script)])
        self.process.setWorkingDirectory(str(ROOT));self.process.readyReadStandardOutput.connect(self._read)
        self.process.readyReadStandardError.connect(self._stderr)
        self.process.errorOccurred.connect(self._failed);self.process.finished.connect(self._finished)
        self.process.start()
        return True

    def _stderr(self):
        data=bytes(self.process.readAllStandardError()).decode('utf8',errors='replace').strip()
        if data:log.warning('MANO: %s',data)

    def _failed(self,*_):
        self.busy=False
        self.window.statusBar().showMessage('MANO 进程启动或运行失败，请检查模型和运行环境')

    def _finished(self,*_):
        self.busy=False
        self.window.statusBar().showMessage('MANO 计算进程已退出，请检查日志')

    def _read(self):
        self.buffer+=bytes(self.process.readAllStandardOutput())
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            try:reply=json.loads(line)
            except (ValueError,UnicodeDecodeError):
                self.busy=False;self.window.statusBar().showMessage('MANO 返回格式错误');continue
            self.busy=False
            if 'error' in reply:
                self.window.statusBar().showMessage('MANO 失败：'+reply['error'])
            elif self.current and (reply.get('sequence'),reply.get('frame'))==self.current[0][:2] and reply.get('request_id')==self.current[0][4] and self.last_key==self.current[0]:
                self.window.skeleton.set_mano_meshes(reply.get('predicted',[]),reply.get('truth',[]))
                details=reply.get('errors',[])
                counts=f"预测 {len(reply.get('predicted',[]))} · 官方真值 {len(reply.get('truth',[]))}"
                self.window.statusBar().showMessage('MANO：'+counts+(' · '+'；'.join(details) if details else ''))
            else:
                self.window.statusBar().showMessage('MANO：数据帧已变化，丢弃旧结果；暂停可查看拟合')
            if self.current and self.current[0]!=self.last_key:QTimer.singleShot(0,self._submit)

    def close(self):
        if self.process and self.process.state()!=QProcess.NotRunning:
            self.process.terminate()
            if not self.process.waitForFinished(1000):self.process.kill();self.process.waitForFinished(1000)
