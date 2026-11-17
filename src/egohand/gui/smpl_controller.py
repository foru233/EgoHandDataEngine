"""One active fit plus one latest request; reject results from older inputs."""
import json
import logging
from pathlib import Path
import yaml
from PySide6.QtCore import QObject,QProcess,QTimer,Signal
from ..core.pipeline import ROOT

log=logging.getLogger(__name__)


class SmplController(QObject):
    completed=Signal(object)
    failed=Signal(str)

    def __init__(self,parent=None):
        super().__init__(parent)
        self.process=None;self.asset=None;self.current=None;self.active=None
        self.buffer=b'';self.serial=0;self.last_sent=None;self.closing=False
        self.timeout=QTimer(self);self.timeout.setSingleShot(True);self.timeout.setInterval(45000)
        self.timeout.timeout.connect(lambda:self._failure('SMPL 计算超时'))

    @property
    def pending(self):return self.current is not None and (self.current['request_id']!=self.last_sent or self.active is not None)

    def request(self,payload):
        self.serial+=1;self.current={**payload,'request_id':self.serial}
        if self._ensure():self._submit()
        return self.serial

    def reset(self):
        self.serial+=1;self.current=None

    def _ensure(self):
        if self.process and self.process.state()!=QProcess.NotRunning:return True
        cfg=yaml.safe_load((ROOT/'configs/mano.yaml').read_text(encoding='utf8'))
        python=Path(cfg['python'])
        if not python.is_file():self._failure('SMPL Python 不可用');return False
        self.asset=None;self.buffer=b''
        self.process=QProcess(self);self.process.setProgram(str(python))
        self.process.setArguments(['-u',str(ROOT/'scripts/smpl_worker.py')]);self.process.setWorkingDirectory(str(ROOT))
        self.process.readyReadStandardOutput.connect(self._read)
        self.process.readyReadStandardError.connect(self._stderr)
        self.process.errorOccurred.connect(self._process_error)
        self.process.finished.connect(self._finished)
        self.timeout.start();self.process.start();return True

    def _submit(self):
        if self.current is None or self.active is not None or self.asset is None:return
        if self.current['request_id']==self.last_sent:return
        self.active=self.current;self.last_sent=self.active['request_id'];self.timeout.start()
        self.process.write((json.dumps(self.active,allow_nan=False)+'\n').encode('utf8'))

    def _read(self):
        self.buffer+=bytes(self.process.readAllStandardOutput())
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            try:reply=json.loads(line)
            except (ValueError,UnicodeDecodeError):continue
            if reply.get('type')=='ready':
                self.asset=reply['asset'];self.timeout.stop();self._submit();continue
            if self.active is None or reply.get('request_id')!=self.active['request_id']:continue
            finished=self.active;self.active=None;self.timeout.stop()
            stream_compatible=(finished.get('streaming') and self.current and
                finished.get('stream_session')==self.current.get('stream_session') and
                finished.get('reference')==self.current.get('reference') and
                finished.get('ground_y_m')==self.current.get('ground_y_m'))
            if self.current and (reply.get('request_id')==self.current['request_id'] or stream_compatible):
                reply['streaming']=bool(stream_compatible)
                if reply.get('type')=='error':self.failed.emit(reply.get('error','SMPL 失败'))
                else:self.completed.emit(reply)
            self._submit()

    def _stderr(self):
        data=bytes(self.process.readAllStandardError()).decode('utf8',errors='replace').strip()
        if data:log.warning('SMPL: %s',data[-2000:])

    def _process_error(self,*_):
        self._failure('SMPL 进程错误')

    def _finished(self,*_):
        if not self.closing:self._failure('SMPL 进程退出')

    def _failure(self,message):
        self.timeout.stop();self.active=None;self.current=None
        process=self.process;self.process=None;self.asset=None
        if process is not None:
            process.blockSignals(True)
            if process.state()!=QProcess.NotRunning:
                process.kill();process.waitForFinished(1000)
            process.deleteLater()
        if not self.closing:self.failed.emit(message)

    def close(self):
        self.closing=True;self.reset();self.timeout.stop()
        if self.process and self.process.state()!=QProcess.NotRunning:
            self.process.terminate()
            if not self.process.waitForFinished(1000):self.process.kill();self.process.waitForFinished(1000)
