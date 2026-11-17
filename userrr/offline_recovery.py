"""Explicit offline candidate profile; production launchers never enable it."""
import hashlib
import json
import os
from pathlib import Path

from egohand.calibration.loader import load_calibration
from egohand.vio.config_snapshot import load_config_snapshot

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'userrr/runtime_configs/offline_recovery_candidate.json'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


class OfflineRecoveryProfile:
    label = '离线恢复候选 v4 · 全分辨率 · 精度待验收'

    def __init__(self, manifest=PROFILE):
        self.manifest = Path(manifest).resolve()
        self.data = json.loads(self.manifest.read_text(encoding='utf8'))
        self.label = self.data.get('label', self.label)
        if self.data.get('schema_version') != 1:
            raise ValueError('不支持的离线候选配置版本')
        self.paths = {key: ROOT / value['path'] for key, value in self.data['files'].items()}
        self.verify_files()

    def verify_files(self):
        for key, path in self.paths.items():
            if sha(path) != self.data['files'][key]['sha256']:
                raise ValueError(f'离线候选文件校验失败：{path}')

    @property
    def environment(self):
        return {**self.data['environment'],
                'EGOHAND_VIO_LIBRARY': str(self.paths['library']),
                'EGOHAND_VIO_RECOVERY_CONFIG': str(self.paths['recovery'])}

    def configure_process(self):
        """Only the dedicated launcher calls this, before any workers start."""
        for key in list(os.environ):
            if key.startswith(('EGOHAND_VIO_', 'EGOHAND_TEST_')):
                del os.environ[key]
        os.environ.update(self.environment)

    def worker_options(self, recording, output, *, isolated_environment=False):
        self.verify_files()
        if not isolated_environment and any(os.environ.get(k) != v for k, v in self.environment.items()):
            raise ValueError('请使用离线候选专用启动入口，恢复策略尚未启用')
        recording, output = Path(recording).resolve(), Path(output).resolve()
        if output == recording or recording in output.parents or output in recording.parents:
            raise ValueError('候选结果必须保存在原始录制之外的独立目录')
        config = load_config_snapshot(calibration=self.paths['calibration'],
            settings=self.paths['settings'], timing=recording/'vio_timing.yaml',
            quality=self.paths['quality'], source='offline_recovery_candidate_v4',
            timing_override={'image_scale': 1., 'lattice_phase_mode': 'session_auto',
                             'session_phase_verified': False})
        config.validate_geometry()
        calibration = load_calibration(config.calibration, kalibr_left_camera='cam0')
        # An explicit DLL uses a fresh isolated process, never the production
        # persistent worker whose identity may describe a different policy.
        output.mkdir(parents=True, exist_ok=False)
        source_files = [*sorted((ROOT/'src/egohand').rglob('*.py')),
                        ROOT/'userrr/client_gui.py', ROOT/'userrr/run_client_gui.py', Path(__file__)]
        provenance = {'schema_version': 1, 'profile': self.label,
            'profile_sha256': sha(self.manifest), 'files': self.data['files'],
            'environment': self.environment, 'recording': str(recording),
            'config': config.describe(), 'hand_calibration_id': calibration.calibration_id,
            'source_sha256': {str(p.relative_to(ROOT)): sha(p) for p in source_files},
            'fresh_hand_inference': True,
            'production_deployed': bool(self.data.get('delivery_selected', False)),
            'release_id': self.data.get('release_id')}
        (output/'gui_launch.json').write_text(json.dumps(provenance, ensure_ascii=False, indent=2), encoding='utf8')
        options=dict(calibration=calibration, vio_config_snapshot=config,
                    vio_library_path=self.paths['library'], vio_worker=None)
        if isolated_environment:options['vio_environment']=self.environment
        return options
