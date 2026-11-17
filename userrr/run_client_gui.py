r"""Run with the existing project Python: .venv\Scripts\python.exe userrr\run_client_gui.py."""
import argparse
import os
import sys
import importlib.util
from pathlib import Path

def main(*, on_window=None):
    root=Path(__file__).resolve().parents[1];os.chdir(root);sys.path.insert(0,str(root/'src'));sys.path.insert(0,str(Path(__file__).resolve().parent))
    parser=argparse.ArgumentParser();parser.add_argument('--demo',action='store_true');parser.add_argument('--quit-after',type=float)
    parser.add_argument('--open-results',type=Path)
    parser.add_argument('--offline-recovery-candidate',action='store_true',help='显式启用隔离离线恢复候选，不改变正式版本')
    parser.add_argument('--offline-recovery-fast-candidate',action='store_true',help='显式启用0.5秒稳定窗口离线候选，不改变正式版本')
    parser.add_argument('--offline-recovery-profile',type=Path,help='显式载入经哈希校验的隔离离线候选清单')
    parser.add_argument('--live-recovery-candidate',action='store_true',help='显式启用 VS030 v4 实时恢复候选，不改变正式版本')
    parser.add_argument('--recording',type=Path);parser.add_argument('--start',action='store_true')
    parser.add_argument('--output-dir',type=Path);parser.add_argument('--exit-when-finished',action='store_true')
    args=parser.parse_args()
    offline_choices=sum(bool(v) for v in (args.offline_recovery_candidate,args.offline_recovery_fast_candidate,args.offline_recovery_profile))
    if args.demo and (offline_choices or args.live_recovery_candidate or args.recording):parser.error('--demo 不能与候选一起使用')
    if offline_choices and args.live_recovery_candidate:parser.error('离线候选与实时候选只能选择一个')
    if offline_choices>1:parser.error('离线候选只能选择一个')
    if args.live_recovery_candidate and args.recording:parser.error('--live-recovery-candidate 不接受录制回放，请直接连接 VS030')
    if args.start and not (args.recording or args.demo or args.live_recovery_candidate):parser.error('--start 需要 --recording、--demo 或实时候选入口')
    if args.exit_when_finished and not (args.start or args.demo):parser.error('--exit-when-finished 需要自动启动')
    profile=None
    if offline_choices:
        from offline_recovery import OfflineRecoveryProfile
        manifest = args.offline_recovery_profile or (Path(__file__).resolve().parent/'runtime_configs/offline_recovery_fast_candidate.json'
                    if args.offline_recovery_fast_candidate else None)
        profile=OfflineRecoveryProfile(manifest=manifest) if manifest else OfflineRecoveryProfile()
        profile.configure_process()
    live_profile=None
    if args.live_recovery_candidate:
        from egohand.vio.live_recovery import LiveRecoveryProfile
        live_profile=LiveRecoveryProfile()
    from PySide6.QtCore import Qt,QCoreApplication,QTimer
    from PySide6.QtWidgets import QApplication
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app=QApplication(sys.argv[:1]);app.setStyle('Fusion')
    # Bind the delivery entry point to its sibling file, regardless of import paths.
    client_path=Path(__file__).resolve().with_name('client_gui.py')
    spec=importlib.util.spec_from_file_location('egohand_delivery_client',client_path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=module
    spec.loader.exec_module(module)
    window=module.ClientMainWindow(recovery_profile=profile,live_recovery_profile=live_profile,replay_output_dir=args.output_dir)
    if on_window:on_window(window)
    print(f'[EgoHand DELIVERY] {client_path} -> {type(window).__name__}',flush=True)
    window.show()
    if args.recording:window.select_recording(args.recording)
    if args.open_results:window.open_results_path(args.open_results)
    if args.exit_when_finished:
        window.session.finished.connect(lambda:QTimer.singleShot(1000,window.close))
    if args.start:
        def start():
            window.start_pipeline()
            if args.exit_when_finished and getattr(window,'last_error',None):window.close()
        QTimer.singleShot(200,start)
    if args.demo:
        window.enable_demo();QTimer.singleShot(200,window.start_pipeline)
    if args.quit_after:QTimer.singleShot(int(args.quit_after*1000),window.close)
    code=app.exec()
    return code or (1 if getattr(window,'last_error',None) else 0)
if __name__=='__main__':sys.exit(main())
