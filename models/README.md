# Hand Landmarker 模型

执行 `.venv\Scripts\python.exe scripts\download_models.py` 下载 Google 官方 float16 version 1 模型。
默认路径 `models/hand_landmarker.task`；支持 `--output` 和 `--sha256`，下载信息与摘要写入同名 JSON。

官方地址：<https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task>

备用官方桶域名：<https://mediapipe-models.storage.googleapis.com/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task>

手动下载时将原文件放入上述路径，不应创建占位二进制。GUI 可加载其他路径的官方兼容 `.task`。
SHA256 是本地内容身份记录；只有提供可信的期望摘要时，才构成与期望版本的校验。
本次下载摘要见 `hand_landmarker.json`。推理使用 CPU，左右视图创建独立实例。

官方 API 与模型说明：<https://developers.google.com/edge/mediapipe/solutions/vision/hand_landmarker/python>。
IMAGE 用于单图，VIDEO 用于顺序数据和实时后台处理；回放定位时重建实例，毫秒时间戳不得重复。
模型的世界关节单独保存，绝不代替双目相机测量。
