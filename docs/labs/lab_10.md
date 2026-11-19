# Lab 10 — Visão Computacional 2FA Real: Pinhole `f=h·D/H`, `ArUco DICT_6X6_1000`, HSV Anti-Spoofing & `DangerDetector IoU`

- **Origens Integradas:** `robotica-computacional Mod 04, 05, 06, 404/`, `Robotics-Projects/Surveillance Robot`, `Introducao-a-robotica`.

## Roteiro Executivo
1. Calibrar distância focal usando `CameraCalibrator.calibrate_from_beacons()` (`f = h_px * D / H`) ou `load_realsense_defaults()` em [camera_calib.py](../../python/aegis_ai/vision_2fa/camera_calib.py).
2. Renderizar e validar crachá vermelho com marcador `ArUco DICT_6X6_1000 ID=42` em [aruco_pose.py](../../python/aegis_ai/vision_2fa/aruco_pose.py) (`solvePnP IPPE_SQUARE` + máscara HSV wrap-around `[170..180]|[0..10]`).
3. Injetar ataque com **crachá em escala de cinza (P&B)** confirmando rejeição `authorized == False` (`color_ratio < 0.08`).
4. Acionar `/poweron_yolo` em [yolo_ondemand.py](../../python/aegis_ai/vision_2fa/yolo_ondemand.py) testando *tailgating* (`person_count == 2`) e ferramenta perigosa (`DangerDetector IoU > 0.0`).
5. Verificar alinhamento visual do ferrolho com `auto_canny(0.33) + HoughLinesP` em [optical_tracker.py](../../python/aegis_ai/vision_2fa/optical_tracker.py).

## Quality Gate Numérico
- Erro de distância `solvePnP` $\le 15\,\text{mm}$ a `300 mm`; bloqueio `100%` de foto P&B e de *tailgating*; execução pesada `YOLO` desligada em repouso.
