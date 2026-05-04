# ROS2 Leg Pose Tracking Project Plan

## 目標

建立一套 ROS2 腿部關節角度追蹤系統，使用 OpenPose 從正面與側面相機影像估測腿部骨架，結合深度資訊計算 4 個角度，並透過 GUI 顯示影像、骨架與角度，同時 publish 給其他控制程式使用。

## 追蹤角度

1. 大腿抬伸角度：hip flexion/extension。
2. 屈膝角度：knee flexion，定義為大腿與小腿向量夾角。
3. 腳踝自由度 1：ankle dorsiflexion/plantarflexion。
4. 腳踝自由度 2：ankle inversion/eversion 或 abduction/adduction，依相機配置與標定結果決定。

## 階段 0：需求定義與座標定義

- 定義追蹤單腿或雙腿；若雙腿皆追蹤，topic message 需支援 left/right。
- 定義正面相機與側面相機的安裝位置、方向、距離、視角與同步方式。
- 定義人體座標系、相機座標系、ROS frame tree。
- 定義 4 個角度的正負方向、單位、合理範圍、失效值與置信度規則。
- 產出：`docs/system_spec.md`、角度定義圖、topic/message 草案。

## 階段 1：ROS2 Workspace 與 Package 架構

建議 package：

- `leg_pose_msgs`：自訂 message。
- `leg_pose_camera`：ZED 2i 影像與深度輸入封裝。
- `leg_pose_openpose`：OpenPose 推論 node。
- `leg_pose_fusion`：2D/3D keypoint 融合、濾波與角度計算。
- `leg_pose_gui`：Qt/RViz-like GUI 顯示雙畫面、骨架、角度。
- `leg_pose_bringup`：launch、parameter、calibration config。

主要任務：

- 建立 `colcon` workspace 與 ROS2 package。
- 決定 ROS2 版本，建議 Humble 或 Jazzy，依 Ubuntu 版本配合。
- 建立 CI 或至少本機 lint/test 指令。
- 產出：可 build 的空 package skeleton。

## 階段 2：ZED 2i 相機輸入與標定

- 使用 `zed-ros2-wrapper` 取得 RGB、depth、camera_info、TF。
- 使用兩台 ZED 2i，分別作為 `front_camera` 與 `side_camera`。
- 驗證 frame rate、depth range、timestamp 與側面/正面影像同步。
- 相機內參由 ZED SDK/driver 提供。
- 相機外參需標定，建立 `front_camera_frame`、`side_camera_frame` 到 `base_link` 或 `leg_tracking_frame` 的 TF。
- 使用 checkerboard/AprilTag/手動量測作初版外參，後續再優化。
- 產出：ZED 2i camera launch、TF tree、標定 YAML。

## 階段 3：OpenPose 整合

任務：

- 建立 OpenPose 推論 node，訂閱 front/side RGB image。
- 輸出每個畫面的 2D keypoints、confidence、timestamp。
- 優先抽取下肢關鍵點：hip、knee、ankle、heel、big toe、small toe。
- 若 OpenPose 不穩，評估 BODY_25 model，因腳部點位較完整。
- 對 OpenPose output 做 ROS message 化，不讓後續 node 依賴 OpenPose 原始資料結構。

產出 topics：

- `/leg_pose/front/keypoints_2d`
- `/leg_pose/side/keypoints_2d`
- `/leg_pose/front/skeleton_overlay`
- `/leg_pose/side/skeleton_overlay`

## 階段 4：3D Keypoint 融合與濾波

任務：

- 用 2D keypoint + depth image back-project 成 3D camera point。
- 用 TF 將 front/side 3D keypoint 轉到共同 frame。
- 根據 confidence、depth validity、view angle 選擇或融合 keypoint。
- 加入濾波：One Euro Filter、Kalman filter 或低通濾波。
- 定義失效策略：短暫 occlusion 使用 prediction，長時間 occlusion publish invalid confidence。
- 提供 synthetic 3D keypoint demo，讓無 ZED/OpenPose 時也能測試 angle estimator、GUI 與控制端 topic。

產出：

- `/leg_pose/keypoints_3d`
- `/leg_pose/debug/fusion_status`

## 階段 5：角度計算

任務：

- 根據 3D keypoint 計算四個角度。
- 大腿抬伸：以 pelvis/hip 到 knee 向量相對 trunk/pelvis reference plane 或 calibration neutral pose 計算。
- 屈膝角度：`angle(thigh_vector, shank_vector)`，依臨床或控制需求轉成 0 度伸直、正值屈曲。
- 腳踝 dorsiflexion/plantarflexion：小腿向量與 foot vector 在 sagittal plane 投影後計算。
- 腳踝第二自由度：foot vector 在 frontal/transverse plane 投影後計算，需靠正面相機與足部 keypoints 提高穩定性。
- 加入 per-angle confidence、range clamp、速度限制與 timestamp。
- 提供 neutral pose capture service，將目前 raw angle 作為 zero offset，並支援 YAML 持久化。
- 提供低通濾波參數，先以 cutoff frequency 調校 jitter/latency tradeoff。

產出 topic：

- `/leg_pose/joint_angles`

## 階段 6：GUI

任務：

- GUI 顯示正面與側面兩個 live view。
- 每個 view 疊加 OpenPose skeleton、關鍵點 confidence、追蹤狀態。
- 顯示四個角度數值、單位、有效/無效狀態、confidence 與最近 5-10 秒時間曲線。
- 提供角度曲線切換：單一角度、全部角度、依有效狀態隱藏 invalid data。
- 顯示追蹤品質面板：OpenPose FPS、angle publish rate、end-to-end latency、front/side timestamp difference、lost keypoints 數量、depth invalid ratio。
- 提供關鍵點 debug view：hip/knee/ankle/heel/toe 名稱、confidence、來源視角(front/side/fused)、valid state。
- 顯示相機與座標系狀態：front/side ZED 2i connection、TF availability、calibration file loaded、`leg_tracking_frame` transform status。
- 提供 neutral pose calibration：按鈕觸發 capture neutral pose，顯示 calibration timestamp 與有效狀態，並可儲存到 config。
- 提供 record/replay 控制：開始/停止 rosbag record、選擇 replay bag、live/replay 模式一致顯示。
- 提供 topic monitor：`/leg_pose/joint_angles` 最新 publish time、Hz、message preview、控制端 subscriber count。
- 提供 warnings/status：low confidence、invalid depth、camera desync、angle out of range、OpenPose FPS drop、TF lookup failed。
- 提供 debug overlay 開關：keypoint labels、confidence colors、depth validity、fusion source。
- 建議使用 Python Qt (`rclpy` + PySide6/PyQtGraph) 或 C++ Qt；若推論主體 Python 化，GUI 也用 Python 可降低整合成本。

產出：

- `/leg_pose_gui` executable。
- GUI launch config。
- GUI MVP：雙畫面 skeleton overlay、四角度即時數值、valid/confidence、angle publish rate、neutral pose calibration、rosbag record/replay 控制。

## 階段 7：資料紀錄、測試與驗證

任務：

- 建立 rosbag record profile：RGB/depth/keypoints/angles/tf。
- 建立 replay launch，讓無硬體時也能測試。
- 建立 demo launch，使用 synthetic keypoints 驗證 `/leg_pose/joint_angles` 與 GUI 顯示。
- 建立單元測試：角度計算、投影、TF transform、message schema。
- 建立整合測試：用 rosbag 驗證 end-to-end latency、角度穩定度、失效策略。
- 與人工量角器、IMU 或 motion capture 做初步精度比較。

驗收指標草案：

- GUI 兩路影像可穩定顯示 skeleton overlay。
- GUI 可顯示四個角度的數值、valid/confidence、5-10 秒趨勢曲線與 warning state。
- GUI 可執行 neutral pose capture，並顯示 calibration 狀態。
- GUI 可啟動/停止 rosbag record，並支援 replay 模式下顯示同樣資訊。
- `/leg_pose/joint_angles` publish rate >= 15 Hz，目標 30 Hz。
- 正常可見姿態下角度 jitter 小於指定門檻，例如 2-5 degrees RMS。
- 單幀推論到 angle publish latency 可量測並符合控制端需求。

## 主要風險

- OpenPose 對腳踝與足部 keypoints 的穩定度可能不足，尤其側面遮擋時。
- 雙相機外參誤差會直接影響 ankle 2DOF。
- 深度相機在反光、黑色材質、快速動作時可能有 invalid depth。
- 若控制程式需要低 latency，OpenPose GPU 推論與 GUI 顯示需分離 thread/process。

## 建議優先順序

1. 先做 rosbag-driven prototype，不依賴 GUI。
2. 先完成 knee angle 與 hip angle，確認 keypoint/depth/TF 管線。
3. 再加入 ankle 2DOF，因這部分對足部 keypoint 與外參最敏感。
4. 最後補 GUI 與 calibration workflow。
