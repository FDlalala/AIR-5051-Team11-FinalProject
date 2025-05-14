# 多模态语音-视觉机器人抓取系统

本项目实现了一个集语音识别、视觉跟踪和机械臂控制于一体的智能机器人抓取系统。系统接收语音指令（支持中文、英文、德文），通过视觉模块扫描并识别颜色目标，最终控制机械臂执行抓取任务。

---

## 功能模块

本系统包括以下几个核心模块：

### 1. 语音识别模块 (`voice_module.py`)

* **功能**：基于 Whisper 模型实现语音识别与翻译功能。
* **支持语言**：中文、英文、德文。
* **模型路径**：本地模型位于 `models/faster-whisper-large-v3`。

### 2. 视觉识别模块 (`vision_module.py`)

* **功能**：实时捕获摄像头画面，通过HSV颜色空间进行颜色目标检测，稳定位置识别并转换为世界坐标。
* **标定文件**：`config/vision_config.yaml`

### 3. 机械臂控制模块 (`robot_controller.py` & `pick_demo.py`)

* **功能**：封装机械臂基本动作控制（移动、夹爪开合），提供抓取和丢弃等任务规划能力。
* **控制工具**：ROS + MoveIt

### 4. 高级控制模块 (`voice_scan_robot.py`)

* **功能**：语音识别后根据指定颜色生成扫描轨迹，视觉识别目标后进行抓取控制。
* **轨迹生成**：动态扫描路径，视觉识别成功后实时控制抓取。

### 5. 测试模块 (`test_voice.py`, `test_vision.py`)

* **功能**：独立测试语音和视觉模块功能，便于调试。

---

## 系统依赖

* ROS Noetic
* MoveIt
* Python 3.8+
* PyTorch
* Faster-Whisper
* OpenCV
* NumPy

---

## 项目目录结构

```
final_project
├── models
│   ├── faster-whisper-large-v3
│   └── Helsinki-NLP (翻译模型)
├── config
│   └── vision_config.yaml (视觉标定参数)
├── test_data
│   └── colour (语音测试文件)
├── voice_module.py
├── vision_module.py
├── robot_controller.py
├── pick_demo.py
├── voice_scan_robot.py
└── test
    ├── test_voice.py
    └── test_vision.py
```

---

## 如何运行

### Step 1: 环境准备

```bash
pip install torch torchvision torchaudio faster-whisper opencv-python numpy pyyaml transformers
```

### Step 2: ROS环境

确保ROS环境已正确安装和配置，启动机械臂驱动和MoveIt服务。

### Step 3: 测试基础模块

* 测试语音识别：

```bash
python test_voice.py
```

* 测试视觉识别：

```bash
python test_vision.py
```

### Step 4: 运行机器人抓取（语音+视觉扫描）

执行语音扫描机器人控制程序，语音指令示例如“红色”、“绿色”、“蓝色”：

```bash
python voice_scan_robot.py /path/to/your/audio.wav
```

若不指定文件，将自动遍历测试文件夹下所有语音文件：

```bash
python voice_scan_robot.py
```

---

## 机械臂移动轨迹（视觉跟踪）

机器人在收到语音指令后，不使用固定坐标，而是根据视觉实时识别结果动态确定目标位置：

1. 接收语音指令，识别颜色。
2. 移动到扫描起始位置，摄像头从绿色到蓝色方向扫描。
3. 在扫描路径中实时视觉检测目标颜色。
4. 视觉模块输出稳定的目标世界坐标。
5. 控制机械臂移动到识别位置进行抓取，抓取成功后移动到指定丢弃点。

---

## 注意事项

* 运行程序前请确认机械臂与摄像头已正确连接。
* 视觉标定参数如有变动需更新 `vision_config.yaml`。
* Whisper 模型和翻译模型较大，首次运行加载可能耗时较长。

---

## 未来扩展方向

* 优化视觉稳定性和识别准确度。
* 增加更多颜色识别和动态路径规划。
* 集成其他模态输入，如深度视觉或触觉反馈。

