# Battery Disassembly Task - 完整使用指南

LLM驱动的机器人电池拆卸系统 - Kinova Gen3 + Robotiq 2F-85

---

## 🚀 快速开始（3种方式）

### 方式1：最简单测试（Mock模式，无需ROS2）

```bash
cd battery_dismantle_task/LLM_Robot_Agent
python3 main.py --mock
```

**说明：** 仅测试Python逻辑，不需要ROS2

---

### 方式2：ROS2仿真执行（推荐）

```bash
# 终端1：启动ROS2系统（自动包含move_group + rviz + skill_server）
ros2 launch battery_dismantle_task llm_agent.launch.py

# 等待看到：✅ Skill Server Ready!

# 终端2：运行LLM Agent
cd battery_dismantle_task/LLM_Robot_Agent
python3 main.py
```

**说明：** 完整的ROS2仿真，包含运动规划和可视化

---

### 方式3：使用LLM理解自然语言指令

```bash
# 终端1：启动ROS2系统
ros2 launch battery_dismantle_task llm_agent.launch.py

# 终端2：设置API密钥并运行
cd battery_dismantle_task/LLM_Robot_Agent
export OPENAI_API_KEY="your-api-key-here"
python3 main.py --llm "carefully remove the battery cover"
```

**说明：** LLM会理解自然语言并生成拆卸计划

---

## 📋 如何输入指令

### 输入方式汇总

| 方式 | 命令 | 说明 |
|------|------|------|
| 默认Demo | `python3 main.py` | 使用预设的拆卸计划 |
| 自定义任务 | `python3 main.py "remove top cover"` | 自然语言描述（目前使用Demo计划） |
| LLM规划 | `python3 main.py --llm "task description"` | LLM理解并生成计划 |
| Mock测试 | `python3 main.py --mock` | 不需要ROS2的快速测试 |
| 完整拆卸 | `python3 main.py --complete` | 7步完整拆卸流程 |

---

## 📁 项目结构

```
battery_dismantle_task/
├── 📄 README.md                      # 主文档（MTC和技术细节）
├── 📄 USAGE_GUIDE.md                 # 本文件 - 使用指南
│
├── 🚀 launch/                        # ROS2启动文件
│   ├── llm_agent.launch.py          # 【推荐】完整LLM Agent系统
│   ├── fake_execution.launch.py     # 【也可用】包含skill_server
│   └── ...
│
├── ⚙️  config/                       # 配置文件
│   ├── waypoints.json               # 机器人位姿和对象定义
│   ├── moveit_controllers.yaml      # 控制器配置
│   └── ...
│
├── 🐍 battery_dismantle_task/LLM_Robot_Agent/  # Python LLM Agent
│   ├── 📄 README.md                 # Python模块详细文档
│   ├── main.py                      # 【入口】主程序
│   ├── planner.py                   # 任务规划器
│   ├── executor.py                  # ROS2执行器
│   ├── validator.py                 # 计划验证器
│   ├── run_demo.sh                  # 交互式测试脚本
│   └── outputs/                     # 执行日志目录
│
├── 🔧 src/                          # C++ ROS2节点
│   ├── skill_server_node.cpp        # 技能服务器（接收Python命令）
│   ├── DismantleTask.cpp            # MTC拆卸任务
│   └── ...
│
└── 📚 include/                      # C++头文件
```

---

## 🎯 完整的拆卸流程

### 默认计划（4步）- 拆卸顶盖螺栓

```
Step 1: grasp(TopCoverBolts)
   → 打开夹爪
   → 移动到approach位姿（自动）
   → 关闭夹爪并抓取
   → 附加物体到夹爪

Step 2: moveTo(place_bolts)
   → 携带螺栓移动到放置位置

Step 3: release(TopCoverBolts)
   → 打开夹爪
   → 分离物体

Step 4: moveTo(HOME)
   → 返回初始位姿
```

### 完整计划（7步）- 拆卸顶盖+主电池

```bash
python3 main.py --complete
```

执行步骤：
1-3. 拆卸TopCoverBolts（如上）
4-6. 拆卸BatteryBox_0
7. 返回HOME

---

## 🔧 可用技能

系统支持3种基本技能：

### 1. grasp(object_name)
**功能：** 抓取物体（复合技能）

**自动执行：**
- 打开夹爪 → OPEN位姿
- 移动到approach位姿（从waypoints.json读取）
- 关闭夹爪 → CLOSE位姿
- 附加物体到夹爪

**示例：** `{"skill": "grasp", "target": "TopCoverBolts"}`

### 2. moveTo(pose_name)
**功能：** 移动机械臂到指定位姿

**可用位姿：**
- `HOME` - 初始位姿
- `place_bolts` - 放置螺栓的位置
- `approach_bolts` - 接近螺栓的位置（通常由grasp自动使用）

**示例：** `{"skill": "moveTo", "target": "HOME"}`

### 3. release(object_name)
**功能：** 释放物体

**自动执行：**
- 打开夹爪
- 从夹爪分离物体

**示例：** `{"skill": "release", "target": "TopCoverBolts"}`

---

## 📊 查看执行结果

所有执行结果保存在：
```
battery_dismantle_task/LLM_Robot_Agent/outputs/
├── plan.json              # 生成的拆卸计划
├── execution.log          # 详细执行日志
└── run_history.log        # 历史运行记录
```

查看日志：
```bash
cd battery_dismantle_task/LLM_Robot_Agent/outputs
cat execution.log
```

---

## 🐛 故障排查

### 问题1：Python端显示"Timeout"

**原因：** skill_server没有运行

**解决：**
```bash
# 检查skill_server是否运行
ros2 node list | grep skill_server

# 如果没有，检查launch文件是否包含skill_server
# 使用llm_agent.launch.py或fake_execution.launch.py
```

---

### 问题2：Launch文件启动失败

**可能原因：**
- 未编译package
- MoveIt 2未安装

**解决：**
```bash
# 重新编译
cd ~/your_workspace
colcon build --packages-select battery_dismantle_task --symlink-install

# 确认MoveIt 2已安装
ros2 pkg list | grep moveit
```

---

### 问题3：Planning失败

**原因：** waypoints不可达或发生碰撞

**解决：**
1. 在RViz中检查机器人模型
2. 查看MoveIt规划日志
3. 调整 `config/waypoints.json` 中的位姿

---

### 问题4：Mock模式正常，ROS2模式失败

**检查清单：**
```bash
# 1. 确认所有ROS2节点都在运行
ros2 node list
# 应该看到: /move_group, /rviz2, /skill_server

# 2. 检查话题
ros2 topic list | grep llm
# 应该看到: /llm_commands, /llm_feedback

# 3. 查看skill_server日志
ros2 node info /skill_server
```

---

## 🎓 进阶使用

### 添加新的拆卸对象

1. 编辑 `config/waypoints.json`：
```json
"objects": {
  "MyNewObject": {
    "approach": [0.1, 0.5, 3.0, -1.6, -3.1, 0.8, 0.1],
    "place": [0.6, 1.0, -3.1, -1.2, -3.1, 0.9, 0.6],
    "retreat": [0.0, 0.26, 3.14, -2.27, 0.0, 0.96, 1.57],
    "gripper_hooks": {
      "on_approach": "OPEN",
      "after_approach": "CLOSE",
      "after_place": "OPEN"
    },
    "io": {
      "do_attach_after_approach": true,
      "do_detach_after_place": true
    }
  }
}
```

2. 更新 `battery_dismantle_task/LLM_Robot_Agent/config/skills.json`：
```json
"available_objects": [
  "TopCoverBolts",
  "BatteryBox_0",
  "MyNewObject"
]
```

3. 在planner.py中使用新对象

---

### 使用LLM API

支持OpenAI和Anthropic：

```bash
# OpenAI
export OPENAI_API_KEY="sk-..."
python3 main.py --llm "remove all battery components"

# Anthropic Claude
export ANTHROPIC_API_KEY="sk-ant-..."
python3 main.py --llm "disassemble carefully"
```

---

## 📖 相关文档

- **主README** - `README.md` - MTC架构和技术细节
- **Python模块文档** - `battery_dismantle_task/LLM_Robot_Agent/README.md`
- **MoveIt 2文档** - https://moveit.picknik.ai/humble/
- **Kinova Gen3文档** - https://github.com/Kinovarobotics/ros2_kortex

---

## 🎉 成功标志

当你看到以下输出，说明系统正常运行：

```
============================================================
🎉 MISSION ACCOMPLISHED
============================================================
✅ All steps completed successfully!
📊 Total steps executed: 4
============================================================
```

---

## 📞 获取帮助

如有问题，请检查：
1. `outputs/execution.log` - 执行日志
2. `ros2 node list` - 确认所有节点运行
3. RViz中的机器人可视化

---

**作者：** Olivia
**License：** Apache 2.0
**版本：** 0.1.0
