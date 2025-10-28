# LLM Robot Agent - Battery Disassembly

一个用于电池拆卸的LLM驱动的机器人Agent系统。

## 系统架构

```
┌─────────────┐
│   用户输入   │ "disassemble the battery"
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  1. Planner │  生成任务计划（LLM或Demo）
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ 2. Validator│  验证计划的安全性和正确性
└──────┬──────┘
       │
       ▼
┌─────────────┐
│ 3. Executor │  通过ROS2执行计划
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  机器人执行  │  Kinova Gen3 + Robotiq gripper
└─────────────┘
```

## 文件说明

- **main.py** - 主入口，完整的Plan→Validate→Execute流程
- **planner.py** - 任务规划器，生成拆卸步骤
- **validator.py** - 计划验证器，检查安全性和可行性
- **executor.py** - 执行器，通过ROS2与机器人通信
- **llm_client.py** - LLM客户端（OpenAI/Claude）
- **run_demo.sh** - 快速测试脚本

## 快速开始

### 1. 最简单的测试（Mock模式，不需要ROS2）

```bash
cd battery_dismantle_task/LLM_Robot_Agent
python3 main.py --mock
```

这将运行一个完整的拆卸流程模拟，输出类似：

```
🤖 LLM Robot Agent for Battery Disassembly
============================================================
📝 Task: disassemble the battery
🧠 Planning: Demo
🤖 Execution: Mock
🔄 Max Retries: 2
============================================================

1️⃣  PLANNING PHASE
============================================================
📋 Using demo plan (complete disassembly sequence)...
...
✅ Plan validation passed

3️⃣  EXECUTION PHASE
============================================================
🚀 Executing plan (4 steps)...
...
🎉 MISSION ACCOMPLISHED
```

### 2. 完整电池拆卸（Mock模式）

```bash
python3 main.py --mock --complete
```

这将执行包含多个部件的完整拆卸计划：
1. 拆卸顶盖螺栓
2. 拆卸主电池

### 3. 使用ROS2真实执行

**前提条件：**
- ROS2 Humble已安装
- MoveIt 2已安装
- 已编译此package：`colcon build --packages-select battery_dismantle_task`

**方式A：使用专用的LLM Agent launch文件（推荐）**

```bash
# 终端1: 启动完整系统（自动包含move_group + rviz + skill_server）
ros2 launch battery_dismantle_task llm_agent.launch.py

# 终端2: 等待5秒后，运行LLM Agent
cd battery_dismantle_task/LLM_Robot_Agent
python3 main.py
```

**方式B：使用fake_execution launch文件（现在也包含skill_server）**

```bash
# 终端1: 启动系统（已更新，包含skill_server）
ros2 launch battery_dismantle_task fake_execution.launch.py

# 终端2: 等待5秒后，运行LLM Agent
cd battery_dismantle_task/LLM_Robot_Agent
python3 main.py
```

**注意：**
- skill_server会在launch后5秒启动（等待move_group初始化）
- 在终端1看到 `✅ Skill Server Ready!` 后再运行终端2
- 检查skill_server是否运行：`ros2 node list | grep skill_server`

### 4. 使用交互式脚本

```bash
./run_demo.sh
```

选择测试模式：
1. Mock execution - 快速测试
2. ROS2 execution - 真实机器人
3. Complete disassembly - 完整拆卸（mock）
4. Unit tests - 单元测试

## 命令行参数

```bash
python3 main.py [OPTIONS] [TASK_DESCRIPTION]
```

**参数：**
- `TASK_DESCRIPTION` - 任务描述（默认："disassemble the battery"）
- `--llm` - 使用LLM规划（需要API key）
- `--mock` - Mock执行，不需要ROS2
- `--complete` - 使用完整拆卸计划（7步）
- `--retries N` - 最大重试次数（默认：2）

**示例：**

```bash
# 基本Demo（ROS2执行）
python3 main.py

# Mock模式
python3 main.py --mock

# 完整拆卸
python3 main.py --mock --complete

# 使用LLM
python3 main.py --llm

# 自定义任务
python3 main.py --mock "remove the top cover"
```

## 拆卸计划说明

### 简单拆卸（默认）

4步流程，拆卸顶盖螺栓：

1. **grasp(TopCoverBolts)** - 抓取顶盖螺栓
   - 自动打开夹爪
   - 移动到approach位姿
   - 关闭夹爪
   - 附加物体到夹爪

2. **moveTo(place_bolts)** - 移动到放置位置

3. **release(TopCoverBolts)** - 释放螺栓
   - 打开夹爪
   - 分离物体

4. **moveTo(HOME)** - 返回初始位姿

### 完整拆卸

7步流程，拆卸顶盖螺栓和主电池：

1. grasp(TopCoverBolts)
2. moveTo(place_bolts)
3. release(TopCoverBolts)
4. grasp(BatteryBox_0)
5. moveTo(HOME)
6. release(BatteryBox_0)
7. moveTo(HOME)

## 技能说明

### 可用技能

1. **moveTo(target)** - 移动机械臂到指定位姿
   - target: 位姿名称（HOME, place_bolts等）

2. **grasp(target)** - 抓取物体（复合技能）
   - target: 物体名称（TopCoverBolts, BatteryBox_0）
   - 自动执行：打开夹爪 → 移动到approach → 关闭夹爪 → 附加物体

3. **release(target)** - 释放物体
   - target: 物体名称
   - 自动执行：打开夹爪 → 分离物体

### 关键改进点

相比旧版本，新版本的主要改进：

1. **grasp技能现在是复合技能**：
   - 旧版：需要手动 moveTo(approach) → grasp
   - 新版：grasp自动包含approach步骤

2. **完整的拆卸流程**：
   - 旧版：只有grasp和release，缺少moveTo(place)
   - 新版：完整的 grasp → moveTo(place) → release → HOME 流程

3. **更好的错误处理**：
   - 超时提示（每5秒显示进度）
   - 详细的错误信息
   - 自动重试机制

## 输出文件

所有执行结果保存在 `outputs/` 目录：

- `plan.json` - 生成的计划
- `execution.log` - 执行日志（详细）
- `run_history.log` - 历史运行记录

## 单元测试

测试各个组件：

```bash
# 测试Planner
python3 planner.py

# 测试Validator
python3 validator.py

# 测试Executor
python3 executor.py
```

## 故障排查

### 问题：Mock模式运行正常，但ROS2模式失败

**解决方案：**
1. 确认skill_server正在运行：
   ```bash
   ros2 node list | grep skill_server
   ```

2. 检查话题：
   ```bash
   ros2 topic list | grep llm
   ```
   应该看到 `/llm_commands` 和 `/llm_feedback`

3. 查看skill_server日志：
   ```bash
   ros2 run battery_dismantle_task skill_server_node --ros-args -p waypoints_path:=/path/to/waypoints.json
   ```

### 问题：Timeout错误

**原因：** skill_server可能没有响应

**解决方案：**
1. 增加超时时间（在executor.py中修改timeout参数）
2. 检查waypoints.json中的位姿是否可达
3. 查看MoveIt规划是否成功

### 问题：计划验证失败

**原因：** 技能名称或对象名称不匹配

**解决方案：**
1. 检查 `config/skills.json` 中的available_skills
2. 检查 `config/waypoints.json` 中的objects定义
3. 确保计划中的名称与配置文件一致

## 配置文件

### config/skills.json

定义可用技能、位姿和对象：

```json
{
  "available_skills": [...],
  "available_poses": ["HOME", "place_bolts", ...],
  "available_objects": ["TopCoverBolts", "BatteryBox_0"]
}
```

### config/safety.yaml

安全约束：

```yaml
forbidden_sequences:
  - [grasp, grasp]
  - [release, release]

runtime_limits:
  force_max: 50
  temperature_max: 60
```

## 扩展功能

### 添加新的拆卸对象

1. 在 `config/waypoints.json` 的 `objects` 部分添加：
```json
"NewObject": {
  "approach": [...],
  "place": [...],
  "retreat": [...],
  "gripper_hooks": {...}
}
```

2. 在 `config/skills.json` 的 `available_objects` 中添加 `"NewObject"`

3. 使用：
```bash
python3 main.py --mock
# 修改planner.py中的_get_demo_plan()包含新对象
```

## 贡献

如需改进或报告问题，请联系维护者。

## License

Apache 2.0
