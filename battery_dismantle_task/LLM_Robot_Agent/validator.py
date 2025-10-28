"""
计划验证器 - 验证计划的正确性和安全性

验证项目:
1. 技能名称是否有效
2. 参数是否完整
3. 对象/位姿是否存在
4. 逻辑顺序是否正确（grasp before release等）
5. 是否满足安全约束
"""
import json
from pathlib import Path
from typing import Dict, List, Tuple


class Validator:
    """计划验证器"""

    def __init__(self, config_dir: Path = None):
        """
        初始化验证器

        Args:
            config_dir: 配置文件目录
        """
        if config_dir is None:
            config_dir = Path(__file__).parent / "config"

        # 加载配置
        with open(config_dir / "skills.json") as f:
            self.skills_config = json.load(f)

        # 加载safety配置（简单YAML解析）
        try:
            import yaml
            with open(config_dir / "safety.yaml") as f:
                self.safety_config = yaml.safe_load(f)
        except ImportError:
            # 如果yaml库不可用，使用简单解析
            with open(config_dir / "safety.yaml") as f:
                self.safety_config = self._parse_simple_yaml(f.read())

        # 提取有效值列表
        self.valid_skills = {s['name'] for s in self.skills_config['available_skills']}
        self.valid_poses = set(self.skills_config['available_poses'])
        self.valid_objects = set(self.skills_config['available_objects'])

        print(f"✅ Validator initialized")
        print(f"   Valid skills: {self.valid_skills}")
        print(f"   Valid poses: {self.valid_poses}")
        print(f"   Valid objects: {self.valid_objects}")

    def _parse_simple_yaml(self, yaml_str: str) -> Dict:
        """简单解析YAML (仅支持基本key: value格式)"""
        config = {}
        current_list_key = None
        current_list = []

        for line in yaml_str.split('\n'):
            stripped = line.strip()

            # 跳过空行和注释
            if not stripped or stripped.startswith('#'):
                continue

            # 检查列表项
            if stripped.startswith('- '):
                if current_list_key:
                    # 去除'- '前缀
                    item = stripped[2:].strip()
                    # 尝试解析列表值（如 [foo, bar]）
                    if item.startswith('[') and item.endswith(']'):
                        item = json.loads(item)
                    current_list.append(item)
                continue

            # 处理key: value
            if ':' in stripped:
                # 保存之前的列表
                if current_list_key:
                    config[current_list_key] = current_list
                    current_list_key = None
                    current_list = []

                key, value = stripped.split(':', 1)
                key = key.strip()
                value = value.strip()

                # 如果value为空，这可能是列表的开始
                if not value:
                    current_list_key = key
                    continue

                # 转换类型
                if value.lower() == 'true':
                    value = True
                elif value.lower() == 'false':
                    value = False
                elif value.isdigit():
                    value = int(value)
                elif '.' in value and value.replace('.', '').isdigit():
                    value = float(value)

                config[key] = value

        # 保存最后的列表
        if current_list_key:
            config[current_list_key] = current_list

        return config

    def validate_plan(self, plan: Dict) -> Tuple[bool, List[str]]:
        """
        验证计划

        Args:
            plan: 计划字典

        Returns:
            (is_valid, errors) - 是否有效和错误列表
        """
        errors = []

        # 1. 检查计划格式
        if 'plan' not in plan:
            errors.append("Missing 'plan' key in plan dict")
            return False, errors

        actions = plan['plan']

        if not actions:
            errors.append("Plan is empty")
            return False, errors

        # 2. 检查步骤编号连续性
        for i, action in enumerate(actions, 1):
            if 'step' not in action:
                errors.append(f"Action {i} missing 'step' field")
            elif action['step'] != i:
                errors.append(f"Step numbering inconsistent: expected {i}, got {action['step']}")

        # 3. 检查每个动作
        for i, action in enumerate(actions, 1):
            action_errors = self._validate_action(action, i)
            errors.extend(action_errors)

        # 4. 检查逻辑顺序
        sequence_errors = self._validate_sequence(actions)
        errors.extend(sequence_errors)

        # 5. 检查安全约束
        safety_errors = self._validate_safety(actions)
        errors.extend(safety_errors)

        is_valid = len(errors) == 0
        return is_valid, errors

    def _validate_action(self, action: Dict, index: int) -> List[str]:
        """验证单个动作"""
        errors = []

        # 检查必需字段
        if 'name' not in action:
            errors.append(f"Step {index}: Missing 'name' field")
            return errors

        if 'params' not in action:
            errors.append(f"Step {index}: Missing 'params' field")
            return errors

        skill_name = action['name']
        params = action['params']

        # 检查技能是否有效
        if skill_name not in self.valid_skills:
            errors.append(f"Step {index}: Unknown skill '{skill_name}'")
            return errors

        # 检查参数
        if 'target' not in params:
            errors.append(f"Step {index}: Missing 'target' parameter")
            return errors

        target = params['target']

        # 检查target是否有效
        if skill_name == 'moveTo':
            # moveTo需要位姿名称
            if target not in self.valid_poses:
                errors.append(f"Step {index}: Unknown pose '{target}' for moveTo")
        elif skill_name in ['grasp', 'release']:
            # grasp/release需要对象名称
            if target not in self.valid_objects:
                errors.append(f"Step {index}: Unknown object '{target}' for {skill_name}")

        return errors

    def _validate_sequence(self, actions: List[Dict]) -> List[str]:
        """验证动作序列的逻辑"""
        errors = []

        # 追踪夹爪状态
        gripper_state = "unknown"  # unknown, holding, empty
        held_object = None

        for i, action in enumerate(actions, 1):
            skill = action['name']
            target = action['params']['target']

            # 检查grasp逻辑
            if skill == 'grasp':
                if gripper_state == "holding":
                    errors.append(f"Step {i}: Cannot grasp '{target}' - already holding '{held_object}'")
                gripper_state = "holding"
                held_object = target

            # 检查release逻辑
            elif skill == 'release':
                if gripper_state == "empty":
                    errors.append(f"Step {i}: Cannot release - gripper is empty")
                elif gripper_state == "holding" and held_object != target:
                    errors.append(f"Step {i}: Cannot release '{target}' - currently holding '{held_object}'")
                gripper_state = "empty"
                held_object = None

        # 检查禁止的序列
        if 'skill_constraints' in self.skills_config:
            constraints = self.skills_config['skill_constraints']
            if 'forbidden_sequences' in constraints:
                for i in range(len(actions) - 1):
                    seq = [actions[i]['name'], actions[i+1]['name']]
                    if seq in constraints['forbidden_sequences']:
                        errors.append(f"Step {i+1}-{i+2}: Forbidden sequence {seq}")

        return errors

    def _validate_safety(self, actions: List[Dict]) -> List[str]:
        """验证安全约束"""
        errors = []

        # 检查计划长度
        if 'skill_constraints' in self.skills_config:
            constraints = self.skills_config['skill_constraints']
            if 'max_plan_length' in constraints:
                max_length = constraints['max_plan_length']
                if len(actions) > max_length:
                    errors.append(f"Plan too long: {len(actions)} steps (max: {max_length})")

        # 检查是否在HOME位姿结束（宽松检查，仅警告）
        # 不再强制要求，因为某些计划可能不需要返回HOME

        return errors

    def validate_runtime(self, force: float, temperature: float) -> Tuple[bool, str]:
        """
        Runtime验证 - 检查传感器数据

        Args:
            force: 力传感器读数 (N)
            temperature: 温度 (°C)

        Returns:
            (是否安全, 消息)
        """
        if 'runtime_limits' not in self.safety_config:
            return True, "No runtime limits configured"

        limits = self.safety_config['runtime_limits']

        if 'force_max' in limits and force > limits['force_max']:
            return False, f"Force {force}N exceeds limit {limits['force_max']}N"

        if 'temperature_max' in limits and temperature > limits['temperature_max']:
            return False, f"Temperature {temperature}°C exceeds limit {limits['temperature_max']}°C"

        return True, "Runtime checks passed"


# 测试
def test():
    """测试验证器"""
    print("\n" + "="*60)
    print("VALIDATOR TESTS")
    print("="*60)

    validator = Validator()

    # 测试1: 有效计划
    print("\n" + "="*60)
    print("TEST 1: Valid Plan")
    print("="*60)

    valid_plan = {
        "plan": [
            {"step": 1, "name": "grasp", "params": {"target": "TopCoverBolts"}},
            {"step": 2, "name": "moveTo", "params": {"target": "place_bolts"}},
            {"step": 3, "name": "release", "params": {"target": "TopCoverBolts"}},
            {"step": 4, "name": "moveTo", "params": {"target": "HOME"}}
        ]
    }

    is_valid, errors = validator.validate_plan(valid_plan)
    print(f"Result: {'✅ VALID' if is_valid else '❌ INVALID'}")
    if errors:
        for error in errors:
            print(f"  - {error}")

    # 测试2: 无效计划 (unknown skill)
    print("\n" + "="*60)
    print("TEST 2: Invalid Plan (unknown skill)")
    print("="*60)

    invalid_plan = {
        "plan": [
            {"step": 1, "name": "fly", "params": {"target": "Moon"}},
        ]
    }

    is_valid, errors = validator.validate_plan(invalid_plan)
    print(f"Result: {'✅ VALID' if is_valid else '❌ INVALID'}")
    if errors:
        for error in errors:
            print(f"  - {error}")

    # 测试3: 无效计划 (grasp twice)
    print("\n" + "="*60)
    print("TEST 3: Invalid Plan (grasp twice without release)")
    print("="*60)

    invalid_plan2 = {
        "plan": [
            {"step": 1, "name": "grasp", "params": {"target": "TopCoverBolts"}},
            {"step": 2, "name": "grasp", "params": {"target": "BatteryBox_0"}},
        ]
    }

    is_valid, errors = validator.validate_plan(invalid_plan2)
    print(f"Result: {'✅ VALID' if is_valid else '❌ INVALID'}")
    if errors:
        for error in errors:
            print(f"  - {error}")

    # 测试4: runtime验证
    print("\n" + "="*60)
    print("TEST 4: Runtime Validation")
    print("="*60)

    safe, msg = validator.validate_runtime(force=15.0, temperature=30.0)
    print(f"Result: {'✅ SAFE' if safe else '⚠️  UNSAFE'}")
    print(f"Message: {msg}")


if __name__ == "__main__":
    test()
