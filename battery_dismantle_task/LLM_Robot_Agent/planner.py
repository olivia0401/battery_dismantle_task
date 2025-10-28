"""
任务规划器 - 生成完整的电池拆卸计划

支持:
1. LLM动态规划
2. Demo固定计划（用于测试）
"""
import json
import asyncio
from pathlib import Path
from typing import Dict, List, Optional
from llm_client import LLMClient


class Planner:
    """任务规划器"""

    def __init__(self, config_dir: Optional[Path] = None):
        """
        初始化规划器

        Args:
            config_dir: 配置文件目录
        """
        if config_dir is None:
            config_dir = Path(__file__).parent / "config"

        # 加载配置
        with open(config_dir / "skills.json") as f:
            self.skills_config = json.load(f)

        with open(config_dir / "prompt.txt") as f:
            self.prompt_template = f.read()

        # 初始化LLM客户端
        self.llm = LLMClient(backend="openai")

    async def plan(self, task: str, use_llm: bool = True) -> Dict:
        """
        生成任务计划

        Args:
            task: 任务描述(自然语言)
            use_llm: 是否使用LLM(False则用demo计划)

        Returns:
            计划字典 {"plan": [...]}
        """
        if not use_llm or not self.llm.api_key:
            print("📋 Using demo plan (complete disassembly sequence)...")
            return self._get_demo_plan()

        # 构造prompt
        prompt = self._build_prompt(task)

        print(f"🤖 Planning: {task}")
        print("📡 Calling LLM...")

        try:
            # 调用LLM
            response = await self.llm.generate(prompt)

            # 解析JSON
            plan = self._parse_response(response)

            print("✅ Plan generated successfully")
            return plan

        except Exception as e:
            print(f"❌ LLM planning failed: {e}")
            print("🔄 Falling back to demo plan...")
            return self._get_demo_plan()

    def _build_prompt(self, task: str) -> str:
        """构造完整的prompt"""
        # 格式化技能列表
        skills_list = "\n".join([
            f"- {s['name']}: {s['description']}"
            for s in self.skills_config['available_skills']
        ])

        poses_list = ", ".join(self.skills_config['available_poses'])
        objects_list = ", ".join(self.skills_config['available_objects'])

        return self.prompt_template.format(
            task_description=task,
            skills_list=skills_list,
            poses_list=poses_list,
            objects_list=objects_list
        )

    def _parse_response(self, response: str) -> Dict:
        """从LLM响应中提取JSON"""
        # 找到JSON部分
        start = response.find("{")
        end = response.rfind("}") + 1

        if start < 0 or end <= start:
            raise ValueError("No valid JSON found in response")

        json_str = response[start:end]
        plan = json.loads(json_str)

        # 验证格式
        if "plan" not in plan:
            raise ValueError("Invalid plan format: missing 'plan' key")

        return plan

    def _get_demo_plan(self) -> Dict:
        """
        返回完整的demo计划 - 拆卸TopCoverBolts

        完整流程:
        1. grasp(TopCoverBolts) - 包含approach、close gripper、attach
        2. moveTo(place_bolts) - 移动到放置位置
        3. release(TopCoverBolts) - 打开夹爪、detach
        4. moveTo(HOME) - 回到初始位姿
        """
        return {
            "task": "Disassemble battery - remove TopCoverBolts",
            "plan": [
                {
                    "step": 1,
                    "name": "grasp",
                    "params": {"target": "TopCoverBolts"},
                    "description": "Grasp top cover bolts (includes approach + close gripper)"
                },
                {
                    "step": 2,
                    "name": "moveTo",
                    "params": {"target": "place_bolts"},
                    "description": "Move to placement position"
                },
                {
                    "step": 3,
                    "name": "release",
                    "params": {"target": "TopCoverBolts"},
                    "description": "Release bolts (open gripper + detach)"
                },
                {
                    "step": 4,
                    "name": "moveTo",
                    "params": {"target": "HOME"},
                    "description": "Return to home position"
                }
            ]
        }

    def get_complete_battery_disassembly_plan(self) -> Dict:
        """
        获取完整电池拆卸计划（多步骤）

        拆卸顺序:
        1. 拆卸TopCoverBolts
        2. 拆卸BatteryBox_0（主电池）

        Returns:
            完整拆卸计划
        """
        return {
            "task": "Complete battery disassembly",
            "plan": [
                # === 第一部分：拆卸顶盖螺栓 ===
                {
                    "step": 1,
                    "name": "grasp",
                    "params": {"target": "TopCoverBolts"},
                    "description": "Grasp top cover bolts"
                },
                {
                    "step": 2,
                    "name": "moveTo",
                    "params": {"target": "place_bolts"},
                    "description": "Move bolts to placement area"
                },
                {
                    "step": 3,
                    "name": "release",
                    "params": {"target": "TopCoverBolts"},
                    "description": "Release bolts"
                },

                # === 第二部分：拆卸主电池 ===
                {
                    "step": 4,
                    "name": "grasp",
                    "params": {"target": "BatteryBox_0"},
                    "description": "Grasp main battery box"
                },
                {
                    "step": 5,
                    "name": "moveTo",
                    "params": {"target": "HOME"},
                    "description": "Move battery to safe position"
                },
                {
                    "step": 6,
                    "name": "release",
                    "params": {"target": "BatteryBox_0"},
                    "description": "Release battery box"
                },

                # === 返回初始位姿 ===
                {
                    "step": 7,
                    "name": "moveTo",
                    "params": {"target": "HOME"},
                    "description": "Return to home position"
                }
            ]
        }

    def print_plan(self, plan: Dict):
        """打印计划"""
        print("\n" + "="*60)
        print("📋 Generated Plan:")
        if "task" in plan:
            print(f"   Task: {plan['task']}")
        print("="*60)

        for action in plan['plan']:
            desc = action.get('description', '')
            print(f"  Step {action['step']}: {action['name']}({action['params']['target']})")
            if desc:
                print(f"         → {desc}")

        print("="*60)


# 测试
async def test():
    planner = Planner()

    print("\n" + "="*60)
    print("TEST 1: Simple Demo Plan")
    print("="*60)
    plan1 = await planner.plan("disassemble the battery", use_llm=False)
    planner.print_plan(plan1)

    print("\n" + "="*60)
    print("TEST 2: Complete Battery Disassembly Plan")
    print("="*60)
    plan2 = planner.get_complete_battery_disassembly_plan()
    planner.print_plan(plan2)


if __name__ == "__main__":
    asyncio.run(test())
