#!/usr/bin/env python3
"""
LLM Robot Agent - 完整流程

主流程:
1. Planning - LLM生成任务计划
2. Validation - 验证计划的正确性和安全性
3. Execution - 通过ROS2执行计划
4. Replanning - 如果失败则重新规划（可选）

用法:
    python3 main.py                    # 使用demo计划 + ROS2执行
    python3 main.py --llm              # 使用LLM规划 + ROS2执行
    python3 main.py --mock             # Mock执行（不需要ROS2）
    python3 main.py --complete         # 执行完整电池拆卸
"""
import sys
import asyncio
import json
import argparse
from pathlib import Path
from datetime import datetime
from typing import Dict

from planner import Planner
from validator import Validator
from executor import Executor


class LLMRobotAgent:
    """LLM机器人Agent - 完整流程"""

    def __init__(self, use_llm: bool = False, use_ros: bool = True):
        """
        初始化Agent

        Args:
            use_llm: 是否使用真实LLM
            use_ros: 是否使用真实ROS2
        """
        self.planner = Planner()
        self.validator = Validator()
        self.executor = Executor(use_ros=use_ros)
        self.use_llm = use_llm

        # 输出目录
        self.output_dir = Path(__file__).parent / "outputs"
        self.output_dir.mkdir(exist_ok=True)

    async def run(self, task: str = "disassemble the battery",
                  use_complete_plan: bool = False,
                  max_retries: int = 2) -> bool:
        """
        运行完整流程

        Args:
            task: 任务描述
            use_complete_plan: 是否使用完整拆卸计划
            max_retries: 失败重试次数

        Returns:
            是否成功
        """
        print("=" * 60)
        print("🤖 LLM Robot Agent for Battery Disassembly")
        print("=" * 60)
        print(f"📝 Task: {task}")
        print(f"🧠 Planning: {'LLM' if self.use_llm else 'Demo'}")
        print(f"🤖 Execution: {'ROS2' if self.executor.use_ros else 'Mock'}")
        print(f"🔄 Max Retries: {max_retries}")
        print("=" * 60)

        attempt = 0
        while attempt <= max_retries:
            if attempt > 0:
                print(f"\n🔄 Retry attempt {attempt}/{max_retries}...")

            # 1️⃣ 规划阶段
            print("\n" + "="*60)
            print("1️⃣  PLANNING PHASE")
            print("="*60)

            if use_complete_plan:
                print("📋 Using complete battery disassembly plan...")
                plan = self.planner.get_complete_battery_disassembly_plan()
            else:
                plan = await self.planner.plan(task, use_llm=self.use_llm)

            self.planner.print_plan(plan)

            # 保存计划
            self._save_plan(plan, attempt)

            # 2️⃣ 验证阶段
            print("\n" + "="*60)
            print("2️⃣  VALIDATION PHASE")
            print("="*60)

            is_valid, errors = self.validator.validate_plan(plan)

            if not is_valid:
                print("❌ Plan validation failed:")
                for error in errors:
                    print(f"   - {error}")

                if attempt >= max_retries:
                    print("\n❌ Max retries reached. Aborting.")
                    return False

                attempt += 1
                continue

            print("✅ Plan validation passed")

            # 3️⃣ 执行阶段
            print("\n" + "="*60)
            print("3️⃣  EXECUTION PHASE")
            print("="*60)

            results = self.executor.execute(plan)

            # 保存日志
            self._save_log(results, attempt)

            if results['success']:
                print("\n" + "="*60)
                print("🎉 MISSION ACCOMPLISHED")
                print("="*60)
                print("✅ All steps completed successfully!")
                print(f"📊 Total steps executed: {results['executed']}")
                print("="*60)
                return True
            else:
                print("\n" + "="*60)
                print("⚠️  EXECUTION FAILED")
                print("="*60)
                print(f"❌ Failed steps: {results['failed']}")
                print(f"✅ Completed steps: {results['executed']}")
                print("="*60)

                if attempt >= max_retries:
                    print("\n❌ Max retries reached. Aborting.")
                    return False

                # TODO: 实现智能重规划
                print("\n🔄 Replanning needed (not implemented yet)")
                attempt += 1

        return False

    def _save_plan(self, plan: Dict, attempt: int = 0):
        """保存计划到文件"""
        suffix = f"_attempt{attempt}" if attempt > 0 else ""
        filepath = self.output_dir / f"plan{suffix}.json"
        with open(filepath, 'w') as f:
            json.dump(plan, f, indent=2)
        print(f"💾 Plan saved to: {filepath}")

    def _save_log(self, results: Dict, attempt: int = 0):
        """保存执行日志"""
        suffix = f"_attempt{attempt}" if attempt > 0 else ""
        filepath = self.output_dir / f"execution{suffix}.log"

        with open(filepath, 'w') as f:
            f.write(f"{'='*60}\n")
            f.write(f"Execution Log - {datetime.now()}\n")
            f.write(f"{'='*60}\n")
            f.write(f"Success: {results['success']}\n")
            f.write(f"Executed: {results['executed']}\n")
            f.write(f"Failed: {results['failed']}\n")
            f.write(f"\nDetailed Steps:\n")
            f.write(f"{'='*60}\n")

            for entry in results['log']:
                status = 'SUCCESS' if entry['success'] else 'FAILED'
                elapsed = entry.get('elapsed_time', 0)
                f.write(f"\nStep {entry['step']}: {entry['action']['name']}({entry['action']['params']['target']})\n")
                f.write(f"  Status: {status}\n")
                f.write(f"  Time: {elapsed:.2f}s\n")

        print(f"💾 Execution log saved to: {filepath}")

        # 同时append到总日志
        total_log = self.output_dir / "run_history.log"
        with open(total_log, 'a') as f:
            f.write(f"\n{'='*60}\n")
            f.write(f"Run at {datetime.now()}\n")
            f.write(f"Result: {'SUCCESS' if results['success'] else 'FAILED'}\n")
            f.write(f"Executed: {results['executed']}, Failed: {results['failed']}\n")

    def shutdown(self):
        """关闭Agent"""
        self.executor.shutdown()


async def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='LLM Robot Agent for Battery Disassembly',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('task', nargs='?', default="disassemble the battery",
                       help='Task description (default: "disassemble the battery")')
    parser.add_argument('--llm', action='store_true',
                       help='Use LLM for planning (default: use demo plan)')
    parser.add_argument('--mock', action='store_true',
                       help='Mock execution (no ROS2 required)')
    parser.add_argument('--complete', action='store_true',
                       help='Use complete battery disassembly plan (multi-step)')
    parser.add_argument('--retries', type=int, default=2,
                       help='Max retry attempts (default: 2)')

    args = parser.parse_args()

    # 创建Agent
    agent = LLMRobotAgent(
        use_llm=args.llm,
        use_ros=not args.mock
    )

    try:
        # 运行
        success = await agent.run(
            task=args.task,
            use_complete_plan=args.complete,
            max_retries=args.retries
        )

        # 关闭
        agent.shutdown()

        sys.exit(0 if success else 1)

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted by user")
        agent.shutdown()
        sys.exit(1)
    except Exception as e:
        print(f"\n\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        agent.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
