"""
执行器 - ROS2 or Mock

功能:
1. 通过ROS2话题与skill_server通信
2. 发送技能命令并等待反馈
3. 支持Mock模式用于测试
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, List


class Executor:
    """任务执行器 - 通过ROS2话题与机器人通信"""

    def __init__(self, use_ros: bool = True):
        """
        初始化执行器

        Args:
            use_ros: 是否使用真实ROS2(False则mock)
        """
        self.use_ros = use_ros
        self.node = None
        self.command_pub = None
        self.feedback_sub = None
        self.last_feedback = None

        if use_ros:
            self._init_ros()

    def _init_ros(self):
        """初始化ROS2节点"""
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node('llm_executor')
        self.command_pub = self.node.create_publisher(String, '/llm_commands', 10)
        self.feedback_sub = self.node.create_subscription(
            String,
            '/llm_feedback',
            self._feedback_callback,
            10
        )

        print("✅ ROS2 executor initialized")

        # Wait for topic connections to establish
        print("⏳ Waiting for skill_server connection...")
        time.sleep(3.0)

        # Spin once to process any pending callbacks
        rclpy.spin_once(self.node, timeout_sec=0.1)
        print("✅ Ready to send commands\n")

    def _feedback_callback(self, msg):
        """接收反馈"""
        self.last_feedback = msg.data
        # 不在这里打印，在execute中统一打印

    def execute(self, plan: Dict, timeout: float = 120.0) -> Dict:
        """
        执行计划

        Args:
            plan: 计划字典
            timeout: 每个动作的超时时间(秒)

        Returns:
            执行结果 {"success": bool, "executed": int, "failed": int, "log": [...]}
        """
        actions = plan['plan']
        results = {
            "success": True,
            "executed": 0,
            "failed": 0,
            "log": []
        }

        print(f"🚀 Executing plan ({len(actions)} steps)...\n")

        for i, action in enumerate(actions, 1):
            skill = action['name']
            target = action['params']['target']
            description = action.get('description', '')

            print("="*60)
            print(f"📍 Step {i}/{len(actions)}: {skill}(target={target})")
            if description:
                print(f"   → {description}")
            print("="*60)

            # 执行
            start_time = time.time()

            if self.use_ros:
                success = self._execute_ros(action, timeout)
            else:
                success = self._execute_mock(action)

            elapsed = time.time() - start_time

            # 记录结果
            log_entry = {
                "step": i,
                "action": action,
                "success": success,
                "elapsed_time": elapsed,
                "timestamp": time.time()
            }
            results['log'].append(log_entry)

            if success:
                results['executed'] += 1
                print(f"✅ Step {i} completed successfully! (took {elapsed:.2f}s)")
            else:
                results['failed'] += 1
                results['success'] = False
                print(f"❌ Step {i} failed! (after {elapsed:.2f}s)")
                print("⚠️  Aborting remaining steps...")
                break

            print()
            time.sleep(0.5)  # 短暂延迟

        # 最终总结
        print("\n" + "="*60)
        print("📊 Execution Summary:")
        print("="*60)
        print(f"   Total Steps: {len(actions)}")
        print(f"   ✅ Succeeded: {results['executed']}")
        print(f"   ❌ Failed: {results['failed']}")
        print(f"   Overall: {'SUCCESS' if results['success'] else 'FAILED'}")
        print("="*60)

        return results

    def _execute_ros(self, action: Dict, timeout: float) -> bool:
        """
        通过ROS2执行

        Args:
            action: 动作字典
            timeout: 超时时间

        Returns:
            是否成功
        """
        # 构造命令
        command = {
            "skill": action['name'],
            "target": action['params']['target']
        }

        # 发布命令
        msg = String()
        msg.data = json.dumps(command)

        print(f"📤 Publishing command: {msg.data}")
        self.command_pub.publish(msg)

        # 等待反馈
        self.last_feedback = None
        start_time = time.time()
        last_spin_time = start_time

        print("⏳ Waiting for feedback...")

        while self.last_feedback is None:
            # 每0.1秒spin一次检查消息
            rclpy.spin_once(self.node, timeout_sec=0.1)

            current_time = time.time()
            elapsed = current_time - start_time

            # 每5秒打印一次进度
            if current_time - last_spin_time > 5.0:
                print(f"   Still waiting... ({elapsed:.1f}s elapsed)")
                last_spin_time = current_time

            # 检查超时
            if elapsed > timeout:
                print(f"   ⏱️  Timeout after {timeout}s!")
                return False

        # 解析JSON反馈: {"status": "success", "message": "...", "timestamp": ...}
        try:
            print(f"📥 Received feedback: {self.last_feedback}")
            feedback = json.loads(self.last_feedback)

            status = feedback.get("status", "unknown")
            message = feedback.get("message", "")

            if message:
                print(f"   Message: {message}")

            return status == "success"

        except (json.JSONDecodeError, KeyError) as e:
            print(f"   ⚠️  Failed to parse feedback: {e}")
            # 兼容简单字符串反馈
            return self.last_feedback == "success"

    def _execute_mock(self, action: Dict) -> bool:
        """Mock执行 (用于测试)"""
        print(f"🎭 Mock execution (simulating {action['name']})")
        time.sleep(1.0)  # 模拟执行时间
        print(f"   ✅ Mock completed")
        return True  # 总是成功

    def shutdown(self):
        """关闭执行器"""
        if self.use_ros and self.node:
            print("\n🔌 Shutting down executor...")
            self.node.destroy_node()
            # 注意：不调用rclpy.shutdown()，因为其他ROS2节点可能还在使用
            # rclpy.shutdown()


# 测试
def test_mock():
    """测试Mock模式"""
    print("\n" + "="*60)
    print("TEST: Mock Executor")
    print("="*60)

    executor = Executor(use_ros=False)

    plan = {
        "task": "Test disassembly",
        "plan": [
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
                "description": "Move to placement position"
            },
            {
                "step": 3,
                "name": "release",
                "params": {"target": "TopCoverBolts"},
                "description": "Release bolts"
            },
            {
                "step": 4,
                "name": "moveTo",
                "params": {"target": "HOME"},
                "description": "Return home"
            }
        ]
    }

    results = executor.execute(plan)

    print(f"\n🎯 Final Result: {'SUCCESS' if results['success'] else 'FAILED'}")

    executor.shutdown()


if __name__ == "__main__":
    test_mock()
