#!/bin/bash
# 快速测试脚本 - Battery Disassembly LLM Agent

echo "=================================================="
echo "Battery Disassembly - LLM Robot Agent"
echo "=================================================="
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 检查Python3
if ! command -v python3 &> /dev/null; then
    echo "❌ Error: python3 not found"
    exit 1
fi

echo "Choose test mode:"
echo "  1) Mock execution (no ROS2 required) - Quick test"
echo "  2) ROS2 execution (requires ROS2 running)"
echo "  3) Complete battery disassembly (mock)"
echo "  4) Run all unit tests"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "🎭 Running MOCK execution test..."
        echo "=================================================="
        python3 main.py --mock
        ;;
    2)
        echo ""
        echo "🤖 Running ROS2 execution test..."
        echo "=================================================="
        echo "⚠️  Make sure ROS2 and skill_server are running!"
        read -p "Press Enter to continue or Ctrl+C to cancel..."
        python3 main.py
        ;;
    3)
        echo ""
        echo "🔧 Running COMPLETE disassembly (mock)..."
        echo "=================================================="
        python3 main.py --mock --complete
        ;;
    4)
        echo ""
        echo "🧪 Running unit tests..."
        echo "=================================================="
        echo ""
        echo "--- Testing Planner ---"
        python3 planner.py
        echo ""
        echo "--- Testing Validator ---"
        python3 validator.py
        echo ""
        echo "--- Testing Executor ---"
        python3 executor.py
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo ""
echo "=================================================="
echo "✅ Test completed!"
echo "=================================================="
