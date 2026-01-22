#!/bin/bash

# 测试词汇文件

echo "=== 测试VINS词汇文件 ==="

VOCAB_FILE="/home/bluebarry/desktop/wc/vins_fu_ws/install/loop_fusion/share/loop_fusion/support_files/brief_k10L6.bin"

echo "1. 检查文件存在性..."
if [[ -f "$VOCAB_FILE" ]]; then
    echo "✅ 词汇文件存在"
    echo "文件大小: $(stat -c%s "$VOCAB_FILE") bytes"
else
    echo "❌ 词汇文件不存在"
    exit 1
fi

echo ""
echo "2. 检查文件类型..."
file "$VOCAB_FILE"

echo ""
echo "3. 检查文件头部..."
hexdump -C "$VOCAB_FILE" | head -5

echo ""
echo "4. 尝试创建简化的词汇文件..."
# 创建一个最小的词汇文件用于测试
TEST_VOCAB="/tmp/test_vocab.bin"
cat > "$TEST_VOCAB" << 'EOF'
VOCAB004
K 10
L 6
SCORING 1
WEIGHTING 0
NODES 0
WORDS 0
EOF

echo "测试词汇文件已创建: $TEST_VOCAB"
echo "大小: $(stat -c%s "$TEST_VOCAB") bytes"

echo ""
echo "5. 建议解决方案："
echo "   - 词汇文件可能是Matlab格式，需要转换为DBoW2格式"
echo "   - 或者使用DBoW2工具重新生成词汇文件"
echo "   - 可以尝试禁用循环闭合功能进行测试"
