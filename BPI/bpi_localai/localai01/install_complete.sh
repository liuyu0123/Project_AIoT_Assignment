#!/bin/bash

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  LocalAI 完整安装（RISC-V 优化版）${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 设置错误时退出
set -e

# ============= 步骤 1: 清理环境 =============
echo -e "${YELLOW}[1/10] 清理环境...${NC}"
cd ~/localai

# 清理编译产物
make clean 2>/dev/null || true
rm -rf sources/* 2>/dev/null || true
rm -f local-ai 2>/dev/null || true

# 清理错误的 Git 配置
git config --global --unset url."https://ghproxy.com/https://github.com".insteadOf 2>/dev/null || true
git config --global --get-regexp url 2>/dev/null | while read line; do
    key=$(echo $line | awk '{print $1}')
    git config --global --unset "$key" 2>/dev/null || true
done

echo -e "${GREEN}✓ 清理完成${NC}"
echo ""

# ============= 步骤 2: 配置 Go 环境（关键：避免 GC 崩溃）=============
echo -e "${YELLOW}[2/10] 配置 Go 环境（RISC-V 优化）...${NC}"

# 设置 Go 代理
export GOPROXY=https://goproxy.cn,direct
export GOSUMDB=sum.golang.google.cn
export PATH=$PATH:$(go env GOPATH)/bin

# 关键：RISC-V 的 GC 优化
export GOGC=800           # 提高 GC 阈值（默认 100）
export GOMEMLIMIT=2GiB    # 限制内存使用
export GOMAXPROCS=4       # 限制并发数

echo -e "  ${BLUE}- GOPROXY: $GOPROXY${NC}"
echo -e "  ${BLUE}- GOGC: $GOGC (优化 RISC-V GC)${NC}"
echo -e "  ${BLUE}- GOPATH: $(go env GOPATH)${NC}"
echo -e "${GREEN}✓ Go 环境配置完成${NC}"
echo ""

# ============= 步骤 3: 安装编译工具 =============
echo -e "${YELLOW}[3/10] 安装 Go 编译工具...${NC}"

# 安装 protoc-gen-go
if ! command -v protoc-gen-go &> /dev/null; then
    echo -e "  ${BLUE}- 安装 protoc-gen-go...${NC}"
    GOGC=off go install google.golang.org/protobuf/cmd/protoc-gen-go@v1.34.2
else
    echo -e "  ${BLUE}- protoc-gen-go 已安装${NC}"
fi

# 安装 protoc-gen-go-grpc
if ! command -v protoc-gen-go-grpc &> /dev/null; then
    echo -e "  ${BLUE}- 安装 protoc-gen-go-grpc...${NC}"
    GOGC=off go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@1958fcbe2ca8bd93af633f11e97d44e567e945af
else
    echo -e "  ${BLUE}- protoc-gen-go-grpc 已安装${NC}"
fi

# 安装 rice
if ! command -v rice &> /dev/null; then
    echo -e "  ${BLUE}- 安装 rice...${NC}"
    GOGC=off go install github.com/GeertJohan/go.rice/rice@latest
else
    echo -e "  ${BLUE}- rice 已安装${NC}"
fi

echo -e "${GREEN}✓ 编译工具安装完成${NC}"
echo ""

# ============= 步骤 4: 生成 Protobuf 代码 =============
echo -e "${YELLOW}[4/10] 生成 Protobuf 代码...${NC}"

mkdir -p pkg/grpc/proto

protoc --experimental_allow_proto3_optional \
    -Ibackend/ \
    --go_out=pkg/grpc/proto/ \
    --go_opt=paths=source_relative \
    --go-grpc_out=pkg/grpc/proto/ \
    --go-grpc_opt=paths=source_relative \
    backend/backend.proto

echo -e "${GREEN}✓ Protobuf 代码生成完成${NC}"
echo ""

# ============= 步骤 5: 创建目录结构 =============
echo -e "${YELLOW}[5/10] 创建目录结构...${NC}"

mkdir -p backend-assets/grpc
mkdir -p backend-assets/backend-assets
mkdir -p models

echo -e "  ${BLUE}- backend-assets/grpc/${NC}"
echo -e "  ${BLUE}- backend-assets/backend-assets/${NC}"
echo -e "  ${BLUE}- models/${NC}"
echo -e "${GREEN}✓ 目录结构创建完成${NC}"
echo ""

# ============= 步骤 6: 编译 LocalAI（跳过 go mod download）=============
echo -e "${YELLOW}[6/10] 编译 LocalAI 主程序...${NC}"

VERSION=$(git describe --always --tags 2>/dev/null || echo "dev")
COMMIT=$(git rev-parse HEAD 2>/dev/null || echo "unknown")

echo -e "  ${BLUE}- 版本: $VERSION${NC}"
echo -e "  ${BLUE}- 提交: ${COMMIT:0:8}${NC}"
echo -e "  ${BLUE}- 优化: 降低并发，避免 GC 崩溃${NC}"
echo ""

# 编译（使用 -p 2 降低并发）
go build -p 2 \
    -ldflags "-s -w \
    -X 'github.com/mudler/LocalAI/internal.Version=${VERSION}' \
    -X 'github.com/mudler/LocalAI/internal.Commit=${COMMIT}'" \
    -o local-ai ./

FILE_SIZE=$(du -h local-ai | awk '{print $1}')
echo -e "${GREEN}✓ 编译完成！文件大小: ${FILE_SIZE}${NC}"
echo ""

# ============= 步骤 7: 安装 RISC-V LLM 后端 =============
echo -e "${YELLOW}[7/10] 安装 RISC-V LLM 后端...${NC}"

cd backend/cpp/spacemit-llama-cpp

if [ -f "install.sh" ]; then
    echo -e "  ${BLUE}- 执行 install.sh...${NC}"
    bash install.sh
else
    echo -e "  ${RED}✗ install.sh 不存在，手动下载...${NC}"
    
    # 手动下载后端
    wget -O llama-cpp-riscv-spacemit \
        https://archive.spacemit.com/spacemit-ai/localai/llama-cpp-riscv-spacemit
    
    chmod +x llama-cpp-riscv-spacemit
    
    # 复制到目标目录
    mkdir -p ../../../backend-assets/grpc
    cp llama-cpp-riscv-spacemit ../../../backend-assets/grpc/
    
    # 下载模型
    cd ../../../models
    if [ ! -f "qwen2.5-0.5b-instruct-q4_0.gguf" ]; then
        wget https://archive.spacemit.com/spacemit-ai/gguf/qwen2.5-0.5b-instruct-q4_0.gguf
    fi
    
    # 创建配置
    cat > qwen2.5-0.5b-instruct.yaml << 'YAML'
name: qwen2.5-0.5b-instruct
backend: llama-cpp-riscv-spacemit
parameters:
  model: qwen2.5-0.5b-instruct-q4_0.gguf
  temperature: 0.7
  top_k: 40
  top_p: 0.9
  max_tokens: 2048
context_size: 8192
threads: 4
f16: true
gpu_layers: 0
mmap: true
mmlock: false

template:
  chat: |
    <|im_start|>system
    {{.SystemPrompt}}<|im_end|>
    {{range .Messages}}
    <|im_start|>{{.Role}}
    {{.Content}}<|im_end|>
    {{end}}
    <|im_start|>assistant
  completion: |
    {{.Input}}
YAML
    
    cd ~/localai
fi

cd ~/localai

echo -e "${GREEN}✓ RISC-V LLM 后端安装完成${NC}"
echo ""

# ============= 步骤 8: 嵌入静态资源 =============
echo -e "${YELLOW}[8/10] 嵌入静态资源...${NC}"

# 检查目录是否存在
if [ -d "backend-assets" ]; then
    rice append --exec local-ai
    echo -e "${GREEN}✓ 静态资源嵌入完成${NC}"
else
    echo -e "${RED}✗ backend-assets 目录不存在，跳过${NC}"
fi

echo ""

# ============= 步骤 9: 验证安装 =============
echo -e "${YELLOW}[9/10] 验证安装...${NC}"

# 检查主程序
if [ -f "./local-ai" ]; then
    SIZE=$(du -h local-ai | awk '{print $1}')
    echo -e "${GREEN}✓ LocalAI 主程序: 存在 (${SIZE})${NC}"
else
    echo -e "${RED}✗ LocalAI 主程序: 不存在${NC}"
    exit 1
fi

# 检查后端
if [ -f "backend-assets/grpc/llama-cpp-riscv-spacemit" ]; then
    SIZE=$(du -h backend-assets/grpc/llama-cpp-riscv-spacemit | awk '{print $1}')
    echo -e "${GREEN}✓ RISC-V 后端: 存在 (${SIZE})${NC}"
else
    echo -e "${RED}✗ RISC-V 后端: 不存在${NC}"
    echo -e "${YELLOW}  提示：需要手动下载后端${NC}"
fi

# 检查模型
MODEL_COUNT=$(ls -1 models/*.gguf 2>/dev/null | wc -l)
if [ $MODEL_COUNT -gt 0 ]; then
    echo -e "${GREEN}✓ 模型文件: ${MODEL_COUNT} 个${NC}"
    ls -1 models/*.gguf 2>/dev/null | while read file; do
        SIZE=$(du -h "$file" | awk '{print $1}')
        NAME=$(basename "$file")
        echo -e "  ${BLUE}- ${NAME} (${SIZE})${NC}"
    done
else
    echo -e "${YELLOW}⚠ 警告: 没有找到模型文件${NC}"
fi

# 检查配置
CONFIG_COUNT=$(ls -1 models/*.yaml 2>/dev/null | wc -l)
if [ $CONFIG_COUNT -gt 0 ]; then
    echo -e "${GREEN}✓ 配置文件: ${CONFIG_COUNT} 个${NC}"
    ls -1 models/*.yaml 2>/dev/null | while read file; do
        NAME=$(basename "$file")
        echo -e "  ${BLUE}- ${NAME}${NC}"
    done
else
    echo -e "${YELLOW}⚠ 警告: 没有找到配置文件${NC}"
fi

echo ""

# ============= 步骤 10: 完成 =============
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  ✅ 安装成功！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}📌 启动命令：${NC}"
echo -e "  ${BLUE}cd ~/localai${NC}"
echo -e "  ${BLUE}./local-ai --debug${NC}"
echo ""
echo -e "${YELLOW}📌 在另一个终端测试：${NC}"
echo -e "  ${BLUE}curl http://localhost:8080/readiness${NC}"
echo -e "  ${BLUE}curl http://localhost:8080/v1/models${NC}"
echo ""
echo -e "${YELLOW}📌 测试对话：${NC}"
echo -e "  ${BLUE}curl http://localhost:8080/v1/chat/completions \\${NC}"
echo -e "  ${BLUE}  -H 'Content-Type: application/json' \\${NC}"
echo -e "  ${BLUE}  -d '{\"model\":\"qwen2.5-0.5b-instruct\",\"messages\":[{\"role\":\"user\",\"content\":\"你好\"}]}'${NC}"
echo ""
echo -e "${YELLOW}📌 Web 界面：${NC}"
IP=$(hostname -I | awk '{print $1}' 2>/dev/null || echo "localhost")
echo -e "  ${BLUE}http://localhost:8080/${NC}"
echo -e "  ${BLUE}http://$IP:8080/${NC}"
echo ""
echo -e "${YELLOW}📌 后台运行：${NC}"
echo -e "  ${BLUE}nohup ./local-ai --debug > localai.log 2>&1 &${NC}"
echo -e "  ${BLUE}tail -f localai.log${NC}"
echo ""

