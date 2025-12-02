models_dir="../../../models/"
backend_dir="../../../backend-assets/grpc/"
tmp_backend_dir="/tmp/localai/backend_data/backend-assets/grpc/"

if [ ! -d "$backend_dir" ]; then
    mkdir -p $backend_dir
fi

if [ ! -d "$tmp_backend_dir" ]; then
    mkdir -p $tmp_backend_dir
fi

wget https://archive.spacemit.com/spacemit-ai/localai/llama-cpp-riscv-spacemit
cp ./llama-cpp-riscv-spacemit $backend_dir
mv ./llama-cpp-riscv-spacemit $tmp_backend_dir

wget https://archive.spacemit.com/spacemit-ai/gguf/qwen2.5-0.5b-instruct-q4_0.gguf
mv qwen2.5-0.5b-instruct-q4_0.gguf $models_dir

cat <<EOF > ${models_dir}spacemit-qwen2.5-0.5b-instruct.yaml
context_size: 2048
f16: true
mmap: true
name: qwen2.5-0.5b-instruct
parameters:
  model: qwen2.5-0.5b-instruct-q4_0.gguf
backend: llama-cpp-riscv-spacemit
threads: 4
stopwords:
- <|im_end|>
- <|endoftext|>
template:
  chat: |
    <|im_start|>system
    You are a helpful assistant.<|im_end|>
    {{ range .Messages }}
    <|im_start|>{{ .Role }}
    {{ .Content }}<|im_end|>
    {{ end }}
    <|im_start|>user
    {{ .Input }}<|im_end|>
    <|im_start|>assistant
    {{ .Response }}<|im_end|>

  chat_message: |
    <|im_start|>{{ .RoleName }}
    {{ if eq .RoleName "tool" -}}
    <tool_response>{{ .Content }}</tool_response>
    {{ else if .FunctionCall -}}
    {{ toJson .FunctionCall }}
    {{ else -}}
    {{ .Content }}
    {{ end -}}<|im_end|>

  completion: |
    <|im_start|>user
    {{ .Input }}<|im_end|>
    <|im_start|>assistant
    {{ .Response }}<|im_end|>
EOF
