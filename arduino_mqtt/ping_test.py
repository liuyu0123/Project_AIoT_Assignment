# 导入 os 模块，这个模块提供了许多与操作系统交互的功能。
import os
# 定义一个变量，存放我们想要测试连通性的网站地址。
hostname = "baidu.com"
# 打印提示信息，让我们知道程序正在做什么。
print(f"正在 Ping {hostname}...")
# 使用 os.system() 函数来执行一个系统命令。
# 这行代码的效果等同于直接在 cmd 中输入 "ping google.com"。
# os.system() 会执行括号内的命令字符串。
os.system(f"ping {hostname}")