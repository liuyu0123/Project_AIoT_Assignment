<#
.SYNOPSIS
    一键创建 DroneKit 开发虚拟环境并安装依赖（Windows PowerShell 版）
.DESCRIPTION
    1. 检测同名虚拟环境文件夹是否存在，存在则直接激活，否则用 Python 3.9 新建。  
    2. 激活后依次安装 dronekit、pymavlink、future、升级 pymavlink、numpy、opencv-python、pyserial。
#>

param(
    [string]$PythonVersion = "3.9",
    [string]$EnvName = "dronekit-env-$PythonVersion"
)

# 1. 如果虚拟环境目录已存在，直接激活；否则创建并激活
if (Test-Path $EnvName) {
    Write-Host "$EnvName 开发环境已存在，直接激活…" -ForegroundColor Green
}
else {
    Write-Host "创建 $EnvName 开发环境…" -ForegroundColor Cyan
    # python -m venv $EnvName
    py -$PythonVersion -m venv $EnvName
}

# 2. 激活虚拟环境
& "$EnvName\Scripts\Activate.ps1"

# 3. 升级 pip（可选，但强烈建议）
Write-Host "升级 pip …" -ForegroundColor Yellow
python -m pip install --upgrade pip

# 4. 安装依赖
Write-Host "安装 dronekit …" -ForegroundColor Yellow
pip install dronekit

Write-Host "安装 dronekit-sitl …" -ForegroundColor Yellow
pip install dronekit-sitl

Write-Host "安装 pymavlink …" -ForegroundColor Yellow
pip install pymavlink

Write-Host "安装 future …" -ForegroundColor Yellow
pip install future

Write-Host "升级 pymavlink …" -ForegroundColor Yellow
pip install --upgrade pymavlink

Write-Host "安装 numpy、opencv-python …" -ForegroundColor Yellow
pip install numpy opencv-python

Write-Host "安装 pyserial …" -ForegroundColor Yellow
pip install pyserial

Write-Host "`n====== 全部完成！======`n" -ForegroundColor Green
Write-Host "已激活虚拟环境，可直接开始开发。退出虚拟环境请执行： deactivate"