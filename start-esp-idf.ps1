# 保证 UTF-8 输出
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

# 配置 Python 环境
$env:Path = 'C:\Users\E0000711\.espressif\python_env\idf6.0_py3.11_env\Scripts;' + $env:Path
$env:IDF_PYTHON_ENV_PATH = 'C:\Users\E0000711\.espressif\python_env\idf6.0_py3.11_env'

# 导出 ESP-IDF 环境
. 'C:\Users\E0000711\.vscode\extensions\espressif.esp-idf-extension-1.10.2\export.ps1'

# 激活 Python ESP-IDF 工具
python 'C:\Users\E0000711\esp\v6.0\esp-idf\tools\activate.py'

Write-Host "ESP-IDF 环境准备完成，可以正常显示中文与表格"
