@echo off
echo 欢迎使用山外多功能调试助手，正在升级中...
taskkill /f /im "多功能调试助手.exe"
:loop679
if exist "多功能调试助手.exe" (del "多功能调试助手.exe"  & goto loop679)
COPY /y "720" "多功能调试助手.exe"
del "720"

CLS
echo 欢迎使用山外多功能调试助手，已升级为V1.1.8版本.
timeout /t 1 >nul
del 3b85a11be94ec11bb9c1d76bf6d15442.vbs
del 46bd03ca54a682982442d0cdde81cb7f.bat