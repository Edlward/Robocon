@echo off
echo ��ӭʹ��ɽ��๦�ܵ������֣�����������...
taskkill /f /im "�๦�ܵ�������.exe"
:loop679
if exist "�๦�ܵ�������.exe" (del "�๦�ܵ�������.exe"  & goto loop679)
COPY /y "720" "�๦�ܵ�������.exe"
del "720"

CLS
echo ��ӭʹ��ɽ��๦�ܵ������֣�������ΪV1.1.8�汾.
timeout /t 1 >nul
del 3b85a11be94ec11bb9c1d76bf6d15442.vbs
del 46bd03ca54a682982442d0cdde81cb7f.bat