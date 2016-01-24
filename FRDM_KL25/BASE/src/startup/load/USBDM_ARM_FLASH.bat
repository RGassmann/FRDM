@echo off

SETLOCAL
SET allargs=%*
IF NOT DEFINED allargs echo no args provided&GOTO :EOF
set TARGET=%1
set UC=%2
set IMAGEFILE=%3
CALL SET someargs=%%allargs:*%3=%%
cls

echo Programming an %TARGET% - %UC% with the image "%IMAGEFILE%" with Parameter %someargs%
echo -target:%TARGET%  -device:%UC% %someargs% -erase:All -program %IMAGEFILE% -execute 
UsbdmFlashProgrammer -target:%TARGET%  -device:%UC% %someargs% -erase:All -unsecure -program %IMAGEFILE% -execute 

if not errorlevel 1 goto endexit
echo Error Programming image failed
echo |
goto end 

:endexit
echo All Done
exit

:end
