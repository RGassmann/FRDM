@echo off

set FLASHCOMMAND=ARM_FlashProgrammer
set IMAGEFILE=%1
set UC=%2
cls
if [%2]==[] ( set POWER= )
echo Programming the image "%IMAGEFILE%" with UC = %UC%
%FLASHCOMMAND% -device=%UC% -erase=All -program %IMAGEFILE% -execute
if not errorlevel 1 goto endexit
echo Error Programming image failed
echo |
goto end 

:endexit
echo All Done
exit

:end
