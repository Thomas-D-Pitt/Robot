@echo off
setlocal

:: Set variables
set SRC_DIR=%~dp0
set PI_USER=tdownes
set PI_HOST=192.168.1.167
set DEST_DIR="/home/%PI_USER%/Robot"

:: Copy directory using SCP
echo Copying files to Raspberry Pi...
scp -r %SRC_DIR% %PI_USER%@%PI_HOST%:%DEST_DIR%

echo Transfer complete.
pause
