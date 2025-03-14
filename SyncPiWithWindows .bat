@echo off
setlocal

:: Set variables
set PI_USER=tdownes
set PI_HOST=192.168.1.167
set SRC_DIR="/home/%PI_USER%/Robot"
set DEST_DIR=%~dp0/tmp  :: Copies to the directory where this script is located

:: Copy directory from Raspberry Pi using SCP
echo Copying files from Raspberry Pi to local machine...
scp -r %PI_USER%@%PI_HOST%:%SRC_DIR% %DEST_DIR%

echo Transfer complete.
pause

