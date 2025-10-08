@REM this will probably only work on my machine, since i have vcvarsall.bat in path
@echo off
setlocal

set CONFIG=Release
set BUILD=vs2022_x64

pushd joltc\build

del /F /S /Q %BUILD%
call cmake_%BUILD%.bat

cd %BUILD%
call vcvarsall.bat amd64
msbuild joltc.sln /p:Configuration=%CONFIG%

popd

@REM NOTE: other builds (e.g. debug or double) have different file names
cp joltc\build\%BUILD%\lib\%CONFIG%\joltc.lib jolt
cp joltc\build\%BUILD%\bin\%CONFIG%\joltc.dll .
@REM cp joltc\build\%BUILD%\bin\%CONFIG%\joltc.dll ..
