@REM this will probably only work on my machine, but should be easy enough to adapt (e.g. path to vcvarsall.bat)
@echo off
setlocal

set CONFIG=Release
set BUILD=vs2026_x64

pushd joltc\build

del /F /S /Q %BUILD%
call cmake_%BUILD%.bat

cd %BUILD%
call "c:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" amd64
msbuild joltc.slnx /p:Configuration=%CONFIG%

popd

@REM NOTE: other builds (e.g. debug or double) have different file names
cp joltc\build\%BUILD%\lib\%CONFIG%\joltc.lib jolt
cp joltc\build\%BUILD%\bin\%CONFIG%\joltc.dll .
@REM cp joltc\build\%BUILD%\bin\%CONFIG%\joltc.dll ..
