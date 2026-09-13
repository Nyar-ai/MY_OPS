@echo off
REM ============================================================================
REM  MY_OPS pose module - PC unit tests
REM  Builds with MinGW gcc (no make, no hardware required).
REM  Exit code 0 = all tests passed.
REM  NOTE: keep this file ASCII-only; cmd.exe parses .bat with the OEM codepage.
REM ============================================================================
setlocal

set ROOT=%~dp0..
set CC=gcc
set CFLAGS=-std=c99 -Wall -Wextra -Wno-unused-parameter -O1 -I "%ROOT%\Core\Inc"
set OUT=%~dp0test_ops.exe

echo [1/2] compiling...
%CC% %CFLAGS% "%~dp0test_ops.c" ^
  "%ROOT%\Core\Src\ops_crc.c" ^
  "%ROOT%\Core\Src\ops_geom.c" ^
  "%ROOT%\Core\Src\ops_cyz_proto.c" ^
  "%ROOT%\Core\Src\ops_frame.c" ^
  "%ROOT%\Core\Src\ops_fusion.c" ^
  "%ROOT%\Core\Src\ops_sample.c" ^
  -o "%OUT%"
if errorlevel 1 (
  echo COMPILE FAILED
  exit /b 1
)

echo [2/2] running...
"%OUT%"
set RC=%errorlevel%
if "%RC%"=="0" echo TESTS PASSED
if not "%RC%"=="0" echo TESTS FAILED, exit code %RC%

endlocal & exit /b %RC%
