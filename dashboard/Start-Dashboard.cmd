@echo off
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel% equ 0 (
  py -3 dashboard.py
) else (
  python dashboard.py
)
echo.
echo If Python was not found, install Python 3 from https://www.python.org/downloads/windows/
echo Select Add Python to PATH during installation, then reopen this launcher.
pause
