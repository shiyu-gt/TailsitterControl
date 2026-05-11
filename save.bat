@echo off
REM 快捷保存脚本 - Windows 版本
cd /d "%~dp0"

git diff --quiet && git diff --cached --quiet
if %errorlevel%==0 (
    echo 没有修改需要保存
    exit /b 0
)

echo === 修改文件 ===
git diff --name-only
echo.

git add -A
for /f "tokens=*" %%a in ('date /t') do set DATE=%%a
git commit -m "auto-save: %DATE% %TIME%"
echo ✓ 已保存
