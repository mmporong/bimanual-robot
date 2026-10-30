@echo off
chcp 65001 >nul
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
"C:\Users\USER\miniconda3\envs\so101_env\python.exe" "%~dp0run_teleop_test.py" %*
