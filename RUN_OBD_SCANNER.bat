@echo off
cd /d "%~dp0"
py OBD.py
if errorlevel 1 pause
