@echo off
REM SAINT.OS Build Script for Windows (Batch wrapper)
REM This script calls the PowerShell build script

powershell -ExecutionPolicy Bypass -File "%~dp0build.ps1" %*
