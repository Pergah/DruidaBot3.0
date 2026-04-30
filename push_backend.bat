@echo off
setlocal EnableExtensions

echo.
echo  DruidaBot3.0 - Pusheando backend.ino.bin
echo  =========================================
echo.

set "PROJECT_DIR=%~dp0"
set "BUILD_DIR=%PROJECT_DIR%build"
set "REPO_URL=https://github.com/Pergah/DruidaBot3.0.git"
set "REPO_DIR=%USERPROFILE%\Documents\Arduino\DruidaBot_release"
set "TARGET_NAME=backend.ino.bin"
set "PREFERRED_BIN=%BUILD_DIR%\esp32.esp32.esp32s3\DruidaBot3.0.ino.bin"

if not exist "%BUILD_DIR%" (
    echo  ERROR: no existe la carpeta build:
    echo  "%BUILD_DIR%"
    goto error
)

if exist "%PREFERRED_BIN%" (
    set "SOURCE_BIN=%PREFERRED_BIN%"
) else (
    for /f "usebackq delims=" %%F in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-ChildItem -Path '%BUILD_DIR%' -Filter '*.ino.bin' -Recurse | Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName"`) do set "SOURCE_BIN=%%F"
)

if not defined SOURCE_BIN (
    echo  ERROR: no se encontro ningun archivo .ino.bin dentro de:
    echo  "%BUILD_DIR%"
    goto error
)

echo  Archivo origen:
echo  "%SOURCE_BIN%"
echo.

if exist "%REPO_DIR%\.git" (
    echo  Actualizando repo local:
    echo  "%REPO_DIR%"
    git -C "%REPO_DIR%" pull --rebase origin main
    if errorlevel 1 goto error
) else (
    echo  Clonando repo:
    echo  %REPO_URL%
    echo.
    if not exist "%REPO_DIR%" mkdir "%REPO_DIR%"
    git clone "%REPO_URL%" "%REPO_DIR%"
    if errorlevel 1 goto error
)

git -C "%REPO_DIR%" config user.name "Pergah"
git -C "%REPO_DIR%" config user.email "pergah@users.noreply.github.com"

echo.
echo  Copiando y renombrando a:
echo  "%REPO_DIR%\%TARGET_NAME%"
copy /Y "%SOURCE_BIN%" "%REPO_DIR%\%TARGET_NAME%" >nul
if errorlevel 1 goto error

git -C "%REPO_DIR%" add "%TARGET_NAME%"
if errorlevel 1 goto error

set "STATUS_FILE=%TEMP%\druidabot_backend_status.txt"
git -C "%REPO_DIR%" status --porcelain > "%STATUS_FILE%"
for %%A in ("%STATUS_FILE%") do set "STATUS_SIZE=%%~zA"

if "%STATUS_SIZE%"=="0" (
    echo.
    echo  No hay cambios para pushear. backend.ino.bin ya esta actualizado.
    goto success
)

echo.
echo  Creando commit...
git -C "%REPO_DIR%" commit -m "Update backend.ino.bin"
if errorlevel 1 goto error

echo.
echo  Pusheando a GitHub...
git -C "%REPO_DIR%" push origin main
if errorlevel 1 goto error

goto success

:error
echo.
echo  ERROR: algo salio mal. Revisa el log de arriba.
pause
exit /b 1

:success
echo.
echo  Listo. Presiona cualquier tecla para cerrar...
pause >nul
exit /b 0
