@echo off
echo.
echo  DruidaBot3.0 — Generando y pusheando frontend.bin
echo  ====================================================
echo.
python "%~dp0make_frontend.py"
echo.
if %errorlevel% neq 0 (
    echo  ERROR: algo salio mal. Revisa el log de arriba.
    pause
) else (
    echo  Presiona cualquier tecla para cerrar...
    pause >nul
)
