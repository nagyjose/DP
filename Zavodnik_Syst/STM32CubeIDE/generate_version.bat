@echo off
cd /d "%~dp0"

:: Získání verze z Gitu
FOR /F "tokens=*" %%i IN ('git describe --tags --always --dirty 2^>nul') DO SET GIT_STR=%%i
IF "%GIT_STR%"=="" SET GIT_STR=unversioned

:: Přesná cesta k tvé složce src (podle screenshotu)
set OUT_FILE=Application\User\Core\src\version.c

:: Vygenerování souboru
echo // TENTO SOUBOR JE GENEROVAN AUTOMATICKY PRED KAZDOU KOMPILACI > %OUT_FILE%
echo // NEUPRAVOVAT! >> %OUT_FILE%
echo. >> %OUT_FILE%
echo const char fw_ver[] = "%GIT_STR% (Z)"; >> %OUT_FILE%
echo const char git_hash[] = "%GIT_STR%"; >> %OUT_FILE%

:: Vynucené ohlášení úspěchu pro Eclipse
exit 0