@ECHO OFF

SET version=20.1C.6-0
SET settings_date=%1

SET release_folder=%~dp0releases
SET backup_folder=%~dp0releases\backup

  ECHO CD src
  ECHO CALL clean.bat      || GOTO :EXIT
  ECHO CALL compile.bat    || GOTO :EXIT

ECHO Copying firmware to release folder.
ECHO %release_folder%\TSDZ2-%version%-PROGRAM.hex
  ECHO MKDIR "%release_folder%" >NUL 2>NUL
  ECHO COPY ..\bin\main.hex "%release_folder%\TSDZ2-%version%.hex"
  ECHO MKDIR "%backup_folder%" >NUL 2>NUL
  ECHO COPY ..\bin\main.hex "%backup_folder%\TSDZ2-%settings_date%.hex" >NUL 2>NUL

echo Press any key to flash... (Ctl+C to stop)
pause > nul
ECHO CALL flash.bat
@ECHO OFF
echo.
echo Press any key to close...
pause > nul

:EXIT
EXIT