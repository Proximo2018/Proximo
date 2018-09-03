rem @ECHO OFF

Set serialnr=%1
IF "%serialnr%" == "" (goto :missingparam)

REM writes at UICR CUSTOMER[0]
nrfjprog --eraseuicr
nrfjprog --memwr 0x10001080 --val %1

GOTO End

:missingparam

@Echo "usage: program_serial_nr.bat <serialnr>"
:End
pause
