REM @ECHO OFF
REM writes at UICR CUSTOMER[0] abcd0102
start /B nrfjprog --eraseuicr
start /B nrfjprog --memwr 0x10001080 --val 0xabcd0102
REM done
pause
