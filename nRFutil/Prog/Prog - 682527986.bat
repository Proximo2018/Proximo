@ECHO OFF
SET /A minor_num=0
::Set all path to all files and configuration correctly here
SET file=test.txt
SET jlink_id= 682527986
SET bootloader_path=../../Bootloader_nRF
SET bootloader_prj_name=secure_bootloader_ble_s132_pca10040
SET prj_path=../../Proximo_nRF
SET sdk_path=../../nRF5_SDK_15.0.0
SET build_config=Release
SET prj_name=Proximo
SET segger_bin=C:/"Program Files"/SEGGER/"SEGGER Embedded Studio for ARM 3.34b"/bin

:: Check if the file where the UICR value to be programmed exist and compile the bootloader and application code when it exists.
IF EXIST %file% (
	ECHO File %file% found!
	SET /P minor_num=<%file%
	ECHO Read the value %minor_num% from %file% to write to the UICR register
	START /B /wait %segger_bin%/emBuild.exe -config "%build_config%" %prj_path%/%prj_name%.emProject
	START /B /wait %segger_bin%/emBuild.exe -config "%build_config%" %bootloader_path%/%bootloader_prj_name%.emProject
	START /B /wait ../nrfutil.exe settings generate --family NRF52 --application %prj_path%/Output/%build_config%/Exe/%prj_name%.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 BL_settings.hex
	START /B /wait ../mergehex.exe --merge %prj_path%/Output/%build_config%/Exe/%prj_name%.hex %sdk_path%/components/softdevice/s132/hex/s132_nrf52_6.0.0_softdevice.hex --output APP_SD.hex
	START /B /wait ../mergehex.exe --merge %bootloader_path%/Output/%build_config%/Exe/%bootloader_prj_name%.hex APP_SD.hex --output APP_SD_BL.hex
	START /B /wait ../mergehex.exe --merge BL_settings.hex App_SD_BL.hex --output APP_SD_BL_BLsettings.hex
	START /B /wait ../nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application %prj_path%/Output/%build_config%/Exe/%prj_name%.hex --sd-req 0xA8 --key-file ../private.key %build_config%.zip
	
	:: Now program the Proximo in a loop.
	GOTO program
) ELSE (
	ECHO File %file% not found!
	GOTO end
)

:program 
ECHO.
ECHO Connect the next %prj_name% for programming with UICR value %minor_num%
SET /P answer=Continue (n/!n)?

:: On every character or button press which is not equal to 'n', program a Proximo and store the incremented UICR value
if /I "%answer:~,1%" EQU "n" (
	ECHO command to execute for no	
	GOTO end
) ELSE (

	START /B /wait nrfjprog --snr %jlink_id% --eraseall
	START /B /wait nrfjprog --snr %jlink_id% --memwr 0x10001080 --val %minor_num%
	START /B /wait nrfjprog --snr %jlink_id% --program APP_SD_BL_BLsettings.hex
	START /B /wait nrfjprog --snr %jlink_id% --reset

	SET /A minor_num+=1
	ECHO Storing minor value %minor_num% to %file%
	ECHO %minor_num% > %file%
	GOTO program
)
	
:end  
ECHO Ending batch file 
PAUSE
