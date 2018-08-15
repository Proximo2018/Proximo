REM Compile the application first
start /B /wait C:/"Program Files"/SEGGER/"SEGGER Embedded Studio for ARM 3.34b"/bin/emBuild.exe -config "Release" ../../Proximo_nRF/Proximo.emProject
start /B /wait C:/"Program Files"/SEGGER/"SEGGER Embedded Studio for ARM 3.34b"/bin/emBuild.exe -config "Release" ../../Bootloader_nRF/secure_bootloader_ble_s132_pca10040.emProject
start /B /wait ../nrfutil.exe settings generate --family NRF52 --application ../../Proximo_nRF/Output/Release/Exe/Proximo.hex --application-version 1 --bootloader-version 1 --bl-settings-version 1 BL_settings.hex
start /B /wait ../mergehex.exe --merge ../../Proximo_nRF/Output/Release/Exe/Proximo.hex ../../nRF5_SDK_15.0.0/components/softdevice/s132/hex/s132_nrf52_6.0.0_softdevice.hex --output APP_SD.hex
start /B /wait ../mergehex.exe --merge ../../Bootloader_nRF/Output/Release/Exe/secure_bootloader_ble_s132_pca10040.hex APP_SD.hex --output APP_SD_BL.hex
start /B /wait ../mergehex.exe --merge BL_settings.hex App_SD_BL.hex --output APP_SD_BL_BLsettings.hex
start /B /wait ../nrfutil.exe pkg generate --hw-version 52 --application-version 1 --application ../../Proximo_nRF/Output/Release/Exe/Proximo.hex --sd-req 0x9B --key-file ../private.key release.zip
start /B /wait nrfjprog --snr 682818092 --eraseall
start /B /wait nrfjprog --snr 682818092 --program APP_SD_BL_BLsettings.hex
start /B /wait nrfjprog --snr 682818092 --reset
pause
