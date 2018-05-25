REM Batch File to generate a private key for signing packages.  
start ./nrfutil.exe keys generate private.key
start ./nrfutil.exe keys display --key pk --format code private.key --out_file public_key.c
