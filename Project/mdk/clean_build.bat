@echo off
echo Cleaning build output...
del /Q out_file\*.obj 2>nul
del /Q out_file\*.crf 2>nul
del /Q out_file\*.lst 2>nul
echo Build output cleaned successfully!
echo.
echo Please rebuild your project in Keil MDK.
pause
