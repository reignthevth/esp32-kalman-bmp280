@echo off
call C:\Espressif\idf_cmd_init.bat esp-idf-20ee62e792ea89630ac6a777ab3ebc57
cd /d C:\Users\brkn\Desktop\ESPIDF
idf.py set-target esp32
idf.py build
