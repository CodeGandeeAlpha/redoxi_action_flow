@echo off
setlocal

:: Default values
set USB_NAME=" USB 2.0  CAMERA"
set PORT="5555"

:: Parse command line arguments
for %%A in (%*) do (
    if "%%~A"=="--usb-name" (
        set USB_NAME="%%~B"
    ) else if "%%~A"=="--port" (
        set PORT="%%~B"
    )
)

:: Run ffmpeg with the specified or default values
ffmpeg -f dshow -i video=%USB_NAME% -preset ultrafast -vcodec libx264 -tune zerolatency -b 6000k -f mpegts udp://127.0.0.1:%PORT%