@echo off
setlocal

set PORT="5555"

:parse_args
for %%A in (%*) do (
    if "%%~A"=="--port" (
        set PORT="%%~B"
    ) else (
        for /f "tokens=1,2 delims==" %%B in ("%%~A") do (
            if /i "%%B"=="--port" (
                set PORT="%%C"
            )
        )
    )
)

ffplay -fflags nobuffer -flags low_delay -framedrop udp://127.0.0.1:%PORT%