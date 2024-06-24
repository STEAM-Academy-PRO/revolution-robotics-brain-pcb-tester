IF EXIST "C:\Program Files\SEGGER\JLink_V796k\JLink.exe" (
    SET JLINK_PATH="C:\Program Files\SEGGER\JLink_V796k\JLink.exe"
) ELSE (
    SET JLINK_PATH="C:\Program Files (x86)\SEGGER\JLink_V796k\JLink.exe"
)

%JLINK_PATH% -device ATSAMD51P19 -if SWD -speed 4000 -CommanderScript files/rrrc_eoltest.jlink
