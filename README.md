# ProductionTest

## Prerequisites

- [`gcc-arm-none-eabi-10.3-2021.10`](https://developer.arm.com/downloads/-/gnu-rm/10-3-2021-10)
- Install to:
  - Windows: `C:/gcc/gcc-arm-none-eabi-10.3-2021.10`
  - Linux: `/usr/share/gcc-arm/gcc-arm-none-eabi-10.3-2021.10/`

## Building

Run `./build.bat`

Or, if you prefer to do things manually:

- In the `firmware` folder, run `make all config=debug`. The final binary can be found in `firmware/Build/Debug/mcu-tester/rrrc_eoltest.bin`.
- Create the following folders:
  - `package/`
  - `package/files/`
- Copy the binary to `package/files/rrrc_eoltest.bin`
- Copy `assets\JLink_Windows_V796k_i386.exe` to `package/files`
- Create `package/files/rrrc_eoltest.jlink` with the following content:
    ```
    loadbin rrrc_eoltest.bin, 0
    r
    exit
    ```
  This file will donwload the binary to address 0x00000000, reset the MCU then exit.
- Create `package/test.bat` with the following content:
    ```bat
    IF EXIST "C:\Program Files\SEGGER\JLink_V796k\JLink.exe" (
        SET JLINK_PATH="C:\Program Files\SEGGER\JLink_V796k\JLink.exe"
    ) ELSE (
        SET JLINK_PATH="C:\Program Files (x86)\SEGGER\JLink_V796k\JLink.exe"
    )

    %JLINK_PATH% -device ATSAMD51P19 -if SWD -speed 4000 -CommanderScript files/rrrc_eoltest.jlink
    ```

You can then distribute the `package` folder as is.

## Usage

On a Windows PC, install the J-Link drivers (`package/files/JLink_Windows_V796k_i386.exe`). When
asked, enable installing the legacy driver.

Double-check that the paths in `test.bat` are correct, modify them if the driver was installed to
some other location.

Place the PCBA into the tester. Connect the J-Link to your PC. Run `test.bat`. After the test has
finished with all-green and a loud buzz, measure voltages.
