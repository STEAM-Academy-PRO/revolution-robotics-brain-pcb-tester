@echo off
cd firmware
make all config=debug
cd ..
mkdir package
mkdir package\files
copy firmware\Build\Debug\mcu-tester\rrrc_eoltest.bin package\files\rrrc_eoltest.bin
copy assets\JLink_Windows_V796k_i386.exe package\files\JLink_Windows_V796k_i386.exe
copy assets\jlink-script package\files\rrrc_eoltest.jlink
copy assets\runner-script.bat package\test.bat
