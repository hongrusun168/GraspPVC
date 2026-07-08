@echo off
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
cd /d "C:\Users\byd\Desktop\PVC_folder\SimGrasp_PVC\c++\QT\GC3DExample"
cl /EHsc /O2 /std:c++11 capture_console.cpp /I04_GCI_SDK\C++SDK\include /Idependencies\opencv\include /link 04_GCI_SDK\C++SDK\lib64\GC3D.lib 04_GCI_SDK\C++SDK\lib64\GC3DAlgorithm.lib dependencies\opencv\lib\opencv_world460.lib
if %ERRORLEVEL% EQU 0 (
    echo.
    echo ===================================
    echo  Build SUCCESS: capture_console.exe
    echo ===================================
) else (
    echo.
    echo ===================================
    echo  Build FAILED
    echo ===================================
)
