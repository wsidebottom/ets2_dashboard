@echo off

set SDK_INC_DIR=..\scs_sdk_1_5\include
set SRC_FILES=plugin.def plugin.cpp serial.cpp options.cpp

cl /nologo /W4 /LD /MD /EHsc /O2 /I%SDK_INC_DIR% %SRC_FILES% /link /OUT:dash_plugin.dll
