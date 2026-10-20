"""Run real firmware math/sample helpers on Windows using installed MSVC.

Only RTOS queue access is mocked. Firmware functions are extracted unchanged;
this is not a Python reimplementation of the algorithm.
Usage: python tests/run_safety_tests.py
"""
from pathlib import Path
import os
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / 'build' / 'Debug' / 'host-tests'
out.mkdir(parents=True, exist_ok=True)
source = (root / 'Tasks/Filter_task/filter_task.c').read_text(encoding='utf-8')
helpers = source[source.index('#define SAMPLE_STALE_MS'):source.index('static void vTask_Filter_BodyFunction')]
prelude = r'''
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "imu_common.h"
typedef IMU_Sample_t *QueueHandle_t;
#define pdTRUE 1
static int xQueuePeek(QueueHandle_t q, IMU_Sample_t *out, int timeout)
{ (void)timeout; *out = *q; return pdTRUE; }
'''
test = (root / 'tests/test_filter_safety.c').read_text(encoding='utf-8')
filter_header = (root / 'Tasks/Filter_task/filter_task.h').read_text(encoding='utf-8')
temperature_macro = next(line for line in filter_header.splitlines()
                         if line.startswith('#define TASK_FILTER_TEMPERATURE_IIR_ALPHA'))
(out / 'filter_test.c').write_text(prelude + temperature_macro + '\n' + helpers + test, encoding='utf-8')
uart_source = (root / 'Tasks/UART_task/uart_task.c').read_text(encoding='utf-8')
prepare = uart_source[uart_source.index('static void PrepareTelemetry'):uart_source.index('static void vTask_UART_BodyFunction')]
fusion_type = filter_header[filter_header.index('typedef struct {'):filter_header.index('} FusionOutput_t;') + len('} FusionOutput_t;')]
(out / 'telemetry_test.c').write_text(
    '#include <math.h>\n#include <string.h>\n#include "Protocol_IMU.h"\n'
    + fusion_type + '\n' + prepare
    + (root / 'tests/test_telemetry_status.c').read_text(encoding='utf-8'), encoding='utf-8')
vcvars = Path(os.environ.get('ProgramFiles', 'C:/Program Files')) / 'Microsoft Visual Studio/2022/Community/VC/Auxiliary/Build/vcvars64.bat'
batch = out / 'run.cmd'
batch.write_text(f'''@echo off
call "{vcvars}" >nul
if errorlevel 1 exit /b 1
cl /nologo /std:c11 /W4 /utf-8 /I"{root / 'Tasks/IMU_task'}" filter_test.c /Fe:filter_test.exe
if errorlevel 1 exit /b 1
filter_test.exe
if errorlevel 1 exit /b 1
''', encoding='utf-8')
with batch.open('a', encoding='utf-8') as stream:
    stream.write(f'''cl /nologo /std:c11 /W4 /utf-8 /I"{root / 'Protocol'}" telemetry_test.c "{root / 'Protocol/Protocol_IMU.c'}" /Fe:telemetry_test.exe
if errorlevel 1 exit /b 1
telemetry_test.exe
if errorlevel 1 exit /b 1
''')
    for name in ('LSM', 'MPU', 'BMI'):
        stream.write(f'''cl /nologo /std:c11 /W4 /utf-8 /DTEST_{name} /I"{root / 'tests/stubs'}" "{root / 'tests/test_driver_safety.c'}" /Fe:driver_{name}.exe
if errorlevel 1 exit /b 1
driver_{name}.exe
if errorlevel 1 exit /b 1
''')
subprocess.run(['cmd', '/c', str(batch)], cwd=out, check=True)
