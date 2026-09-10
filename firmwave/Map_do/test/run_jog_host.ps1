param([string]$Compiler = 'C:\msys64\ucrt64\bin\g++.exe')
$ErrorActionPreference = 'Stop'
$project = Split-Path $PSScriptRoot -Parent
$library = Join-Path $project '.pio\libdeps\esp32-s3-devkitm-1\AccelStepper\src'
$outDir = Join-Path $project '.pio\build\jog-host'
New-Item -ItemType Directory -Path $outDir -Force | Out-Null
$output = Join-Path $outDir 'jog_test.exe'
$env:PATH = (Split-Path $Compiler -Parent) + ';' + $env:PATH
& $Compiler -std=c++17 -DARDUINO=100 '-I' "$PSScriptRoot\host" '-I' "$project\include" '-I' $library "$PSScriptRoot\host\jog_test.cpp" "$library\AccelStepper.cpp" "$project\src\MechanismController.cpp" "$project\src\StepperProfiles.cpp" "$project\src\ValveController.cpp" "$project\src\UartProtocol.cpp" -o $output
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& $output
exit $LASTEXITCODE
