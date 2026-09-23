# Compiles esp32_servo_control.ino for the Wokwi simulator.
# Run this whenever you change the sketch, then (re)start the Wokwi sim.
$ErrorActionPreference = "Stop"
$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$sketch = Join-Path $PSScriptRoot "esp32_servo_control"

& $cli compile `
    --fqbn esp32:esp32:esp32 `
    --output-dir (Join-Path $sketch "build") `
    $sketch

Write-Host "Build output -> esp32_servo_control/build/  (wokwi.toml points here)"
