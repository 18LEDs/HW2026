# Compiles morse_puzzle.ino for the Wokwi simulator.
# Run this whenever you change the sketch, then (re)start the Wokwi sim.
$ErrorActionPreference = "Stop"
$cli = "C:\Program Files\Arduino CLI\arduino-cli.exe"
$sketch = Join-Path $PSScriptRoot "morse_puzzle"

& $cli compile `
    --fqbn esp32:esp32:esp32 `
    --output-dir (Join-Path $sketch "build") `
    $sketch

Write-Host "Build output -> morse_puzzle/build/  (wokwi.toml points here)"
