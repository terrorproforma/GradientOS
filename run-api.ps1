[CmdletBinding()]
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$ScriptArgs
)

$ErrorActionPreference = "Stop"

# Ensure we run from repository root.
Set-Location -LiteralPath $PSScriptRoot
$repoRoot = (Get-Location).Path
$venvScripts = Join-Path $repoRoot ".venv\Scripts"
$venvPython = Join-Path $venvScripts "python.exe"
$srcPath = Join-Path $repoRoot "src"
$activateScript = Join-Path $venvScripts "Activate.ps1"
$repoVenv = Join-Path $repoRoot ".venv"

# Activate repo environment first for consistent package/environment behavior.
if ((-not $env:VIRTUAL_ENV) -or ($env:VIRTUAL_ENV -ne $repoVenv)) {
    if (Test-Path -LiteralPath $activateScript) {
        . $activateScript
    }
}

# Prepend virtualenv Scripts to PATH when available.
if (Test-Path -LiteralPath $venvScripts) {
    $env:Path = "$venvScripts;$env:Path"
}

if ([string]::IsNullOrWhiteSpace($env:PYTHONPATH)) {
    $env:PYTHONPATH = $srcPath
} else {
    $env:PYTHONPATH = "$srcPath;$($env:PYTHONPATH)"
}

if (-not (Test-Path -LiteralPath $venvPython)) {
    Write-Error "Missing repo virtualenv interpreter at '$venvPython'. Use the single repo env from start.sh semantics: create/install .venv first."
    exit 1
}

if (-not [string]::IsNullOrWhiteSpace($env:VIRTUAL_ENV) -and (Test-Path -LiteralPath $env:VIRTUAL_ENV)) {
    $activeVenv = (Resolve-Path -LiteralPath $env:VIRTUAL_ENV).Path
    $repoVenv = (Resolve-Path -LiteralPath (Join-Path $repoRoot ".venv")).Path
    if ($activeVenv -ne $repoVenv) {
        Write-Warning "A different virtualenv is active: '$env:VIRTUAL_ENV'. This launcher will still use '$venvPython' to keep a single repo environment."
    }
}

$cmd = $venvPython
$cmdArgs = @("-m", "gradient_os.api.main")

Write-Host "[gradient-robotics] Launching API service from $repoRoot"
& $cmd @cmdArgs @ScriptArgs
exit $LASTEXITCODE
