param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)
$ErrorActionPreference = 'Stop'
$manifestPath = Join-Path $PSScriptRoot 'manifest.json'
$items = Get-Content -Raw -Encoding UTF8 $manifestPath | ConvertFrom-Json
if ($items -isnot [System.Array]) { $items = @($items) }
foreach ($item in $items) {
    $outPath = Join-Path $Root $item.original_path
    $outDir = Split-Path -Parent $outPath
    if (!(Test-Path $outDir)) { New-Item -ItemType Directory -Path $outDir | Out-Null }
    if (Test-Path $outPath) { Remove-Item -LiteralPath $outPath -Force }
    $out = [System.IO.File]::OpenWrite($outPath)
    try {
        for ($i = 0; $i -lt [int]$item.part_count; $i++) {
            $partPath = Join-Path (Join-Path $Root $item.part_dir) ('part_{0:D3}.bin' -f $i)
            $bytes = [System.IO.File]::ReadAllBytes($partPath)
            $out.Write($bytes, 0, $bytes.Length)
        }
    } finally {
        $out.Close()
    }
    $hash = (Get-FileHash -LiteralPath $outPath -Algorithm SHA256).Hash
    if ($hash -ne $item.sha256) {
        throw "SHA256 mismatch: $($item.original_path)"
    }
    Write-Host "restored $($item.original_path)"
}
