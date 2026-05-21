# Single-thread NO_BUFFERING write benchmark
# Uses VirtualAlloc (guaranteed 4096-aligned) and WriteFile directly.
# Uses IntPtr (not UIntPtr) for size params — avoids PowerShell 5.1 coercion bug.
Add-Type @"
using System;
using System.Runtime.InteropServices;
public class RawIO {
    [DllImport("kernel32.dll", SetLastError=true)]
    public static extern IntPtr VirtualAlloc(IntPtr addr, IntPtr size, uint type, uint protect);
    [DllImport("kernel32.dll", SetLastError=true)]
    public static extern bool VirtualFree(IntPtr addr, IntPtr size, uint type);
    [DllImport("kernel32.dll", CharSet=CharSet.Auto, SetLastError=true)]
    public static extern IntPtr CreateFile(string path, uint access, uint share,
        IntPtr sec, uint disp, uint flags, IntPtr tmpl);
    [DllImport("kernel32.dll", SetLastError=true)]
    public static extern bool WriteFile(IntPtr h, IntPtr buf, uint n, out uint written, IntPtr ov);
    [DllImport("kernel32.dll", SetLastError=true)]
    public static extern bool CloseHandle(IntPtr h);
}
"@

$chunkSize   = 44957696         # 8 × FRAME_STRIDE
$chunks      = 350              # ~15 GB total — enough to exhaust SLC cache on QLC drives
$totalMB     = [math]::Round($chunks * $chunkSize / 1MB)
$path        = "C:\Git\TARA-camera\nobuf_test.bin"

$MEM_COMMIT_RESERVE = 0x3000
$PAGE_READWRITE     = 0x04
$GENERIC_WRITE      = 0x40000000
$CREATE_ALWAYS      = 2
$NO_BUF_SEQ         = 0x28000000   # FILE_FLAG_NO_BUFFERING | FILE_FLAG_SEQUENTIAL_SCAN
$MEM_RELEASE        = 0x8000

$buf = [RawIO]::VirtualAlloc([IntPtr]::Zero, [IntPtr]$chunkSize, $MEM_COMMIT_RESERVE, $PAGE_READWRITE)
if ($buf -eq [IntPtr]::Zero) {
    Write-Host "ERROR: VirtualAlloc failed (error $([Runtime.InteropServices.Marshal]::GetLastWin32Error()))"
    exit 1
}
Write-Host "Buffer allocated at: $buf"

$h = [RawIO]::CreateFile($path, $GENERIC_WRITE, 0, [IntPtr]::Zero, $CREATE_ALWAYS, $NO_BUF_SEQ, [IntPtr]::Zero)
$INVALID = [IntPtr](-1)
if ($h -eq $INVALID) {
    Write-Host "ERROR: CreateFile failed (error $([Runtime.InteropServices.Marshal]::GetLastWin32Error()))"
    [RawIO]::VirtualFree($buf, [IntPtr]::Zero, $MEM_RELEASE) | Out-Null
    exit 1
}
Write-Host "File opened. Writing $chunks x $([math]::Round($chunkSize/1MB,2)) MB = $totalMB MB ..."

# --- Test 1: single file — report speed every 50 chunks to show SLC cache drop ---
$errors = 0
$reportEvery = 50
$swTotal = [Diagnostics.Stopwatch]::StartNew()
$swWindow = [Diagnostics.Stopwatch]::StartNew()
$windowStart = 0
for ($i = 0; $i -lt $chunks; $i++) {
    $w = [uint32]0
    $ok = [RawIO]::WriteFile($h, $buf, [uint32]$chunkSize, [ref]$w, [IntPtr]::Zero)
    if (-not $ok -or $w -ne [uint32]$chunkSize) { $errors++ }
    if ((($i + 1) % $reportEvery) -eq 0) {
        $windowChunks = ($i + 1) - $windowStart
        $windowMB  = [math]::Round($windowChunks * $chunkSize / 1MB)
        $windowMBs = [math]::Round($windowMB / $swWindow.Elapsed.TotalSeconds)
        $gbSoFar   = [math]::Round(($i + 1) * $chunkSize / 1GB, 2)
        Write-Host "  chunk $($i+1)/$chunks  ($gbSoFar GB written)  window: $windowMBs MB/s"
        $swWindow.Restart()
        $windowStart = $i + 1
    }
}
$swTotal.Stop()
[RawIO]::CloseHandle($h) | Out-Null
Remove-Item $path -Force -ErrorAction SilentlyContinue

$mbps   = [math]::Round($totalMB / $swTotal.Elapsed.TotalSeconds)
$msEach = [math]::Round($swTotal.Elapsed.TotalMilliseconds / $chunks, 2)
$mbEach = [math]::Round($chunkSize / 1MB, 2)
if ($errors -gt 0) {
    Write-Host "1-file:  WARNING $errors / $chunks writes failed"
} else {
    Write-Host "1-file avg:  $mbps MB/s  ($msEach ms per $mbEach MB write)"
}

# --- Test 2: 4 files interleaved (same as recording app) ---
$handles = @()
for ($i = 0; $i -lt 4; $i++) {
    $p = $path + "_$i.bin"
    $hh = [RawIO]::CreateFile($p, $GENERIC_WRITE, 0, [IntPtr]::Zero, $CREATE_ALWAYS, $NO_BUF_SEQ, [IntPtr]::Zero)
    if ($hh -eq [IntPtr](-1)) { Write-Host "CreateFile failed for file $i"; exit 1 }
    $handles += $hh
}

$errors2 = 0
$chunks2 = 33   # 4-file test stays small — we only need burst speed here
$sw2 = [Diagnostics.Stopwatch]::StartNew()
for ($i = 0; $i -lt $chunks2; $i++) {
    $w = [uint32]0
    $ok = [RawIO]::WriteFile($handles[$i % 4], $buf, [uint32]$chunkSize, [ref]$w, [IntPtr]::Zero)
    if (-not $ok -or $w -ne [uint32]$chunkSize) { $errors2++ }
}
$sw2.Stop()

for ($i = 0; $i -lt 4; $i++) {
    [RawIO]::CloseHandle($handles[$i]) | Out-Null
    Remove-Item ($path + "_$i.bin") -Force -ErrorAction SilentlyContinue
}

$totalMB2 = [math]::Round($chunks2 * $chunkSize / 1MB)
$mbps2    = [math]::Round($totalMB2 / $sw2.Elapsed.TotalSeconds)
$msEach2  = [math]::Round($sw2.Elapsed.TotalMilliseconds / $chunks2, 2)
if ($errors2 -gt 0) {
    Write-Host "4-file:  WARNING $errors2 / $chunks writes failed"
} else {
    Write-Host "4-file:  $mbps2 MB/s  ($msEach2 ms per $mbEach MB write)"
}

[RawIO]::VirtualFree($buf, [IntPtr]::Zero, $MEM_RELEASE) | Out-Null
