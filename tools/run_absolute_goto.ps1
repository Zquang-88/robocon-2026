param(
  [string]$Port = 'COM5',
  [int]$Baud = 57600,
  [double]$TargetX = 1000.0,
  [double]$TargetY = 1000.0,
  [double]$TargetYaw = -3.12,
  [double]$MaxSpeed = 300.0,
  [int]$TimeoutSeconds = 20
)

$serial = [System.IO.Ports.SerialPort]::new($Port, $Baud, 'None', 8, 'One')
$serial.NewLine = "`n"
$serial.ReadTimeout = 50
$serial.WriteTimeout = 500
$buffer = ''
$completed = $false
$failed = $false
$lastHeartbeat = [DateTime]::UtcNow.AddSeconds(-1)

function Send-Line([string]$line) {
  $script:serial.WriteLine($line)
  Write-Output "TX,$line"
}

function Read-Lines {
  $chunk = $script:serial.ReadExisting()
  if ($chunk.Length -eq 0) { return @() }
  $script:buffer += $chunk
  $parts = $script:buffer -split "`n"
  $script:buffer = $parts[-1]
  if ($parts.Count -le 1) { return @() }
  return $parts[0..($parts.Count - 2)] | ForEach-Object { $_.Trim("`r", " ") } | Where-Object { $_ }
}

function Observe([int]$milliseconds) {
  $until = [DateTime]::UtcNow.AddMilliseconds($milliseconds)
  while ([DateTime]::UtcNow -lt $until) {
    if (([DateTime]::UtcNow - $script:lastHeartbeat).TotalMilliseconds -ge 100) {
      $script:serial.WriteLine('HEARTBEAT')
      $script:lastHeartbeat = [DateTime]::UtcNow
    }
    foreach ($line in (Read-Lines)) {
      if ($line -match '^(ACK|ERR|POSE|SYS|HDG|MOTION),') { Write-Output $line }
      if ($line -eq 'ACK,GOTO_DONE') { $script:completed = $true }
      if ($line -match '^ERR,') { $script:failed = $true }
      if ($line -match '^SYS,[^,]+,([1-9][0-9]*),') { $script:failed = $true }
      if ($line -match '^SYS,(?:[^,]*,){6}1$') { $script:failed = $true }
    }
    if ($script:completed -or $script:failed) { return }
    Start-Sleep -Milliseconds 20
  }
}

try {
  $serial.Open()
  Start-Sleep -Milliseconds 250
  $serial.DiscardInBuffer()

  Send-Line 'STOP'
  Observe 300
  if ($failed) { throw 'Robot reported an error before reset.' }

  Send-Line 'POSE_RESET'
  Observe 300
  if ($failed) { throw 'POSE_RESET failed.' }

  Send-Line 'MODE,AUTO'
  Observe 300
  if ($failed) { throw 'MODE,AUTO failed.' }

  $goto = 'GOTO,{0:F1},{1:F1},{2:F2},{3:F1}' -f $TargetX,$TargetY,$TargetYaw,$MaxSpeed
  Send-Line $goto

  $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
  while (-not $completed -and -not $failed -and [DateTime]::UtcNow -lt $deadline) {
    Observe 100
  }

  if (-not $completed) {
    Send-Line 'STOP'
    if ($failed) { throw 'Robot reported an error during GOTO.' }
    throw 'Host timeout waiting for GOTO completion.'
  }
  Write-Output 'RESULT,GOTO_COMPLETED'
}
catch {
  $failed = $true
  Write-Output "RESULT,FAILED,$($_.Exception.Message)"
}
finally {
  if ($serial.IsOpen) {
    try { $serial.WriteLine('STOP'); Start-Sleep -Milliseconds 100; $serial.WriteLine('MODE,MANUAL') } catch {}
    Start-Sleep -Milliseconds 100
    $serial.Close()
  }
  $serial.Dispose()
}

if ($failed) { exit 1 }
