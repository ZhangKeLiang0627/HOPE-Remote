param(
  [string]$Cmd  = "help",
  [int]$ReadMs  = 1200,
  [int]$WaitMs  = 200,
  [string]$Port = "COM7",
  [int]$Baud    = 115200
)
$p = New-Object System.IO.Ports.SerialPort $Port,$Baud,None,8,One
$p.ReadTimeout = 2000; $p.WriteTimeout = 3000
$p.Open()
$p.DiscardInBuffer(); $p.DiscardOutBuffer()
$p.Write($Cmd + "`r")
Start-Sleep -Milliseconds $WaitMs
$out = ""
$deadline = (Get-Date).AddMilliseconds($ReadMs)
while ((Get-Date) -lt $deadline) {
  try { $out += $p.ReadExisting() } catch {}
  Start-Sleep -Milliseconds 50
}
$p.Close()
Write-Output $out
