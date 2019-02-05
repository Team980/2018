$url = "devsite.ctr-electronics.com/maven/release/com/ctre/phoenix/Phoenix-latest.json"
$output = "$PSScriptRoot\Phoenix.json"
$start_time = Get-Date

Invoke-WebRequest -Uri $url -OutFile $output
Write-Output "Time taken: $((Get-Date).Subtract($start_time).Seconds) second(s)"