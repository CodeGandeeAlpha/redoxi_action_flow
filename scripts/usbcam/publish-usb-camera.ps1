# Get all camera devices using pnputil
$cameras = pnputil /enum-devices /class Camera | Select-String "Device Description:" 

# Extract and format camera names
$cameraNames = $cameras | ForEach-Object {
    # Remove "Device Description:" prefix and trim only the 8 leading spaces
    $_.Line -replace "Device Description:", "" -replace "^         ", ""
}

# Print camera names with index
if ($cameraNames.Count -gt 0) {
    Write-Host "Found USB cameras:"
    for ($i = 0; $i -lt $cameraNames.Count; $i++) {
        Write-Host "[$i] $($cameraNames[$i])"
    }
} else {
    Write-Host "No USB cameras found"
    exit 1
}

# Default values
$USB_NAME = $cameraNames[0]
$PORT = 5555
$PROTOCOL = "udp"
$ENCODE_DEVICE = "cpu"

# Parse command line arguments
foreach ($arg in $args) {
    if ($arg -match "^--camera-name=(.+)$") {
        $USB_NAME = $matches[1]
    }
    elseif ($arg -match "^--port=(\d+)$") {
        $PORT = $matches[1]
    }
    elseif ($arg -match "^--protocol=(udp|rtp|rtsp|tcp)$") {
        $PROTOCOL = $matches[1]
    }
    elseif ($arg -match "^--encode-device=(gpu|cpu)$") {
        $ENCODE_DEVICE = $matches[1]
    }
}

# Build the output URL based on protocol
$outputUrl = switch ($PROTOCOL) {
    "udp"  { "udp://localhost:$PORT/live" }
    "rtp"  { "rtp://localhost:$PORT/live" }
    "rtsp" { "rtsp://localhost:$PORT/live" }
    "tcp"  { "tcp://localhost:$PORT/live" }
}

# Check if NVIDIA GPU is available when GPU encoding is requested
$hasNvidia = $null -ne (Get-WmiObject Win32_VideoController | Where-Object { $_.Name -like "*NVIDIA*" })
if ($ENCODE_DEVICE -eq "gpu" -and -not $hasNvidia) {
    Write-Host "Warning: NVIDIA GPU not found, falling back to CPU encoding"
    $ENCODE_DEVICE = "cpu"
}

# Run ffmpeg with the specified or default values
$baseCommand = "ffmpeg -f dshow -i video=`"$USB_NAME`" -fflags nobuffer -flags low_delay"
$format = if ($PROTOCOL -eq "rtsp" -or $PROTOCOL -eq "rtp") { $PROTOCOL } else { "mpegts" }

$encoderSettings = if ($ENCODE_DEVICE -eq "gpu") {
    "-c:v h264_nvenc -preset fast -b:v 10M"
} else {
    "-preset ultrafast -vcodec libx264 -tune zerolatency -b:v 10M"
}

$command = "$baseCommand $encoderSettings -f $format $outputUrl"
Write-Host "Running command: $command"
Invoke-Expression $command