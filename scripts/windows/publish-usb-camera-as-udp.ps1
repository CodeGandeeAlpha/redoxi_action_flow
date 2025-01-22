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

# Parse command line arguments
foreach ($arg in $args) {
    if ($arg -match "^--camera-name=(.+)$") {
        $USB_NAME = $matches[1]
    }
    elseif ($arg -match "^--port=(\d+)$") {
        $PORT = $matches[1]
    }
}

# Run ffmpeg with the specified or default values
ffmpeg -f dshow -i video="$USB_NAME" -preset ultrafast -vcodec libx264 -tune zerolatency -b 6000k -f mpegts udp://127.0.0.1:$PORT