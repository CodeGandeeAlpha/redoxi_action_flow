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