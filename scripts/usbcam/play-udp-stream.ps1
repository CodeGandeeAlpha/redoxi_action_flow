# Default port value
$PORT = 5555

# Parse command line arguments
foreach ($arg in $args) {
    if ($arg -match "^--port=(\d+)$") {
        $PORT = $matches[1]
    }
}

# Run ffplay with the specified or default port
ffplay -fflags nobuffer -flags low_delay -framedrop udp://127.0.0.1:$PORT