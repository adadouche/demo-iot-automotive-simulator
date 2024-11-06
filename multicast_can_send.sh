#!/bin/bash

# Function to convert candump output to cansend format
convert_to_cansend_format() {
    while IFS= read -r line; do
        # Extract the can_id and the data payload using awk
        # This uses a custom field separator to handle variable whitespace
        can_id=$(echo "$line" | awk -F '  +' '{print $3}')
        data=$(echo "$line" | awk -F '  +' '{for (i=5; i<=NF; i++) printf "%s", $i}' | tr -d '[] ' | sed 's/ //g')

        # Output in cansend format
        echo "${can_id}#${data}"
    done
}

# Main pipeline
candump vcan0 | convert_to_cansend_format | socat - UDP4-DATAGRAM:239.255.0.1:3030,reuseaddr
