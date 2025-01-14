#!/bin/bash

echo "Building project .."
cd build && cmake --build .
echo "Running Extended Kalman Filter"
./extended_kalman_filter

# Run python display
if [[ $1 -eq 1 ]]; then
    cd ../python
    python3 display_output.py
fi
