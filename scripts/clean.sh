#!/bin/bash

# ==========================================
# FLIGHT CONTROLLER — CLEAN BUILD
# ==========================================

echo "Cleaning build directories..."

rm -rf build/
rm -f *.o
rm -f firmware/src/**/*.o
rm -f tests/*.o

echo "✓ Clean complete"