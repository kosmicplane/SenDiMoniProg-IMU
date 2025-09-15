
## !!THIS FILE IS AUTO-GENERATED, AND IT'S UNDERCONSTRUCTION, DON'T USE IT OR RUN IT YET
#!/bin/bash
set -e

echo "🚀 Installing dependencies for SenDiMoniProg-IMU..."

# --- Update system ---
sudo apt-get update

# --- Python dependencies ---
if [ -f requirements.txt ]; then
    echo "📦 Installing Python dependencies..."
    pip install --upgrade pip
    pip install -r requirements.txt
fi

echo "✅ All dependencies installed successfully."

