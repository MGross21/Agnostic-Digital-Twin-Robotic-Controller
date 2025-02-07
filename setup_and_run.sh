#!/bin/bash

# 1. Check if Anaconda or Miniconda is installed
if command -v conda &> /dev/null
then
    echo "Conda is already installed."
else
    echo "Conda not found. Please install Anaconda or Miniconda first."
    exit 1
fi

# 2. Ensure the script is running inside a cloned repository
if [ ! -f "environment.yml" ]; then
    echo "Error: No environment.yml file found in the current directory."
    echo "Make sure you're inside the cloned repository before running this script."
    exit 1
fi

# 3. Create the Conda environment
echo "Creating Conda environment from environment.yml..."
conda env create -f environment.yml

# 4. Extract environment name from environment.yml
ENV_NAME=$(grep "name:" environment.yml | awk '{print $2}')
echo "Activating Conda environment: $ENV_NAME"
conda activate $ENV_NAME

# 5. Run the Python script (replace with your desired entry point)
SCRIPT="main.py"  # Change this to the script you want to run
if [ -f "$SCRIPT" ]; then
    echo "Running the script: $SCRIPT..."
    python "$SCRIPT"
else
    echo "Error: Script $SCRIPT not found."
    exit 1
fi

# 6. Deactivate the Conda environment after execution
conda deactivate
