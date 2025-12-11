
import pandas as pd
from pathlib import Path

def inspect_csv(path):
    print(f"--- Inspecting {path.name} ---")
    try:
        # metadata usually takes some lines, let's try reading with no header first to find the header row
        with open(path, 'r') as f:
            lines = [f.readline() for _ in range(25)]
            for i, line in enumerate(lines):
                print(f"Line {i}: {line.strip()}")
        
        # Determining header row index (visually for now, then hardcode in script)
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    data_dir = Path("lab2_ryan/data_ryan")
    files = list(data_dir.glob("*.csv"))
    if not files:
        print("No files found!")
    else:
        inspect_csv(files[0])
