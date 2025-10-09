#!/usr/bin/env python3


import csv
import os

def txt_to_csv(txt_path, csv_path):
    """Convert a space-separated TXT file to a comma-separated CSV file."""
    try:
        # Ensure the output directory exists
        os.makedirs(os.path.dirname(csv_path), exist_ok=True)

        with open(txt_path, 'r') as infile, open(csv_path, 'w', newline='') as outfile:
            writer = csv.writer(outfile)
            for line in infile:
                parts = line.strip().split()
                if parts:
                    writer.writerow(parts)

        print(f"✅ Successfully converted:\n  {txt_path}\n→ {csv_path}")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    txt_path = "/home/heven/catkin_ws/src/heven_simul_2025/mm_common/txt/25molit_main_mission.txt"
    csv_path = "/home/heven/catkin_ws/src/heven_simul_2025/mm_common/csv/main.csv"

    if not os.path.isfile(txt_path):
        print(f"❌ Input file not found: {txt_path}")
    else:
        txt_to_csv(txt_path, csv_path)
