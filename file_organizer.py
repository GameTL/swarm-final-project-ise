#!/usr/bin/env python3
"""
File Organizer Script
Takes filenames like TurtleBot3Burger_8, replaces the last number with variable x,
and organizes files into folders based on the original number.

Usage:
    python file_organizer.py TurtleBot3Burger_8.json 1
    
This will:
1. Replace the '8' with '1' -> TurtleBot3Burger_1.json
2. Create folder '8' if it doesn't exist
3. Move the renamed file to folder '8'
"""

import os
import sys
import shutil
from pathlib import Path

def organize_file(filename, x, base_folder="test_result"):
    """
    Organize file by replacing last number with x and moving to folder named after original number
    
    Args:
        filename (str): Original filename (e.g., "TurtleBot3Burger_8.json")
        x (int): New number to replace the last number with
        base_folder (str): Base folder to organize files into
        
    Returns:
        tuple: (success, message)
    """
    try:
        # Check if file exists
        if not os.path.exists(filename):
            return False, f"Error: File '{filename}' not found"
        
        # Split filename and extension
        name_part, extension = os.path.splitext(filename)
        
        # Split by underscore to get parts
        parts = name_part.split('_')
        
        if len(parts) < 2:
            return False, f"Error: Filename '{filename}' doesn't contain underscore separator"
        
        # Get the last part (should be a number)
        last_part = parts[-1]
        
        # Validate that last part is a number
        try:
            original_number = int(last_part)
        except ValueError:
            return False, f"Error: Last part '{last_part}' is not a valid number"
        
        # Create new filename with x replacing the last number
        new_parts = parts[:-1] + [str(x)]
        new_filename = '_'.join(new_parts) + extension
        
        # Create base folder and subfolder named after original number
        folder_path = os.path.join(base_folder, str(original_number))
        os.makedirs(folder_path, exist_ok=True)
        
        # Define destination path
        destination = os.path.join(folder_path, new_filename)
        
        # Copy and rename the file to the destination
        shutil.copy2(filename, destination)
        
        return True, f"Successfully copied '{filename}' to '{destination}'"
        
    except Exception as e:
        return False, f"Error: {str(e)}"

def main():
    """Main function to handle command line arguments"""
    if len(sys.argv) != 3:
        print("Usage: python file_organizer.py <filename> <x>")
        print("Example: python file_organizer.py TurtleBot3Burger_8.json 1")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    try:
        x = int(sys.argv[2])
    except ValueError:
        print("Error: x must be a valid integer")
        sys.exit(1)
    
    success, message = organize_file(filename, x)
    
    if success:
        print(f"✅ {message}")
    else:
        print(f"❌ {message}")
        sys.exit(1)

def batch_organize(pattern="TurtleBot3Burger_*.json", x=1, base_folder="test_result"):
    """
    Batch organize multiple files matching a pattern
    
    Args:
        pattern (str): Glob pattern to match files
        x (int): New number to replace with
        base_folder (str): Base folder to organize files into
    """
    import glob
    
    files = glob.glob(pattern)
    if not files:
        print(f"No files found matching pattern: {pattern}")
        return
    
    print(f"Found {len(files)} files matching pattern: {pattern}")
    print(f"Organizing files into '{base_folder}' folder...")
    
    for filename in files:
        success, message = organize_file(filename, x, base_folder)
        if success:
            print(f"✅ {message}")
        else:
            print(f"❌ {message}")

if __name__ == "__main__":
    # Batch organize all TurtleBot3Burger files into test_result folder
    batch_organize("TurtleBot3Burger_*.json", 1, "test_result")
    
    # Uncomment the line below for single file processing instead
    # main()
