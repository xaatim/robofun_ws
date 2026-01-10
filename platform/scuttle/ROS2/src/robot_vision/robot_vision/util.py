import pathlib
import sys
import os

def get_project_root():
    # 1. Get the path of the script currently running
    # resolve() handles the case where ROS symlinks the executable
    current_path = pathlib.Path(sys.argv[0]).resolve()

    # 2. Walk up the directory tree until we find the 'data' folder
    # This works whether you are in 'src' (dev) or 'install' (ros2 run)
    while current_path != current_path.parent: # Stop if we hit the root of the OS (/)
        
        # Check if the "data" folder exists in this directory
        possible_data_dir = current_path / "data"
        
        if possible_data_dir.exists() and possible_data_dir.is_dir():
            # FOUND IT! This is the workspace root.
            return current_path
        
        # Move up one level
        current_path = current_path.parent

    raise FileNotFoundError(
        "Could not find the project root. "
        "Ensure the 'data' folder exists in your workspace root."
    )

def resolve_project_path(relative_path_str):
    project_root = get_project_root()
    return project_root / pathlib.Path(relative_path_str)