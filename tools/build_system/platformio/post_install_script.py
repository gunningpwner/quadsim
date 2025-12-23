Import("env")
import os
import shutil
import glob

print("Running script to remove Eigen's extra directories...")

# Get the path to the libraries for the current build environment
libdeps_dir = os.path.join(env.get("PROJECT_LIBDEPS_DIR"), env.get("PIOENV"))

# Find the Eigen library directory, which might have a suffix like _ID1234
eigen_path_list = glob.glob(os.path.join(libdeps_dir, "eigen*"))

if eigen_path_list:
    eigen_path = eigen_path_list[0]
    print(f"Found Eigen at: {eigen_path}")

    # Define the paths to the directories we want to delete
    dirs_to_delete = ["bench", "test", "unsupported","demos","doc","failtest","lapack"]

    for dirname in dirs_to_delete:
        dir_path = os.path.join(eigen_path, dirname)
        if os.path.exists(dir_path):
            print(f"  - Deleting directory: {dir_path}")
            try:
                shutil.rmtree(dir_path)
                print(f"    ...Successfully deleted.")
            except OSError as e:
                print(f"    ...Error deleting directory {dir_path}: {e}")
        else:
            print(f"  - Directory not found, skipping: {dir_path}")
else:
    print("Eigen library not found in libdeps, skipping.")