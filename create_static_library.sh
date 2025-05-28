#!/bin/bash

set -e

# Allow multiple input files as arguments, or default to both if none given
if [[ $# -eq 0 ]]; then
    INPUT_FILES=("static_library_test_files.txt" "static_library_files.txt")
else
    INPUT_FILES=("$@")
fi

for INPUT_FILE in "${INPUT_FILES[@]}"; do
    if [[ ! -f "$INPUT_FILE" ]]; then
        echo "Input file $INPUT_FILE does not exist, skipping."
        continue
    fi

    while IFS= read -r line; do
        # Skip empty lines or comments
        [[ -z "$line" || "$line" =~ ^# || "$line" =~ ^// ]] && continue

        # Remove possible leading/trailing whitespace
        target_path=$(echo "$line" | xargs)
        filename=$(basename "$target_path")

        # Search for the file in the current directory (excluding the target path itself)
        found_path=$(find . -type f -name "$filename" ! -path "./$target_path" | head -n 1)

        if [[ -n "$found_path" ]]; then
            # Create target directory if it doesn't exist
            mkdir -p "$(dirname "$target_path")"
            # Copy the file
            cp "$found_path" "$target_path"
            echo "Copied $found_path -> $target_path"
        else
            echo "File $filename not found for $target_path"
        fi
    done < "$INPUT_FILE"
done

# Tar the two folders after all is done
tar -cvf static_library.tar static_library_test static_library
