#!/bin/bash

# List of input files containing file paths to process
input_files=("static_library_files.txt")

# Loop through each input file
for input in "${input_files[@]}"; do
    base_dir=""
    # Read each line from the input file
    while IFS= read -r line; do
        # Skip empty lines
        [[ -z "$line" ]] && continue

        # If line ends with '/', treat it as a base directory
        if [[ "$line" == */ ]]; then
            base_dir="$line"
            continue
        fi

        # Otherwise, treat the line as a destination file path
        dest="$line"
        
        # Extract the filename from the destination path
        filename=$(basename "$dest")

        # If the file is a static library (.a), search by filename only
        if [[ "$filename" == *.a ]]; then
            matches=($(find "$base_dir" -type f -name "$filename"))
        else
            # Remove the first two directories from the path to get the relative path
            relpath=$(echo "$dest" | cut -d'/' -f3-)
            # Search for files in base_dir matching the relative path
            matches=($(find "$base_dir" -type f -path "*/$relpath"))
        fi

        # If multiple matches are found, print them all
        if [[ ${#matches[@]} -gt 1 ]]; then
            echo "Multiple files found for $filename in $base_dir:"
            for match in "${matches[@]}"; do
                echo "  $match"
            done
            # Use the first match as the source file
            src="${matches[0]}"
        else
            # Use the single match (if any) as the source file
            src="${matches[0]}"
        fi

        # If a source file was found, copy it to the destination
        if [[ -n "$src" ]]; then
            mkdir -p "$(dirname "$dest")"
            cp "$src" "$dest"
            echo "Copied $src to $dest"
        else
            # Print an error if the file was not found
            echo "File $filename not found in $base_dir"
        fi

    done < "$input"
done
