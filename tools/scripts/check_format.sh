#!/bin/bash

FIX=0
DIR="."

for arg in "$@"; do
    case "$arg" in
        --fix) FIX=1 ;;
        *) DIR="$arg" ;;
    esac
done

# Find all C++ source/header files
FILES=$(find "$DIR" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.cc" -o -name "*.h" \))

if [ "$FIX" -eq 1 ]; then
    echo "$FILES" | xargs clang-format -i
    echo "Done formatting all C++ files."
    exit 0
fi

NOT_FORMATTED=()

for file in $FILES; do
    if ! diff -q "$file" <(clang-format "$file") >/dev/null; then
        NOT_FORMATTED+=("$file")
    fi
done

if [ ${#NOT_FORMATTED[@]} -eq 0 ]; then
    echo "All files are properly formatted."
else
    echo "The following files are not properly formatted:"
    for f in "${NOT_FORMATTED[@]}"; do
        echo "  $f"
    done
    echo ""
    echo "Run './tools/scripts/check_format.sh --fix' to fix them."
    exit 1
fi