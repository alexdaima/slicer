#!/bin/bash

# Line Diff Script
# Compares the total line count of libslic3r C++ files vs Rust slicer files

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Paths (adjust these based on your setup)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SLICER_ROOT="$(dirname "$SCRIPT_DIR")"
# Default assumes BambuStudio is in Code/3rdParty relative to Code/Personal/slicer
BAMBU_LIBSLIC3R="${BAMBU_LIBSLIC3R:-$SLICER_ROOT/../../3rdParty/BambuStudio/src/libslic3r}"
RUST_LIB="${RUST_LIB:-$SLICER_ROOT/lib/src}"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Line Count Comparison: libslic3r vs Rust${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if directories exist
if [ ! -d "$BAMBU_LIBSLIC3R" ]; then
    echo -e "${RED}Error: BambuStudio libslic3r directory not found at:${NC}"
    echo "  $BAMBU_LIBSLIC3R"
    echo ""
    echo "Set BAMBU_LIBSLIC3R environment variable to the correct path."
    exit 1
fi

if [ ! -d "$RUST_LIB" ]; then
    echo -e "${RED}Error: Rust lib directory not found at:${NC}"
    echo "  $RUST_LIB"
    echo ""
    echo "Set RUST_LIB environment variable to the correct path."
    exit 1
fi

echo -e "${YELLOW}C++ Source:${NC} $BAMBU_LIBSLIC3R"
echo -e "${YELLOW}Rust Source:${NC} $RUST_LIB"
echo ""

# Count C++ lines
echo -e "${BLUE}Counting C++ lines (.cpp, .hpp, .h)...${NC}"

CPP_FILES=$(find "$BAMBU_LIBSLIC3R" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) | wc -l)
CPP_LINES=$(find "$BAMBU_LIBSLIC3R" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec cat {} \; | wc -l)
CPP_CODE_LINES=$(find "$BAMBU_LIBSLIC3R" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec cat {} \; | grep -v '^\s*$' | grep -v '^\s*//' | grep -v '^\s*\*' | wc -l)

echo -e "  Files: ${GREEN}$CPP_FILES${NC}"
echo -e "  Total lines: ${GREEN}$CPP_LINES${NC}"
echo -e "  Non-blank/comment lines (approx): ${GREEN}$CPP_CODE_LINES${NC}"
echo ""

# Count Rust lines
echo -e "${BLUE}Counting Rust lines (.rs)...${NC}"

RUST_FILES=$(find "$RUST_LIB" -type f -name "*.rs" | wc -l)
RUST_LINES=$(find "$RUST_LIB" -type f -name "*.rs" -exec cat {} \; | wc -l)
RUST_CODE_LINES=$(find "$RUST_LIB" -type f -name "*.rs" -exec cat {} \; | grep -v '^\s*$' | grep -v '^\s*//' | wc -l)

# Calculate percentages
if [ "$CPP_FILES" -gt 0 ]; then
    PERCENT_FILES=$(echo "scale=1; $RUST_FILES * 100 / $CPP_FILES" | bc)
else
    PERCENT_FILES="N/A"
fi

if [ "$CPP_LINES" -gt 0 ]; then
    PERCENT_TOTAL=$(echo "scale=1; $RUST_LINES * 100 / $CPP_LINES" | bc)
else
    PERCENT_TOTAL="N/A"
fi

if [ "$CPP_CODE_LINES" -gt 0 ]; then
    PERCENT_CODE=$(echo "scale=1; $RUST_CODE_LINES * 100 / $CPP_CODE_LINES" | bc)
else
    PERCENT_CODE="N/A"
fi

echo -e "  Files: ${GREEN}$RUST_FILES${NC} (${PERCENT_FILES}% of C++)"
echo -e "  Total lines: ${GREEN}$RUST_LINES${NC} (${PERCENT_TOTAL}% of C++)"
echo -e "  Non-blank/comment lines (approx): ${GREEN}$RUST_CODE_LINES${NC} (${PERCENT_CODE}% of C++)"
echo ""

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Summary${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}Rust vs C++ Comparison:${NC}"
printf "  %-35s %8s / %8s  (%s%%)\n" "Files:" "$RUST_FILES" "$CPP_FILES" "$PERCENT_FILES"
printf "  %-35s %8s / %8s  (%s%%)\n" "Total lines:" "$RUST_LINES" "$CPP_LINES" "$PERCENT_TOTAL"
printf "  %-35s %8s / %8s  (%s%%)\n" "Code lines (non-blank/comment):" "$RUST_CODE_LINES" "$CPP_CODE_LINES" "$PERCENT_CODE"
echo ""

# Calculate lines per file ratio
if [ "$RUST_FILES" -gt 0 ] && [ "$CPP_FILES" -gt 0 ]; then
    CPP_LINES_PER_FILE=$(echo "scale=0; $CPP_LINES / $CPP_FILES" | bc)
    RUST_LINES_PER_FILE=$(echo "scale=0; $RUST_LINES / $RUST_FILES" | bc)
    echo -e "${YELLOW}Average lines per file:${NC}"
    printf "  %-20s %8s lines/file\n" "C++:" "$CPP_LINES_PER_FILE"
    printf "  %-20s %8s lines/file\n" "Rust:" "$RUST_LINES_PER_FILE"
    echo ""
fi


# Breakdown by directory (top-level)
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Breakdown by Directory${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

echo -e "${YELLOW}C++ libslic3r subdirectories:${NC}"
for dir in "$BAMBU_LIBSLIC3R"/*/; do
    if [ -d "$dir" ]; then
        dirname=$(basename "$dir")
        lines=$(find "$dir" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec cat {} \; 2>/dev/null | wc -l)
        if [ "$lines" -gt 0 ]; then
            printf "  %-30s %8d lines\n" "$dirname/" "$lines"
        fi
    fi
done
# Root level files
root_lines=$(find "$BAMBU_LIBSLIC3R" -maxdepth 1 -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.h" \) -exec cat {} \; 2>/dev/null | wc -l)
printf "  %-30s %8d lines\n" "(root)" "$root_lines"
echo ""

echo -e "${YELLOW}Rust lib subdirectories:${NC}"
for dir in "$RUST_LIB"/*/; do
    if [ -d "$dir" ]; then
        dirname=$(basename "$dir")
        lines=$(find "$dir" -type f -name "*.rs" -exec cat {} \; 2>/dev/null | wc -l)
        if [ "$lines" -gt 0 ]; then
            printf "  %-30s %8d lines\n" "$dirname/" "$lines"
        fi
    fi
done
# Root level files
rust_root_lines=$(find "$RUST_LIB" -maxdepth 1 -type f -name "*.rs" -exec cat {} \; 2>/dev/null | wc -l)
printf "  %-30s %8d lines\n" "(root)" "$rust_root_lines"
echo ""

# Optional: detailed file-by-file comparison
if [ "$1" == "--detailed" ] || [ "$1" == "-d" ]; then
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Detailed File Listing${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    echo -e "${YELLOW}Top 20 largest C++ files:${NC}"
    find "$BAMBU_LIBSLIC3R" -type f \( -name "*.cpp" -o -name "*.hpp" \) -exec wc -l {} \; | sort -rn | head -20 | while read lines file; do
        printf "  %8d  %s\n" "$lines" "$(basename "$file")"
    done
    echo ""

    echo -e "${YELLOW}Top 20 largest Rust files:${NC}"
    find "$RUST_LIB" -type f -name "*.rs" -exec wc -l {} \; | sort -rn | head -20 | while read lines file; do
        printf "  %8d  %s\n" "$lines" "$(basename "$file")"
    done
fi

echo ""
echo -e "${GREEN}Done!${NC}"
echo ""
echo "Tip: Run with --detailed or -d flag for file-by-file breakdown"
