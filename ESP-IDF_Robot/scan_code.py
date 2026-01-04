import os

# Configure this to your project root
PROJECT_ROOT = "/home/abobkov/MyProjects/ESP-Nodes/ESP-IDF_Robot"
OUTPUT_MD = "project_sources.md"

# Folders to scan (relative to project root)
TARGET_FOLDERS = [
    "main",
]

# File extensions to include
SOURCE_EXTENSIONS = {
    ".c", ".h", ".cpp", ".hpp",
    ".py", ".txt", ".md", ".cmake",
    ".yml", ".yaml", ".projbuild",
}

# Special filenames to include even without extension
SPECIAL_FILES = {
    "CMakeLists.txt",
    "Makefile",
    "LICENSE",
    "README",
    "README.md",
    "sdkconfig",
    "sdkconfig.old",
    "sdkconfig.ci.led_strip_spi",
}

def is_source_file(filename):
    # Special cases
    if filename in SPECIAL_FILES:
        return True

    # Extension-based detection
    _, ext = os.path.splitext(filename)
    return ext in SOURCE_EXTENSIONS

def collect_sources(root):
    sources = []

    # Scan only selected folders
    for folder in TARGET_FOLDERS:
        full_path = os.path.join(root, folder)
        if not os.path.exists(full_path):
            continue

        for dirpath, _, filenames in os.walk(full_path):
            for f in filenames:
                if is_source_file(f):
                    sources.append(os.path.join(dirpath, f))

    # Also include top-level files
    for f in os.listdir(root):
        if is_source_file(f):
            sources.append(os.path.join(root, f))

    return sources

def write_markdown(sources, output_file):
    with open(output_file, "w", encoding="utf-8") as md:
        md.write("# Project Source Archive\n\n")
        md.write("Generated automatically for analysis.\n\n")

        for path in sources:
            md.write(f"## File: `{path}`\n\n")
            md.write("```text\n")

            try:
                with open(path, "r", encoding="utf-8", errors="replace") as src:
                    md.write(src.read())
            except Exception as e:
                md.write(f"[Error reading file: {e}]")

            md.write("\n```\n\n")

if __name__ == "__main__":
    sources = collect_sources(PROJECT_ROOT)
    write_markdown(sources, OUTPUT_MD)
    print(f"Markdown document generated: {OUTPUT_MD}")
