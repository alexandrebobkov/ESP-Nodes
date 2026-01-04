import os

# Configure this to your project root
PROJECT_ROOT = "/home/abobkov/MyProjects/ESP-Nodes/ESP-IDF_NovaGlide"
OUTPUT_MD = "project_sources.md"

# File extensions to include
SOURCE_EXTENSIONS = {
    ".c", ".h", ".cpp", ".hpp", ".py", ".txt", ".md",
    ".cmake", "CMakeLists.txt"
}

def is_source_file(filename):
    if filename == "CMakeLists.txt":
        return True
    _, ext = os.path.splitext(filename)
    return ext in SOURCE_EXTENSIONS

def collect_sources(root):
    collected = []
    for dirpath, _, filenames in os.walk(root):
        for f in filenames:
            if is_source_file(f):
                full_path = os.path.join(dirpath, f)
                collected.append(full_path)
    return collected

def write_markdown(sources, output_file):
    with open(output_file, "w") as md:
        md.write("# Project Source Archive\n\n")
        md.write("Generated for analysis.\n\n")

        for path in sources:
            md.write(f"## File: `{path}`\n\n")
            md.write("```text\n")
            try:
                with open(path, "r", errors="ignore") as src:
                    md.write(src.read())
            except Exception as e:
                md.write(f"[Error reading file: {e}]")
            md.write("\n```\n\n")

if __name__ == "__main__":
    sources = collect_sources(PROJECT_ROOT)
    write_markdown(sources, OUTPUT_MD)
    print(f"Markdown document generated: {OUTPUT_MD}")
