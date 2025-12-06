#!/usr/bin/env python3
"""Analyze diagrams in documentation for completeness."""

import re
from pathlib import Path
from collections import defaultdict

def analyze_diagrams():
    """Count and analyze Mermaid diagrams in markdown files."""
    docs_dir = Path("docs")

    # Track diagrams
    total_diagrams = 0
    diagrams_by_file = defaultdict(list)
    diagrams_with_title = 0

    # Exclude tutorial directories (Docusaurus defaults)
    exclude_patterns = ['tutorial-basics', 'tutorial-extras']

    for md_file in docs_dir.rglob("*.md"):
        # Skip if in excluded directories
        if any(exc in str(md_file) for exc in exclude_patterns):
            continue

        content = md_file.read_text()

        # Find all Mermaid code blocks
        mermaid_blocks = re.findall(r'```mermaid\n(.*?)\n```', content, re.DOTALL)

        if mermaid_blocks:
            # Find titles/captions (lines before mermaid blocks or after)
            lines = content.split('\n')
            for i, line in enumerate(lines):
                if '```mermaid' in line:
                    total_diagrams += 1

                    # Check for title in previous line(s)
                    title = None
                    if i > 0:
                        prev_line = lines[i-1].strip()
                        if prev_line and not prev_line.startswith('```'):
                            title = prev_line
                            diagrams_with_title += 1

                    diagrams_by_file[str(md_file.relative_to(docs_dir))].append({
                        'index': len(diagrams_by_file[str(md_file.relative_to(docs_dir))]) + 1,
                        'has_title': title is not None
                    })

    # Print report
    print(f"Total Mermaid diagrams: {total_diagrams}")
    if total_diagrams > 0:
        print(f"Diagrams with titles/captions: {diagrams_with_title}/{total_diagrams} ({diagrams_with_title/total_diagrams*100:.0f}%)")
    else:
        print(f"Diagrams with titles/captions: 0/0 (N/A)")
    print(f"\nDiagrams by file:")

    for file_path in sorted(diagrams_by_file.keys()):
        count = len(diagrams_by_file[file_path])
        print(f"  {file_path}: {count} diagram(s)")

    print(f"\nSource files in docs/assets/diagrams/source/:")
    source_dir = Path("docs/assets/diagrams/source")
    if source_dir.exists():
        source_files = list(source_dir.glob("*"))
        print(f"  Found {len(source_files)} source file(s)")
        for sf in source_files:
            print(f"    - {sf.name}")
    else:
        print("  Directory not found")

    print(f"\nRendered files in docs/assets/diagrams/rendered/:")
    rendered_dir = Path("docs/assets/diagrams/rendered")
    if rendered_dir.exists():
        rendered_files = list(rendered_dir.glob("*"))
        print(f"  Found {len(rendered_files)} rendered file(s)")
        for rf in rendered_files:
            print(f"    - {rf.name}")
    else:
        print("  Directory not found")

    # Assessment
    print("\n" + "="*60)
    print("ASSESSMENT:")
    print("="*60)
    print(f"✓ Total diagrams: {total_diagrams} (target: 20+)")

    if diagrams_with_title / total_diagrams >= 0.8:
        print(f"✓ Diagrams with titles: {diagrams_with_title}/{total_diagrams} ({diagrams_with_title/total_diagrams*100:.0f}%)")
    else:
        print(f"⚠ Diagrams with titles: {diagrams_with_title}/{total_diagrams} ({diagrams_with_title/total_diagrams*100:.0f}%) - Consider adding more titles")

    print(f"\nNote: Mermaid diagrams are embedded in markdown source files.")
    print(f"Separate source files are only needed for complex diagrams created externally.")

    return total_diagrams, diagrams_with_title

if __name__ == "__main__":
    analyze_diagrams()
