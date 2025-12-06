#!/usr/bin/env python3

"""
analyze-readability.py - Analyze readability of documentation

Usage:
    python3 scripts/analyze-readability.py
    python3 scripts/analyze-readability.py --detailed

Description:
    Calculates Flesch-Kincaid Grade Level and Flesch Reading Ease
    for all markdown files. Target range is grade 11-14 (college level).
"""

import re
import sys
import argparse
from pathlib import Path
from collections import defaultdict

# ANSI color codes
GREEN = '\033[0;32m'
RED = '\033[0;31m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No Color

class ReadabilityAnalyzer:
    def __init__(self, docs_dir, verbose=False):
        self.docs_dir = Path(docs_dir)
        self.verbose = verbose

    def clean_markdown(self, content):
        """Remove markdown syntax and code blocks."""
        # Remove YAML frontmatter
        content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

        # Remove code blocks
        content = re.sub(r'```.*?```', '', content, flags=re.DOTALL)

        # Remove inline code
        content = re.sub(r'`[^`]+`', '', content)

        # Remove HTML comments
        content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

        # Remove URLs
        content = re.sub(r'https?://[^\s\)]+', '', content)

        # Remove markdown images
        content = re.sub(r'!\[.*?\]\(.*?\)', '', content)

        # Remove markdown links but keep text
        content = re.sub(r'\[([^\]]+)\]\([^\)]+\)', r'\1', content)

        # Remove markdown headers
        content = re.sub(r'^#+\s+', '', content, flags=re.MULTILINE)

        # Remove markdown formatting
        content = re.sub(r'\*\*([^\*]+)\*\*', r'\1', content)
        content = re.sub(r'\*([^\*]+)\*', r'\1', content)
        content = re.sub(r'__([^_]+)__', r'\1', content)
        content = re.sub(r'_([^_]+)_', r'\1', content)

        return content

    def count_syllables(self, word):
        """Estimate syllable count in a word."""
        word = word.lower()
        count = 0
        vowels = "aeiouy"
        previous_was_vowel = False

        for char in word:
            is_vowel = char in vowels
            if is_vowel and not previous_was_vowel:
                count += 1
            previous_was_vowel = is_vowel

        # Adjust for silent e
        if word.endswith('e'):
            count -= 1

        # Ensure at least one syllable
        if count == 0:
            count = 1

        return count

    def analyze_text(self, text):
        """Calculate readability metrics for text."""
        # Count sentences
        sentences = re.split(r'[.!?]+', text)
        sentences = [s.strip() for s in sentences if s.strip()]
        num_sentences = len(sentences)

        if num_sentences == 0:
            return None

        # Count words
        words = text.split()
        words = [w for w in words if w.strip()]
        num_words = len(words)

        if num_words == 0:
            return None

        # Count syllables
        num_syllables = sum(self.count_syllables(word) for word in words)

        # Calculate Flesch-Kincaid Grade Level
        # Grade = 0.39 * (words/sentences) + 11.8 * (syllables/words) - 15.59
        fk_grade = 0.39 * (num_words / num_sentences) + 11.8 * (num_syllables / num_words) - 15.59

        # Calculate Flesch Reading Ease
        # Score = 206.835 - 1.015 * (words/sentences) - 84.6 * (syllables/words)
        flesch_ease = 206.835 - 1.015 * (num_words / num_sentences) - 84.6 * (num_syllables / num_words)

        return {
            'sentences': num_sentences,
            'words': num_words,
            'syllables': num_syllables,
            'fk_grade': fk_grade,
            'flesch_ease': flesch_ease
        }

    def analyze_file(self, file_path):
        """Analyze a single markdown file."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Clean markdown
        text = self.clean_markdown(content)

        # Analyze
        return self.analyze_text(text)

    def interpret_grade(self, grade):
        """Interpret FK grade level."""
        if grade < 6:
            return "Elementary", RED
        elif grade < 9:
            return "Middle School", YELLOW
        elif grade < 11:
            return "High School", YELLOW
        elif grade < 14:
            return "College", GREEN
        elif grade < 16:
            return "College Graduate", GREEN
        else:
            return "Professional", YELLOW

    def interpret_ease(self, score):
        """Interpret Flesch Reading Ease score."""
        if score >= 90:
            return "Very Easy (5th grade)"
        elif score >= 80:
            return "Easy (6th grade)"
        elif score >= 70:
            return "Fairly Easy (7th grade)"
        elif score >= 60:
            return "Standard (8th-9th grade)"
        elif score >= 50:
            return "Fairly Difficult (10th-12th grade)"
        elif score >= 30:
            return "Difficult (College)"
        else:
            return "Very Difficult (College Graduate)"

    def analyze_all(self):
        """Analyze all markdown files."""
        print(f"{BLUE}================================{NC}")
        print(f"{BLUE}Readability Analysis{NC}")
        print(f"{BLUE}================================{NC}")
        print()

        md_files = list(self.docs_dir.rglob("*.md"))
        print(f"{BLUE}Analyzing {len(md_files)} markdown files...{NC}")
        print()

        results = []
        module_results = defaultdict(list)

        for md_file in sorted(md_files):
            metrics = self.analyze_file(md_file)

            if not metrics:
                continue

            rel_path = md_file.relative_to(self.docs_dir)
            results.append({
                'file': rel_path,
                'metrics': metrics
            })

            # Categorize by module
            if str(rel_path).startswith('module-1'):
                module_results['Module 1: ROS 2'].append(metrics)
            elif str(rel_path).startswith('module-2'):
                module_results['Module 2: Simulation'].append(metrics)
            elif str(rel_path).startswith('module-3'):
                module_results['Module 3: Isaac Sim'].append(metrics)
            elif str(rel_path).startswith('module-4'):
                module_results['Module 4: VLA'].append(metrics)
            elif str(rel_path).startswith('appendices'):
                module_results['Appendices'].append(metrics)
            else:
                module_results['Other'].append(metrics)

            if self.verbose:
                fk = metrics['fk_grade']
                level, color = self.interpret_grade(fk)
                print(f"  {rel_path}")
                print(f"    FK Grade: {color}{fk:.1f}{NC} ({level})")
                print(f"    Flesch Ease: {metrics['flesch_ease']:.1f} ({self.interpret_ease(metrics['flesch_ease'])})")
                print()

        # Calculate overall averages
        all_metrics = [r['metrics'] for r in results]
        avg_fk = sum(m['fk_grade'] for m in all_metrics) / len(all_metrics)
        avg_ease = sum(m['flesch_ease'] for m in all_metrics) / len(all_metrics)

        # Calculate module averages
        print(f"{BLUE}Readability by Section:{NC}")
        print()

        for module in ['Module 1: ROS 2', 'Module 2: Simulation', 'Module 3: Isaac Sim', 'Module 4: VLA', 'Appendices', 'Other']:
            if module not in module_results:
                continue

            metrics = module_results[module]
            if not metrics:
                continue

            avg_module_fk = sum(m['fk_grade'] for m in metrics) / len(metrics)
            level, color = self.interpret_grade(avg_module_fk)

            print(f"  {module:30s} FK Grade: {color}{avg_module_fk:5.1f}{NC} ({level})")

        print()
        print(f"{BLUE}================================{NC}")
        overall_level, overall_color = self.interpret_grade(avg_fk)
        print(f"{BLUE}Overall FK Grade Level: {overall_color}{avg_fk:.1f}{NC} ({overall_level})")
        print(f"{BLUE}Overall Flesch Ease: {avg_ease:.1f}{NC} ({self.interpret_ease(avg_ease)})")
        print(f"{BLUE}================================{NC}")
        print()

        # Check if within target range (grade 11-14)
        TARGET_MIN = 11.0
        TARGET_MAX = 14.0

        if avg_fk < TARGET_MIN:
            print(f"{YELLOW}⚠ Below target range (too easy){NC}")
            print(f"  Target: Grade {TARGET_MIN}-{TARGET_MAX}")
            print(f"  Current: Grade {avg_fk:.1f}")
            return 1
        elif avg_fk > TARGET_MAX:
            print(f"{YELLOW}⚠ Above target range (too difficult){NC}")
            print(f"  Target: Grade {TARGET_MIN}-{TARGET_MAX}")
            print(f"  Current: Grade {avg_fk:.1f}")
            return 1
        else:
            print(f"{GREEN}✓ Within target range (College level){NC}")
            print(f"  Target: Grade {TARGET_MIN}-{TARGET_MAX}")
            print(f"  Current: Grade {avg_fk:.1f}")
            return 0

def main():
    parser = argparse.ArgumentParser(description='Analyze documentation readability')
    parser.add_argument('--detailed', '-d', action='store_true', help='Show file-by-file results')
    args = parser.parse_args()

    # Get docs directory
    script_dir = Path(__file__).parent
    repo_root = script_dir.parent
    docs_dir = repo_root / 'docs'

    if not docs_dir.exists():
        print(f"{RED}✗ Error: docs/ directory not found{NC}")
        sys.exit(1)

    # Analyze readability
    analyzer = ReadabilityAnalyzer(docs_dir, verbose=args.detailed)
    sys.exit(analyzer.analyze_all())

if __name__ == '__main__':
    main()
