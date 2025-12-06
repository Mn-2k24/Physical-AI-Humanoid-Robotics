#!/usr/bin/env python3

"""
check-links.py - Validate all links in markdown documentation

Usage:
    python3 scripts/check-links.py
    python3 scripts/check-links.py --verbose

Description:
    Scans all markdown files for links and validates:
    - Internal links (files and anchors)
    - External links (HTTP/HTTPS URLs)
    - Relative paths
"""

import os
import re
import sys
import argparse
from pathlib import Path
from urllib.parse import urlparse
import requests
from collections import defaultdict

# ANSI color codes
GREEN = '\033[0;32m'
RED = '\033[0;31m'
YELLOW = '\033[1;33m'
BLUE = '\033[0;34m'
NC = '\033[0m'  # No Color

class LinkChecker:
    def __init__(self, docs_dir, verbose=False):
        self.docs_dir = Path(docs_dir)
        self.verbose = verbose
        self.internal_links = []
        self.external_links = []
        self.broken_links = []
        self.file_cache = set()

        # Build cache of all files
        for md_file in self.docs_dir.rglob("*.md"):
            self.file_cache.add(md_file.relative_to(self.docs_dir))

    def extract_links(self, file_path):
        """Extract all markdown links from a file."""
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Match markdown links: [text](url)
        link_pattern = r'\[([^\]]+)\]\(([^\)]+)\)'
        matches = re.findall(link_pattern, content)

        links = []
        for text, url in matches:
            # Skip anchor-only links
            if url.startswith('#'):
                continue

            links.append({
                'text': text,
                'url': url,
                'file': file_path.relative_to(self.docs_dir)
            })

        return links

    def check_internal_link(self, link, source_file):
        """Check if an internal link is valid."""
        url = link['url']

        # Remove anchor if present
        if '#' in url:
            url_path, anchor = url.split('#', 1)
        else:
            url_path = url

        # Resolve relative path
        source_dir = source_file.parent
        target_path = (source_dir / url_path).resolve()

        # Check if file exists
        try:
            target_relative = target_path.relative_to(self.docs_dir.resolve())
            if target_relative in self.file_cache:
                return True, None
            else:
                return False, f"File not found: {target_relative}"
        except ValueError:
            # Path is outside docs directory
            if target_path.exists():
                return True, None
            else:
                return False, f"File not found: {target_path}"

    def check_external_link(self, link):
        """Check if an external link is accessible."""
        url = link['url']

        try:
            # Send HEAD request (faster than GET)
            response = requests.head(url, timeout=5, allow_redirects=True)
            if response.status_code < 400:
                return True, None
            else:
                return False, f"HTTP {response.status_code}"
        except requests.exceptions.Timeout:
            return False, "Timeout"
        except requests.exceptions.ConnectionError:
            return False, "Connection error"
        except requests.exceptions.RequestException as e:
            return False, str(e)

    def is_external_link(self, url):
        """Check if a URL is external (HTTP/HTTPS)."""
        return url.startswith('http://') or url.startswith('https://')

    def check_all_links(self):
        """Check all links in all markdown files."""
        print(f"{BLUE}================================{NC}")
        print(f"{BLUE}Link Validation Report{NC}")
        print(f"{BLUE}================================{NC}")
        print()

        md_files = list(self.docs_dir.rglob("*.md"))
        print(f"{BLUE}Scanning {len(md_files)} markdown files...{NC}")
        print()

        total_links = 0
        broken_links = []

        for md_file in sorted(md_files):
            links = self.extract_links(md_file)

            if not links:
                continue

            if self.verbose:
                print(f"{YELLOW}File:{NC} {md_file.relative_to(self.docs_dir)}")

            for link in links:
                total_links += 1
                url = link['url']

                if self.is_external_link(url):
                    valid, error = self.check_external_link(link)
                    link_type = "external"
                else:
                    valid, error = self.check_internal_link(link, md_file)
                    link_type = "internal"

                if not valid:
                    broken_links.append({
                        **link,
                        'error': error,
                        'type': link_type
                    })
                    if self.verbose:
                        print(f"  {RED}✗{NC} [{link['text']}]({url}) - {error}")
                elif self.verbose:
                    print(f"  {GREEN}✓{NC} [{link['text']}]({url})")

            if self.verbose:
                print()

        # Print summary
        print(f"{BLUE}================================{NC}")
        print(f"{BLUE}Summary{NC}")
        print(f"{BLUE}================================{NC}")
        print(f"Total links found: {total_links}")
        print(f"{GREEN}Valid links: {total_links - len(broken_links)}{NC}")
        print(f"{RED}Broken links: {len(broken_links)}{NC}")
        print()

        if broken_links:
            print(f"{RED}Broken Links:{NC}")
            print()
            for link in broken_links:
                print(f"  {RED}✗{NC} {link['file']}")
                print(f"    Link: [{link['text']}]({link['url']})")
                print(f"    Error: {link['error']}")
                print(f"    Type: {link['type']}")
                print()

            return 1
        else:
            print(f"{GREEN}✓ All links are valid!{NC}")
            return 0

def main():
    parser = argparse.ArgumentParser(description='Check all links in documentation')
    parser.add_argument('--verbose', '-v', action='store_true', help='Show all links')
    args = parser.parse_args()

    # Get docs directory
    script_dir = Path(__file__).parent
    repo_root = script_dir.parent
    docs_dir = repo_root / 'docs'

    if not docs_dir.exists():
        print(f"{RED}✗ Error: docs/ directory not found{NC}")
        sys.exit(1)

    # Check links
    checker = LinkChecker(docs_dir, verbose=args.verbose)
    sys.exit(checker.check_all_links())

if __name__ == '__main__':
    main()
