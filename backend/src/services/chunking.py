"""Semantic chunking service for Markdown content."""

import re
from typing import List
from pathlib import Path

import mistune
import tiktoken

from ..core import MIN_CHUNK_SIZE_TOKENS, MAX_CHUNK_SIZE_TOKENS, CHUNK_OVERLAP_TOKENS
from ..models.chunk import TextChunk

# Initialize tiktoken encoder for token counting
encoder = tiktoken.get_encoding("cl100k_base")


def count_tokens(text: str) -> int:
    """Count tokens in text using tiktoken."""
    return len(encoder.encode(text))


def chunk_section(
    section_text: str,
    section_heading: str,
    file_path: str,
    chunk_index_start: int = 0
) -> List[TextChunk]:
    """Recursively chunk a section into token-sized segments.

    Args:
        section_text: The text content to chunk
        section_heading: The heading/title of this section
        file_path: Source file path
        chunk_index_start: Starting index for chunks

    Returns:
        List of TextChunk objects with 300-1,000 tokens each
    """
    chunks: List[TextChunk] = []
    token_count = count_tokens(section_text)

    # If section fits in one chunk, return it
    if MIN_CHUNK_SIZE_TOKENS <= token_count <= MAX_CHUNK_SIZE_TOKENS:
        chunk = TextChunk(
            file_path=file_path,
            section=section_heading,
            text=section_text.strip(),
            token_count=token_count,
            chunk_index=chunk_index_start,
            overlap_with_prev=False,
            overlap_with_next=False,
        )
        return [chunk]

    # If section is too small, still return it (edge case handling)
    if token_count < MIN_CHUNK_SIZE_TOKENS:
        chunk = TextChunk(
            file_path=file_path,
            section=section_heading,
            text=section_text.strip(),
            token_count=token_count,
            chunk_index=chunk_index_start,
            overlap_with_prev=False,
            overlap_with_next=False,
        )
        return [chunk]

    # Section is too large - split recursively

    # Try splitting on paragraphs (double newlines)
    paragraphs = re.split(r'\n\n+', section_text)

    if len(paragraphs) > 1:
        # Split on paragraphs
        current_chunk_text = ""
        current_token_count = 0
        chunk_index = chunk_index_start

        for para in paragraphs:
            para = para.strip()
            if not para:
                continue

            para_tokens = count_tokens(para)

            # If adding this paragraph would exceed max, save current chunk
            if current_token_count + para_tokens > MAX_CHUNK_SIZE_TOKENS and current_chunk_text:
                # Add overlap from next paragraph
                overlap_text = para[:200] if len(para) > 200 else para  # Approximate overlap
                final_text = current_chunk_text + "\n\n" + overlap_text
                final_tokens = count_tokens(final_text)

                chunk = TextChunk(
                    file_path=file_path,
                    section=section_heading,
                    text=current_chunk_text.strip(),
                    token_count=current_token_count,
                    chunk_index=chunk_index,
                    overlap_with_prev=chunk_index > chunk_index_start,
                    overlap_with_next=True,
                )
                chunks.append(chunk)
                chunk_index += 1

                # Start new chunk with overlap from previous
                overlap_from_prev = current_chunk_text.split()[-CHUNK_OVERLAP_TOKENS:]
                current_chunk_text = " ".join(overlap_from_prev) + "\n\n" + para
                current_token_count = count_tokens(current_chunk_text)
            else:
                # Add paragraph to current chunk
                if current_chunk_text:
                    current_chunk_text += "\n\n" + para
                else:
                    current_chunk_text = para
                current_token_count = count_tokens(current_chunk_text)

        # Add final chunk
        if current_chunk_text.strip():
            chunk = TextChunk(
                file_path=file_path,
                section=section_heading,
                text=current_chunk_text.strip(),
                token_count=current_token_count,
                chunk_index=chunk_index,
                overlap_with_prev=chunk_index > chunk_index_start,
                overlap_with_next=False,
            )
            chunks.append(chunk)

        return chunks

    # If no paragraph breaks, split on sentences
    sentences = re.split(r'(?<=[.!?])\s+', section_text)

    if len(sentences) > 1:
        current_chunk_text = ""
        current_token_count = 0
        chunk_index = chunk_index_start

        for sent in sentences:
            sent = sent.strip()
            if not sent:
                continue

            sent_tokens = count_tokens(sent)

            if current_token_count + sent_tokens > MAX_CHUNK_SIZE_TOKENS and current_chunk_text:
                chunk = TextChunk(
                    file_path=file_path,
                    section=section_heading,
                    text=current_chunk_text.strip(),
                    token_count=current_token_count,
                    chunk_index=chunk_index,
                    overlap_with_prev=chunk_index > chunk_index_start,
                    overlap_with_next=True,
                )
                chunks.append(chunk)
                chunk_index += 1

                # Start new chunk with overlap
                overlap_from_prev = current_chunk_text.split()[-CHUNK_OVERLAP_TOKENS:]
                current_chunk_text = " ".join(overlap_from_prev) + " " + sent
                current_token_count = count_tokens(current_chunk_text)
            else:
                if current_chunk_text:
                    current_chunk_text += " " + sent
                else:
                    current_chunk_text = sent
                current_token_count = count_tokens(current_chunk_text)

        if current_chunk_text.strip():
            chunk = TextChunk(
                file_path=file_path,
                section=section_heading,
                text=current_chunk_text.strip(),
                token_count=current_token_count,
                chunk_index=chunk_index,
                overlap_with_prev=chunk_index > chunk_index_start,
                overlap_with_next=False,
            )
            chunks.append(chunk)

        return chunks

    # Fallback: hard split by token count
    tokens = encoder.encode(section_text)
    chunk_index = chunk_index_start

    for i in range(0, len(tokens), MAX_CHUNK_SIZE_TOKENS - CHUNK_OVERLAP_TOKENS):
        chunk_tokens = tokens[i:i + MAX_CHUNK_SIZE_TOKENS]
        chunk_text = encoder.decode(chunk_tokens)

        chunk = TextChunk(
            file_path=file_path,
            section=section_heading,
            text=chunk_text.strip(),
            token_count=len(chunk_tokens),
            chunk_index=chunk_index,
            overlap_with_prev=i > 0,
            overlap_with_next=i + MAX_CHUNK_SIZE_TOKENS < len(tokens),
        )
        chunks.append(chunk)
        chunk_index += 1

    return chunks


def process_markdown_file(file_path: str) -> List[TextChunk]:
    """Parse a Markdown file and chunk it semantically.

    Args:
        file_path: Path to the Markdown file

    Returns:
        List of TextChunk objects for the entire file
    """
    path = Path(file_path)

    if not path.exists():
        raise FileNotFoundError(f"File not found: {file_path}")

    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Parse with mistune to get sections
    markdown = mistune.create_markdown(renderer='ast')
    ast = markdown(content)

    chunks: List[TextChunk] = []
    chunk_index = 0

    # Extract sections by headings
    current_section = "Introduction"  # Default section name
    current_content = ""

    def extract_text(node):
        """Recursively extract text from AST node."""
        if isinstance(node, str):
            return node
        if isinstance(node, dict):
            if node.get('type') == 'text':
                return node.get('raw', '')
            if 'children' in node:
                return ''.join(extract_text(child) for child in node['children'])
        if isinstance(node, list):
            return ''.join(extract_text(item) for item in node)
        return ""

    # Process AST nodes
    for node in ast:
        if isinstance(node, dict):
            if node.get('type') == 'heading':
                # Save previous section
                if current_content.strip():
                    section_chunks = chunk_section(
                        current_content.strip(),
                        current_section,
                        file_path,
                        chunk_index
                    )
                    chunks.extend(section_chunks)
                    chunk_index += len(section_chunks)

                # Start new section
                current_section = extract_text(node.get('children', []))
                current_content = ""
            else:
                # Add content to current section
                current_content += extract_text(node) + "\n\n"

    # Process final section
    if current_content.strip():
        section_chunks = chunk_section(
            current_content.strip(),
            current_section,
            file_path,
            chunk_index
        )
        chunks.extend(section_chunks)

    return chunks
