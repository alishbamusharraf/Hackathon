"""
Document ingestion script for book content.
Parses markdown files, chunks them, and stores in Qdrant.
"""

import os
import re
import asyncio
from pathlib import Path
from typing import List, Tuple
import hashlib

# Add parent directory to path for imports
import sys
sys.path.insert(0, str(Path(__file__).parent.parent))

from models.document import DocumentChunk, IngestResponse
from services.embeddings import get_embeddings_service
from services.qdrant_service import get_qdrant_service


def parse_markdown_file(file_path: Path) -> Tuple[str, dict]:
    """
    Parse a markdown file and extract content and metadata.
    
    Args:
        file_path: Path to the markdown file
        
    Returns:
        Tuple of (content, metadata)
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Extract frontmatter if present
    metadata = {}
    if content.startswith('---'):
        try:
            end_idx = content.index('---', 3)
            frontmatter = content[3:end_idx].strip()
            content = content[end_idx + 3:].strip()
            
            # Parse simple YAML-like frontmatter
            for line in frontmatter.split('\n'):
                if ':' in line:
                    key, value = line.split(':', 1)
                    metadata[key.strip()] = value.strip()
        except ValueError:
            pass
    
    # Extract title from first heading
    title_match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    if title_match:
        metadata['title'] = title_match.group(1).strip()
    
    return content, metadata


def chunk_text(
    text: str,
    chunk_size: int = 500,
    chunk_overlap: int = 50
) -> List[str]:
    """
    Split text into overlapping chunks.
    
    Args:
        text: Text to chunk
        chunk_size: Maximum characters per chunk
        chunk_overlap: Overlap between chunks
        
    Returns:
        List of text chunks
    """
    # Split by paragraphs first
    paragraphs = re.split(r'\n\n+', text)
    
    chunks = []
    current_chunk = ""
    
    for para in paragraphs:
        para = para.strip()
        if not para:
            continue
        
        # If paragraph itself is too long, split by sentences
        if len(para) > chunk_size:
            sentences = re.split(r'(?<=[.!?])\s+', para)
            for sentence in sentences:
                if len(current_chunk) + len(sentence) + 1 <= chunk_size:
                    current_chunk = f"{current_chunk} {sentence}".strip()
                else:
                    if current_chunk:
                        chunks.append(current_chunk)
                    current_chunk = sentence
        else:
            if len(current_chunk) + len(para) + 2 <= chunk_size:
                current_chunk = f"{current_chunk}\n\n{para}".strip()
            else:
                if current_chunk:
                    chunks.append(current_chunk)
                current_chunk = para
    
    if current_chunk:
        chunks.append(current_chunk)
    
    # Add overlap between chunks
    if chunk_overlap > 0 and len(chunks) > 1:
        overlapped_chunks = [chunks[0]]
        for i in range(1, len(chunks)):
            prev_overlap = chunks[i-1][-chunk_overlap:] if len(chunks[i-1]) > chunk_overlap else chunks[i-1]
            overlapped_chunks.append(f"{prev_overlap}... {chunks[i]}")
        chunks = overlapped_chunks
    
    return chunks


def get_module_and_chapter(file_path: Path, docs_root: Path) -> Tuple[str, str]:
    """
    Extract module and chapter names from file path.
    
    Args:
        file_path: Path to the markdown file
        docs_root: Root docs directory
        
    Returns:
        Tuple of (module, chapter)
    """
    relative = file_path.relative_to(docs_root)
    parts = relative.parts
    
    if len(parts) >= 2:
        module = parts[0]
        chapter = file_path.stem.replace('-', ' ').title()
    else:
        module = "General"
        chapter = file_path.stem.replace('-', ' ').title()
    
    return module, chapter


async def ingest_all_docs(
    docs_path: str = "../docs",
    chunk_size: int = 500,
    chunk_overlap: int = 50
) -> IngestResponse:
    """
    Ingest all markdown documents from the docs directory.
    
    Args:
        docs_path: Path to docs directory
        chunk_size: Maximum characters per chunk
        chunk_overlap: Overlap between chunks
        
    Returns:
        IngestResponse with statistics
    """
    # Resolve docs path
    script_dir = Path(__file__).parent
    docs_root = (script_dir.parent.parent / "docs").resolve()
    if not docs_root.exists():
        docs_root = Path(docs_path).resolve()
    
    print(f"Ingesting documents from: {docs_root}")
    
    # Get services
    embeddings_service = get_embeddings_service()
    qdrant_service = get_qdrant_service()
    
    # Ensure collection exists
    await qdrant_service.ensure_collection_exists()
    
    # Find all markdown files
    md_files = list(docs_root.rglob("*.md"))
    print(f"Found {len(md_files)} markdown files")
    
    all_chunks: List[DocumentChunk] = []
    errors = []
    
    for file_path in md_files:
        try:
            print(f"Processing: {file_path.name}")
            
            # Parse file
            content, metadata = parse_markdown_file(file_path)
            
            # Get module and chapter
            module, chapter = get_module_and_chapter(file_path, docs_root)
            
            # Chunk content
            text_chunks = chunk_text(content, chunk_size, chunk_overlap)
            
            # Create DocumentChunk objects
            for i, chunk_text_content in enumerate(text_chunks):
                chunk_id = hashlib.md5(
                    f"{file_path}:{i}:{chunk_text_content[:50]}".encode()
                ).hexdigest()
                
                chunk = DocumentChunk(
                    id=chunk_id,
                    text=chunk_text_content,
                    source=str(file_path.relative_to(docs_root)),
                    chapter=chapter,
                    module=module,
                    chunk_index=i,
                    metadata=metadata
                )
                all_chunks.append(chunk)
            
            print(f"  Created {len(text_chunks)} chunks")
            
        except Exception as e:
            error_msg = f"Error processing {file_path}: {str(e)}"
            print(f"  {error_msg}")
            errors.append(error_msg)
    
    print(f"\nTotal chunks to embed: {len(all_chunks)}")
    
    # Generate embeddings
    if all_chunks:
        print("Generating embeddings...")
        chunk_texts = [c.text for c in all_chunks]
        embeddings = embeddings_service.embed_texts(chunk_texts)
        
        print("Storing in Qdrant...")
        num_stored = await qdrant_service.upsert_chunks(all_chunks, embeddings)
        print(f"Stored {num_stored} chunks")
    
    return IngestResponse(
        success=len(errors) == 0,
        documents_processed=len(md_files),
        chunks_created=len(all_chunks),
        errors=errors
    )


async def main():
    """CLI entry point for ingestion."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Ingest book documents into Qdrant")
    parser.add_argument("--docs-path", default="../docs", help="Path to docs directory")
    parser.add_argument("--chunk-size", type=int, default=500, help="Max chunk size")
    parser.add_argument("--chunk-overlap", type=int, default=50, help="Chunk overlap")
    parser.add_argument("--dry-run", action="store_true", help="Don't actually store")
    
    args = parser.parse_args()
    
    result = await ingest_all_docs(
        docs_path=args.docs_path,
        chunk_size=args.chunk_size,
        chunk_overlap=args.chunk_overlap
    )
    
    print(f"\n=== Ingestion Complete ===")
    print(f"Success: {result.success}")
    print(f"Documents: {result.documents_processed}")
    print(f"Chunks: {result.chunks_created}")
    if result.errors:
        print(f"Errors: {len(result.errors)}")


if __name__ == "__main__":
    asyncio.run(main())
