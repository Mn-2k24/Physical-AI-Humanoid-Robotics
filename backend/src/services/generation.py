"""Answer generation service using flan-t5-base."""

from typing import List, Optional

from transformers import AutoTokenizer, AutoModelForSeq2SeqLM

from ..core import Config
from ..models.query import RetrievedChunk

# Initialize model and tokenizer (singleton pattern)
_tokenizer: Optional[AutoTokenizer] = None
_model: Optional[AutoModelForSeq2SeqLM] = None


def get_answer_model():
    """Get or initialize the answer generation model and tokenizer."""
    global _tokenizer, _model

    if _tokenizer is None or _model is None:
        _tokenizer = AutoTokenizer.from_pretrained(Config.ANSWER_MODEL_NAME)
        _model = AutoModelForSeq2SeqLM.from_pretrained(Config.ANSWER_MODEL_NAME)

    return _tokenizer, _model


def generate_answer(query: str, chunks: List[RetrievedChunk]) -> str:
    """Generate an answer to the query based on retrieved chunks.

    Args:
        query: The user's query text
        chunks: List of retrieved chunks (should be 3 chunks)

    Returns:
        Generated answer text
    """
    tokenizer, model = get_answer_model()

    # Construct prompt with context
    context_parts = []
    for i, chunk in enumerate(chunks, start=1):
        context_parts.append(
            f"[Source {i} from {chunk.file_path} - {chunk.section}]\n{chunk.text}"
        )

    context = "\n\n".join(context_parts)

    # Build the prompt
    prompt = f"""Answer the question based only on the following context:

Context:
{context}

Question: {query}

Answer:"""

    # Generate answer
    inputs = tokenizer(prompt, return_tensors="pt", max_length=512, truncation=True)
    outputs = model.generate(
        inputs.input_ids,
        max_length=200,
        num_beams=4,
        early_stopping=True,
        temperature=0.7,
    )

    answer = tokenizer.decode(outputs[0], skip_special_tokens=True)

    return answer.strip()
