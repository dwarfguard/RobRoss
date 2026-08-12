"""Optional Gemini image-to-line-art preprocessing for the sketch route.

The dependency and API key are intentionally lazy. Callers can always fall
back to the local outline extractor without importing ``google-genai``.
Errors expose only stable reason codes; raw SDK messages are never copied into
job metadata because they may contain request details.
"""

import mimetypes
import os
from pathlib import Path


class SemanticPreprocessingError(RuntimeError):
    """A safe, categorized semantic preprocessing failure."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def _classify_exception(error: Exception) -> str:
    """Map SDK/network failures to non-sensitive diagnostic categories."""
    name = type(error).__name__.lower()
    message = str(error).lower()
    if isinstance(error, TimeoutError) or "timeout" in name or "deadline" in message:
        return "timeout"
    if any(token in message for token in ("quota", "resource exhausted", "429")):
        return "quota"
    if any(token in message for token in ("safety", "blocked", "prohibited")):
        return "safety_rejection"
    if any(token in name for token in ("connection", "network", "transport")):
        return "network"
    if any(token in message for token in ("connection", "network", "dns")):
        return "network"
    return "api_failure"


def generate_outline_image(
    source_image_path: Path,
    prompt: str,
    model: str,
    timeout_seconds: float,
) -> bytes:
    """Generate structural black-on-white line art and return encoded bytes.

    ``types.HttpOptions.timeout`` is expressed in milliseconds by the SDK.
    Missing setup, safety rejection, malformed output, timeouts, quota, and
    network failures are converted to :class:`SemanticPreprocessingError`.
    """
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise SemanticPreprocessingError("missing_api_key")

    source_image_path = Path(source_image_path)
    if not source_image_path.is_file():
        raise SemanticPreprocessingError("source_unreadable")

    try:
        from google import genai
        from google.genai import types
    except (ImportError, ModuleNotFoundError) as error:
        raise SemanticPreprocessingError("missing_dependency") from None

    image_bytes = source_image_path.read_bytes()
    mime_type = mimetypes.guess_type(str(source_image_path))[0] or "image/jpeg"

    try:
        http_options = types.HttpOptions(timeout=int(float(timeout_seconds) * 1000))
        client = genai.Client(api_key=api_key, http_options=http_options)
        response = client.models.generate_content(
            model=model,
            contents=[
                prompt,
                types.Part.from_bytes(data=image_bytes, mime_type=mime_type),
            ],
        )
    except Exception as error:
        raise SemanticPreprocessingError(_classify_exception(error)) from None

    candidates = getattr(response, "candidates", None) or []
    prompt_feedback = getattr(response, "prompt_feedback", None)
    block_reason = getattr(prompt_feedback, "block_reason", None)
    if block_reason:
        raise SemanticPreprocessingError("safety_rejection")
    if candidates:
        finish_reason = getattr(candidates[0], "finish_reason", None)
        finish_name = getattr(finish_reason, "name", finish_reason)
        normalized_finish = str(finish_name).upper().rsplit(".", 1)[-1]
        if finish_name and normalized_finish != "STOP":
            safety_finishes = {
                "SAFETY",
                "BLOCKLIST",
                "PROHIBITED_CONTENT",
                "IMAGE_SAFETY",
                "RECITATION",
            }
            reason = (
                "safety_rejection"
                if normalized_finish in safety_finishes
                else "api_failure"
            )
            raise SemanticPreprocessingError(reason)

        content = getattr(candidates[0], "content", None)
        for part in getattr(content, "parts", None) or []:
            inline_data = getattr(part, "inline_data", None)
            data = getattr(inline_data, "data", None)
            if data:
                return bytes(data)

    raise SemanticPreprocessingError("malformed_response")
