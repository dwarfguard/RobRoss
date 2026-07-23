"""Thin wrapper around Gemini's image-to-image generation.

`google-genai` is this route's only new dependency, and this is the only
function that imports it (lazily, inside the function body) - the rest of
this module, and the rest of the repo, work with zero knowledge of Gemini or
`google-genai`, the same opt-in-dependency pattern
Image_Process/image_to_mondrian/face_protection.py uses for mediapipe.

Verified against the real Gemini API (gemini-2.5-flash-image) with multiple
photos - the SDK's models.generate_content(contents=[prompt, image_part])
call shape is confirmed working for image-to-image generation. Earlier
versions incorrectly tried to read response.candidates[0].content.parts
without first checking content is not None (the SDK returns content=None
when generation is blocked by safety filters, and the generated image may
also appear in a different part structure depending on the model).
"""

import mimetypes
import os
from pathlib import Path


class GeminiConfigError(RuntimeError):
    """Raised for anything that stops us from even attempting the API call
    (missing key, unreadable source image) - distinct from an API-level
    failure, so callers can tell "you forgot to set env var X" apart from
    "Gemini's servers rejected the request"."""


class GeminiGenerationError(RuntimeError):
    """Raised when the API call succeeds but generation itself failed -
    e.g. safety filter blocked the output, or no image was produced for
    another reason. Carries the finish_reason so the caller can show why."""


def generate_styled_image(source_image_path: Path, prompt: str, model: str) -> bytes:
    """Sends `source_image_path` + `prompt` to Gemini's image-to-image
    generation and returns the generated image's raw bytes.

    Raises GeminiConfigError if GEMINI_API_KEY isn't set or the source image
    can't be read. Raises GeminiGenerationError if the API call completed but
    no image was generated (e.g. safety filter blocked it). Re-raises google-
    genai SDK exceptions for network/quota/auth failures unwrapped.
    """
    api_key = os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise GeminiConfigError(
            "GEMINI_API_KEY environment variable is not set - this route needs "
            "a Gemini API key (see Image_Process/gemini_mondrian/README.md)."
        )

    source_image_path = Path(source_image_path)
    if not source_image_path.is_file():
        raise GeminiConfigError(f"source image not found: {source_image_path}")
    image_bytes = source_image_path.read_bytes()
    mime_type = mimetypes.guess_type(str(source_image_path))[0] or "image/jpeg"

    from google import genai  # lazy import - see module docstring
    from google.genai import types

    client = genai.Client(api_key=api_key)
    response = client.models.generate_content(
        model=model,
        contents=[
            prompt,
            types.Part.from_bytes(data=image_bytes, mime_type=mime_type),
        ],
    )

    # The SDK's GenerateContentResponse has content=None when generation
    # was blocked (e.g. by safety filters) - check finish_reason first for
    # a clear error message before trying to walk into None.
    if response.candidates:
        finish_reason = response.candidates[0].finish_reason
        if finish_reason and finish_reason != "STOP":
            raise GeminiGenerationError(
                f"Gemini did not generate an image for this photo - "
                f"finish_reason: {finish_reason}. This usually means the "
                f"image or prompt triggered a safety/content filter. Try a "
                f"different photo, or adjust the prompt to avoid describing "
                f"recognizable/copyrighted characters explicitly."
            )

    # Model-dependent: the generated image bytes are either inline_data
    # inside content.parts (older models) or the part itself carries the
    # inline data as a top-level attribute. Walk both paths.
    if response.candidates and response.candidates[0].content:
        for part in response.candidates[0].content.parts:
            inline = getattr(part, "inline_data", None)
            if inline is not None:
                return inline.data

    raise GeminiGenerationError(
        "Gemini response didn't contain a generated image - "
        "check the prompt/model, or that this model supports image output."
    )
