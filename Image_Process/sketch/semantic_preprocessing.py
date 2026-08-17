"""Optional Qwen image-to-line-art preprocessing for the sketch route.

The HTTP dependency and API key are intentionally lazy. Callers can always
fall back to the local outline extractor without importing ``requests``.
Errors expose only stable reason codes; raw API messages are never copied into
job metadata because they may contain request details.
"""

import base64
import mimetypes
import os
from pathlib import Path


QWEN_BEIJING_API_URL = (
    "https://ws-2xu46zkb1q2h7j9k.cn-beijing.maas.aliyuncs.com/api/v1/services/"
    "aigc/multimodal-generation/generation"
)


class SemanticPreprocessingError(RuntimeError):
    """A safe, categorized semantic preprocessing failure."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


def _classify_exception(error: Exception) -> str:
    """Map API/network failures to non-sensitive diagnostic categories."""
    name = type(error).__name__.lower()
    message = str(error).lower()
    if isinstance(error, TimeoutError) or "timeout" in name or "deadline" in message:
        return "timeout"
    if any(
        token in message
        for token in ("quota", "resource exhausted", "rate limit", "ratelimit", "429")
    ):
        return "quota"
    if any(
        token in message
        for token in (
            "safety",
            "blocked",
            "prohibited",
            "data inspection",
            "datainspection",
        )
    ):
        return "safety_rejection"
    if any(
        token in name for token in ("connection", "network", "transport", "proxy")
    ):
        return "network"
    if any(token in message for token in ("connection", "network", "dns", "proxy")):
        return "network"
    return "api_failure"


def _response_value(value, key, default=None):
    """Read response-like objects and plain dictionaries uniformly."""
    if isinstance(value, dict):
        return value.get(key, default)
    return getattr(value, key, default)


def _first_output_image_url(response) -> str:
    output = _response_value(response, "output", {})
    choices = _response_value(output, "choices", []) or []
    if not choices:
        return ""
    message = _response_value(choices[0], "message", {})
    content = _response_value(message, "content", []) or []
    for part in content:
        image_url = _response_value(part, "image")
        if isinstance(image_url, str) and image_url.startswith("https://"):
            return image_url
    return ""


def generate_outline_image(
    source_image_path: Path,
    prompt: str,
    model: str,
    timeout_seconds: float,
) -> bytes:
    """Generate structural black-on-white line art and return encoded bytes.

    Missing setup, safety rejection, malformed output, timeouts, quota, and
    network failures are converted to :class:`SemanticPreprocessingError`.
    """
    api_key = os.environ.get("DASHSCOPE_API_KEY")
    if not api_key:
        raise SemanticPreprocessingError("missing_api_key")

    source_image_path = Path(source_image_path)
    if not source_image_path.is_file():
        raise SemanticPreprocessingError("source_unreadable")

    try:
        import requests
    except (ImportError, ModuleNotFoundError) as error:
        raise SemanticPreprocessingError("missing_dependency") from None

    image_bytes = source_image_path.read_bytes()
    mime_type = mimetypes.guess_type(str(source_image_path))[0] or "image/jpeg"
    image_data = (
        f"data:{mime_type};base64,"
        f"{base64.b64encode(image_bytes).decode('ascii')}"
    )

    try:
        # This workspace-specific endpoint matches this project's
        # Beijing-region API keys.
        response = requests.post(
            QWEN_BEIJING_API_URL,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            json={
                "model": model,
                "input": {
                    "messages": [
                        {
                            "role": "user",
                            "content": [
                                {"image": image_data},
                                {"text": prompt},
                            ],
                        }
                    ]
                },
                "parameters": {
                    "n": 1,
                    "watermark": False,
                    # Preserve the specialized robot line-art instructions.
                    "prompt_extend": False,
                },
            },
            timeout=float(timeout_seconds),
        )
        status_code = response.status_code
        response_body = response.json()
        if status_code != 200:
            code = _response_value(response_body, "code", "")
            message = _response_value(response_body, "message", "")
            raise RuntimeError(f"{status_code} {code} {message}")

        image_url = _first_output_image_url(response_body)
        if not image_url:
            raise SemanticPreprocessingError("malformed_response")
        download = requests.get(image_url, timeout=float(timeout_seconds))
        download.raise_for_status()
        if not download.content:
            raise SemanticPreprocessingError("malformed_response")
        return bytes(download.content)
    except SemanticPreprocessingError:
        raise
    except Exception as error:
        raise SemanticPreprocessingError(_classify_exception(error)) from None
