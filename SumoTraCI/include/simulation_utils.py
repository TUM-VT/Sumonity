import json
from typing import Any, Dict
import re

def is_valid_json(s: str) -> bool:
    """Check if a string is valid JSON."""
    try:
        json.loads(s)
        return True
    except ValueError:
        return False

def extract_number(input_string: str) -> str:
    """Extract number from string between ':' and '_'."""
    pattern = r":(\d+)_"
    match = re.search(pattern, input_string)
    return match.group(1) if match else None

def clamp_value(value: float, min_value: float, max_value: float) -> float:
    """Clamp a value between min and max."""
    return max(min_value, min(value, max_value)) 