from typing import Dict


BODY25_LOWER_BODY: Dict[str, int] = {
    "hip": 9,
    "knee": 10,
    "ankle": 11,
    "big_toe": 22,
    "small_toe": 23,
    "heel": 24,
}


BODY25_LEFT_LOWER_BODY: Dict[str, int] = {
    "hip": 12,
    "knee": 13,
    "ankle": 14,
    "big_toe": 19,
    "small_toe": 20,
    "heel": 21,
}


BODY25_RIGHT_LOWER_BODY = BODY25_LOWER_BODY


def keypoint_indices_for_side(side: str) -> Dict[str, int]:
    if side == "left":
        return BODY25_LEFT_LOWER_BODY
    return BODY25_RIGHT_LOWER_BODY
