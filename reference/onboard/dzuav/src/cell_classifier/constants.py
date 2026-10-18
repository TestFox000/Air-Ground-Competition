"""Shared label definitions and image normalization settings."""

LETTERS = tuple("ABCDEFGHIJKLMN")
ROWS = tuple(str(value) for value in range(1, 12))

LETTER_TO_INDEX = {label: index for index, label in enumerate(LETTERS)}
ROW_TO_INDEX = {label: index for index, label in enumerate(ROWS)}

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

