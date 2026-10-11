# -*- coding: utf-8 -*-
"""
Coordinate system verification script.
Tests block name <-> coordinate conversion logic.
"""

import sys

COL_LETTERS = 'ABCDEFGHIJKLMN'
MAP_COLS = 14
MAP_ROWS = 11
BLOCK_SIZE = 0.35
START_X = 1.5
START_Y = 0.5


def get_block_name(x, y):
    row_idx = round(10.5 - x)
    col_idx = round(13.5 - y)
    row_idx = max(0, min(MAP_ROWS - 1, row_idx))
    col_idx = max(0, min(MAP_COLS - 1, col_idx))
    letter = COL_LETTERS[col_idx]
    number = row_idx + 1
    return f"{letter}{number}"


def get_block_center(col_name, row_num):
    """Return block center coordinates in block units."""
    col_idx = COL_LETTERS.index(col_name)
    row_idx = row_num - 1
    x = 10.5 - row_idx
    y = 13.5 - col_idx
    return x, y


def print_grid():
    """Print the full coordinate grid."""
    print("")
    print("=" * 80)
    print("Map Coordinate Grid (origin N11 bottom-right)")
    print("X-axis: Row 11->1 (down->up),  Y-axis: Col N->A (right->left)")
    print("=" * 80)

    # header
    print(f"{'':>8}", end="")
    for letter in COL_LETTERS:
        print(f"{letter:>8}", end="")
    print()

    # each row
    for row in range(11, 0, -1):
        print(f"{'':>8}", end="")
        for col_idx in range(14):
            x, y = get_block_center(COL_LETTERS[col_idx], row)
            cell = f"({x:.1f},{y:.1f})"
            print(f"{cell:>8}", end="")
        print()

    print("")
    print("Note: Format = (X,Y), unit = blocks")
    print("    X: 0.5=N11, 10.5=A1")
    print("    Y: 0.5=N11, 13.5=A1")


def test_coordinate_system():
    """Verify coordinate system correctness. Returns True if all pass."""
    print("=" * 60)
    print("Coordinate System Verification")
    print("=" * 60)

    # test key points: (name, col_idx, row_idx, exp_col, exp_row, exp_x, exp_y)
    test_cases = [
        ("N11", 13, 10, 13, 10, 0.5, 0.5),
        ("A1",  0,  0,  0,  0,  10.5, 13.5),
        ("A11", 0,  10, 0,  10, 0.5, 13.5),
        ("N1",  13, 0,  13, 0,  10.5, 0.5),
        ("H6",  7,  5,  7,  5,  5.5, 6.5),
    ]

    all_ok = True
    print("")
    print("Block center tests:")
    for name, col_idx, row_idx, exp_col, exp_row, exp_x, exp_y in test_cases:
        x, y = get_block_center(COL_LETTERS[col_idx], row_idx + 1)
        ok = abs(x - exp_x) < 0.01 and abs(y - exp_y) < 0.01
        if not ok:
            all_ok = False
        match = "[OK]" if ok else "[FAIL]"
        print(f"  {name}: col={col_idx} row={row_idx} -> ({x:.1f}, {y:.1f}) expected({exp_x}, {exp_y}) {match}")

    # test start position
    print(f"")
    print(f"Start position: ({START_X}, {START_Y}) -> {get_block_name(START_X, START_Y)}")

    # motion simulation
    print(f"")
    print("Movement simulation:")
    print(f"  Start: ({START_X}, {START_Y}) = {get_block_name(START_X, START_Y)}")

    delta_m = 1.0
    delta_blocks = delta_m / BLOCK_SIZE
    new_x = START_X + delta_blocks
    new_y = START_Y
    print(f"  Forward {delta_m:g}m: ({new_x:.2f}, {new_y:.2f}) = {get_block_name(new_x, new_y)}")

    new_x = START_X
    new_y = START_Y + delta_blocks
    print(f"  Left {delta_m:g}m: ({new_x:.2f}, {new_y:.2f}) = {get_block_name(new_x, new_y)}")

    new_x = START_X + delta_blocks
    new_y = START_Y + delta_blocks
    print(f"  Forward+Left: ({new_x:.2f}, {new_y:.2f}) = {get_block_name(new_x, new_y)}")

    print(f"  Start: ({START_X}, {START_Y}) = {get_block_name(START_X, START_Y)}")

    print("")
    print("=" * 60)
    print("Check direction signs if forward/left is wrong")
    print("=" * 60)

    if not all_ok:
        print("[FAIL] Some coordinate tests failed")
        return False
    print("[OK] All coordinate tests passed")
    return True


if __name__ == "__main__":
    print_grid()
    ok = test_coordinate_system()
    sys.exit(0 if ok else 1)
