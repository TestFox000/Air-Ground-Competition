# -*- coding: utf-8 -*-
"""
坐标系验证脚本 - 测试坐标转换逻辑
"""

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
    """
    给定方块名计算中心坐标
    例如: get_block_center('N', 11) -> (0.5, 0.5)
    """
    col_idx = COL_LETTERS.index(col_name)
    row_idx = row_num - 1
    x = 10.5 - row_idx
    y = 13.5 - col_idx
    return x, y


def print_grid():
    """打印整个地图的坐标网格"""
    print("\n" + "=" * 80)
    print("地图坐标网格 (原点 N11右下角)")
    print("X轴: 行 11→1 (下→上),  Y轴: 列 N→A (右→左)")
    print("=" * 80)
    
    # 表头: 列号
    print(f"{'':>8}", end="")
    for letter in COL_LETTERS:
        print(f"{letter:>8}", end="")
    print()
    
    # 每行
    for row in range(11, 0, -1):
        print(f"{'':>8}", end="")
        for col_idx in range(14):
            x, y = get_block_center(COL_LETTERS[col_idx], row)
            cell = f"({x:.1f},{y:.1f})"
            print(f"{cell:>8}", end="")
        print()
    
    print("\n注: 格式为 (X坐标, Y坐标), 单位为方块")
    print("    X: 0.5=N11, 10.5=A1")
    print("    Y: 0.5=N11, 13.5=A1")


def test_coordinate_system():
    """测试坐标系正确性"""
    print("=" * 60)
    print("坐标系验证")
    print("=" * 60)
    
    # 测试关键点
    test_cases = [
        ("N11", 10.5, 13, 10, 0.5, 0.5),      # 原点附近
        ("A1", 0, 0, 0, 10.5, 13.5),          # 左上角
        ("A11", 10, 0, 10, 0.5, 13.5),         # 左下角
        ("N1", 0, 13, 0, 10.5, 0.5),           # 右上角
        ("H6", 5, 7, 5, 5.5, 6.5),             # 中间位置
    ]
    
    print("\n测试方块中心坐标:")
    for name, col_idx, row_idx, exp_x, exp_y in test_cases[:5]:
        x, y = get_block_center(COL_LETTERS[col_idx], row_idx + 1)
        match = "✓" if (abs(x - exp_x) < 0.01 and abs(y - exp_y) < 0.01) else "✗"
        print(f"  {name}: col={col_idx} row={row_idx} -> ({x:.1f}, {y:.1f}) 期望({exp_x}, {exp_y}) {match}")
    
    # 测试起始位置
    print(f"\n起始位置: ({START_X}, {START_Y}) -> {get_block_name(START_X, START_Y)}")
    
    # 模拟移动测试
    print("\n模拟小车移动:")
    print(f"  起点: ({START_X}, {START_Y}) = {get_block_name(START_X, START_Y)}")
    
    # 向前移动1米
    delta_m = 1.0
    delta_blocks = delta_m / BLOCK_SIZE
    new_x = START_X + delta_blocks
    new_y = START_Y
    print(f"  前进{delta_m}米: ({new_x:.2f}, {new_y:.2f}) = {get_block_name(new_x, new_y)}")
    
    # 向左移动1米
    new_x = START_X
    new_y = START_Y + delta_blocks
    print(f"  左移{delta_m}米: ({new_x:.2f}, {new_y:.2f}) = {get_block_name(new_x, new_y)}")
    
    # 前进+左移
    new_x = START_X + delta_blocks
    new_y = START_Y + delta_blocks
    print(f"  前进+左移: ({new_x:.2f}, {new_y:.2f}) = {get_block_name(new_x, new_y)}")
    
    # 回到起点
    print(f"  起点: ({START_X}, {START_Y}) = {get_block_name(START_X, START_Y)}")
    
    print("\n" + "=" * 60)
    print("如果前进方向不对, 修改 FLIP_X = True")
    print("如果左移方向不对, 修改 FLIP_Y = True")
    print("=" * 60)


if __name__ == "__main__":
    print_grid()
    test_coordinate_system()
