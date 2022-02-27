"""Modulate moving on circle"""
import turtle
from math import sqrt

STEP_X = 10.
STEP_Y = 10.
R = 100.
X, Y = 0, 0

WITDH, HEIGHT = (2. * R,) * 2

"""
[START]
Перемещаемся на верх окружности по Y.
Замер
Пока не кончатся хорды:
{
    Переехать в левую точку хорды (хорда следующих замеров)
    Спуститься вниз на шаг
    Замеры по движению вправо до крайней точки хорды
}
Переместиться в самый низ окружности
Замер
"""
def turtle_go():
    global X, Y
    turtle.goto(X, Y)
    turtle.dot(5)

def reset():
    global X, Y
    X, Y = 0, 0
    turtle.home()
    # turtle.position()

def move_x(x):
    global X
    X = x

def move_y(y):
    global Y
    Y = y

def measure(until_x):
    # Длина хорды
    to_ride = 2 * until_x
    # Округленное количество шагов
    steps = int(to_ride / STEP_X)
    for i in range(steps):
        move_x(X + STEP_X)
        output_coords("\t")
        turtle_go()
    # Завершающий замер на хорде (последний шаг не всегда совпадает с краем)
    move_x(until_x)
    turtle_go()


def output_coords(add=""):
    print(f"{add}X: {X}\t Y: {Y}")

def calculate_next_x(y):
    """Calculates next X coord to move."""
    x = sqrt(R**2 - y**2)
    return x

def main():
    # Move to the circle top
    reset()
    move_y(R)
    output_coords()
    turtle_go()
    # Начинаем мерить внутри круга в цикле
    hords_amount = int(HEIGHT / STEP_Y) - 2

    for n in range(hords_amount + 1):
        # перемещаем ниже на 1 шаг
        move_y(Y - STEP_Y)
        # считаем следующую позицию по Х и перемещаем влево (-х)
        next_x = calculate_next_x(Y)
        move_x(-next_x)
        output_coords()
        turtle_go()
        measure(next_x)
    # Последняя нижняя точка окружности
    move_y(-R)
    print("\nLAST DOT:\n")
    next_x = calculate_next_x(Y)
    move_x(-next_x)
    output_coords()
    turtle_go()


if __name__ == "__main__":
    main()
