from build import shyphe


def draw_shape(window, shape, body, color):
    if isinstance(shape, shyphe.Circle):
        window.draw_circle(
            position=(body.position + shape.position.rotate(body.angle)).reflect((1, 0)) + (0, window.height),
            radius=int(shape.radius), color=color)
    elif isinstance(shape, shyphe.Polygon):
        points = [(body.position + (p + shape.position).rotate(body.angle)).reflect((1, 0)) + (0, window.height)
                  for p in shape.points]
        window.draw_polygon(points=points, color=color)


def draw_body(window, body, color):
    for shape in body.shapes:
        draw_shape(window, shape, body, color)
