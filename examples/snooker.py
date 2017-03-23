from build import shyphe
import pygame_go as pygo
import draw_shape

SHAPE_SIZE = 50

window = pygo.window(1200, 600, frame_rate=60)
world = shyphe.World(0.3)

for i in range(5):
    y = 300 - i * SHAPE_SIZE
    for c in range(i + 1):
        x = window.width / 2 + i / 2 * SHAPE_SIZE - c * SHAPE_SIZE
        body = shyphe.Body(position=(x, window.height - y))
        body.add_shape(shyphe.Circle(mass=1, radius=SHAPE_SIZE // 2))
        world.add_body(body)


while window.active():
    window.fill("white")

    for event in window.events():
        if event.is_key() and event.key == " ":
            body = shyphe.Body(position=(pygo.mouse_position()[0], 10), velocity=(0, 25))
            body.add_shape(shyphe.Circle(mass=1, radius=SHAPE_SIZE // 2))
            world.add_body(body)

    for body in world.bodies:
        if body.position.x < 0 and body.velocity.x < 0 or body.position.x > window.width and body.velocity.x > 0:
            body.apply_impulse((2 * -body.velocity.x * body.mass, 0), (0, 0))
        if body.position.y < 0 and body.velocity.y < 0 or body.position.y > window.height and body.velocity.y > 0:
            body.apply_impulse((0, 2 * -body.velocity.y * body.mass), (0, 0))

    colliding = []
    world.begin_frame()
    while world.has_next_collision():
        ctr = world.next_collision()
        cola, colb = world.calculate_collision(ctr, shyphe.CollisionParameters(0.9))
        cola.apply_impulse()
        colb.apply_impulse()
        world.finished_collision(ctr, True)
        colliding.extend((cola.body, colb.body))
    world.end_frame()

    for body in world.bodies:
        draw_shape.draw_body(window, body, "red" if body in colliding else "green")
    window.update()
