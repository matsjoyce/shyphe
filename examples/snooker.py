from build import physics
import pygame_go as pygo
import random

SHAPE_SIZE = 50

window = pygo.window(1200, 600, frame_rate=20)

circle_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE)
circle_image.draw_circle(position=circle_image.center, radius=SHAPE_SIZE // 2, color="green")
colliding_circle_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE)
colliding_circle_image.draw_circle(position=colliding_circle_image.center, radius=SHAPE_SIZE // 2, color="red")
square_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE, color="green")
colliding_square_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE, color="red")

world = physics.World(1)

bodies = []

for i in range(5):
    y = 300 - i * SHAPE_SIZE
    for c in range(i + 1):
        x = window.width / 2 + i / 2 * SHAPE_SIZE - c * SHAPE_SIZE
        body = physics.Body(position=(x, y))
        body.add_shape(physics.Circle(mass=1, radius=SHAPE_SIZE // 2))
        world.add_body(body)
        bodies.append(body)


while window.active():
    window.fill("white")
    colliding = []

    for event in window.events():
        if event.is_key() and event.key == " ":

            body = physics.Body(position=(pygo.mouse_position()[0], window.height - 10), velocity=(0, 25))
            body.add_shape(physics.Circle(mass=1, radius=SHAPE_SIZE // 2))
            world.add_body(body)
            bodies.append(body)

    for body in bodies:
        if body.position.x < 0 and body.velocity.x < 0 or body.position.x > window.width and body.velocity.x > 0:
            body.apply_impulse((2 * -body.velocity.x * body.mass, 0), (0, 0))
        if body.position.y < 0 and body.velocity.y < 0 or body.position.y > window.height and body.velocity.y > 0:
            body.apply_impulse((0, 2 * -body.velocity.y * body.mass), (0, 0))

    world.begin_frame()
    while world.has_next_collision():
        ctr = world.next_collision()
        cola, colb = world.calculate_collision(ctr, physics.CollisionParameters(0.9))
        cola.apply_impulse()
        colb.apply_impulse()
        world.finished_collision(ctr, True)
        colliding.extend((cola.body, colb.body))
    world.end_frame()

    for body in bodies:
        if body not in colliding:
            window.draw_image(circle_image,
                              align=pygo.center, position=body.position.as_tuple())
        else:
            window.draw_image(colliding_circle_image,
                              align=pygo.center, position=body.position.as_tuple())
    window.update()
