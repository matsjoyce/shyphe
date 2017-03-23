from build import physics
import pygame_go as pygo
import random
import draw_shape

SHAPE_SIZE = 50
NUMBER_OF_BODIES = 50

window = pygo.window(1200, 600, frame_rate=20)
world = physics.World(1)

bodies_x_spacing = window.width // (window.width // SHAPE_SIZE) + 50
bodies_x = window.width // bodies_x_spacing
bodies_y_spacing = window.height // (window.height // SHAPE_SIZE) + 50
bodies_y = window.width // bodies_y_spacing

for i in range(NUMBER_OF_BODIES):
    body = physics.Body(velocity=(random.randrange(10), random.randrange(10)), position=(bodies_x_spacing * (i % bodies_x),
                                                                                         bodies_y_spacing * (i // bodies_x)))
    if random.randrange(2):
        body.add_shape(physics.Circle(mass=1, radius=SHAPE_SIZE // 2))
    else:
        body.add_shape(physics.Polygon(points=[(-SHAPE_SIZE // 2, -SHAPE_SIZE // 2),
                                               (-SHAPE_SIZE // 2, SHAPE_SIZE // 2),
                                               (SHAPE_SIZE // 2, SHAPE_SIZE // 2),
                                               (SHAPE_SIZE // 2, -SHAPE_SIZE // 2)], mass=1))
    world.add_body(body)

while window.active():
    window.fill("white")

    for body in world.bodies:
        if body.position.x < 0 and body.velocity.x < 0 or body.position.x > window.width and body.velocity.x > 0:
            body.apply_impulse((2 * -body.velocity.x * body.mass, 0), (0, 0))
        if body.position.y < 0 and body.velocity.y < 0 or body.position.y > window.height and body.velocity.y > 0:
            body.apply_impulse((0, 2 * -body.velocity.y * body.mass), (0, 0))

    colliding = []
    world.begin_frame()
    while world.has_next_collision():
        ctr = world.next_collision()
        cola, colb = world.calculate_collision(ctr, physics.CollisionParameters(1))
        cola.apply_impulse()
        colb.apply_impulse()
        colliding.extend((cola.body, colb.body))
        world.finished_collision(ctr, True)
    world.end_frame()

    for body in world.bodies:
        draw_shape.draw_body(window, body, "red" if body in colliding else "green")
    window.update()
