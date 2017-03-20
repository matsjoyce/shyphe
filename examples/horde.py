from build import physics
import pygame_go as pygo
import random

SHAPE_SIZE = 25
NUMBER_OF_BODIES = 300

window = pygo.window(1200, 600, frame_rate=20)

circle_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE)
circle_image.draw_circle(position=circle_image.center, radius=SHAPE_SIZE // 2, color="green")
colliding_circle_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE)
colliding_circle_image.draw_circle(position=colliding_circle_image.center, radius=SHAPE_SIZE // 2, color="red")
square_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE, color="green")
colliding_square_image = pygo.image(SHAPE_SIZE, SHAPE_SIZE, color="red")

world = physics.World(1)

bodies = []
circles = []

bodies_x = window.width // SHAPE_SIZE
bodies_x_spacing = window.width // bodies_x + 10
bodies_x = window.width // bodies_x_spacing
bodies_y = window.height // SHAPE_SIZE
bodies_y_spacing = window.height // bodies_y + 10
bodies_y = window.width // bodies_y_spacing

for i in range(NUMBER_OF_BODIES):
    body = physics.Body(velocity=(random.randrange(10), random.randrange(10)), position=(bodies_x_spacing * (i % bodies_x),
                                                                                         bodies_y_spacing * (i // bodies_x)))
    if random.randrange(2):
        body.add_shape(physics.Circle(mass=1, radius=SHAPE_SIZE // 2))
        circles.append(body)
    else:
        body.add_shape(physics.Polygon(points=[(-SHAPE_SIZE // 2, -SHAPE_SIZE // 2),
                                               (-SHAPE_SIZE // 2, SHAPE_SIZE // 2),
                                               (SHAPE_SIZE // 2, SHAPE_SIZE // 2),
                                               (SHAPE_SIZE // 2, -SHAPE_SIZE // 2)], mass=1))
    world.add_body(body)
    bodies.append(body)


while window.active():
    window.fill("white")
    colliding = []

    for body in bodies:
        if body.position.x < 0 and body.velocity.x < 0 or body.position.x > window.width and body.velocity.x > 0:
            body.apply_impulse((2 * -body.velocity.x * body.mass, 0), (0, 0))
        if body.position.y < 0 and body.velocity.y < 0 or body.position.y > window.height and body.velocity.y > 0:
            body.apply_impulse((0, 2 * -body.velocity.y * body.mass), (0, 0))

    world.begin_frame()
    while world.has_next_collision():
        ctr = world.next_collision()
        cola, colb = world.calculate_collision(ctr, physics.CollisionParameters(1, 10000, 1))
        cola.body.apply_impulse(cola.impulse, cola.touch_point)
        colb.body.apply_impulse(colb.impulse, colb.touch_point)
        colliding.extend((cola.body, colb.body))
        world.finished_collision(ctr, True)
    world.end_frame()

    for body in bodies:
        if body not in colliding:
            window.draw_image((circle_image if body in circles else square_image),
                              align=pygo.center, position=body.position.as_tuple())
        else:
            window.draw_image((colliding_circle_image if body in circles else colliding_square_image),
                              align=pygo.center, position=body.position.as_tuple())
    window.update()
