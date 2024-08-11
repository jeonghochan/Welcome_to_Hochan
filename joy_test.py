import pygame

# print button, axes of joystick
pygame.init()
joystick_count = pygame.joystick.get_count()
print("Number of joysticks: {}".format(joystick_count))

for i in range(joystick_count):
    joystick = pygame.joystick.Joystick(i)
    

    print("Joystick {}".format(i))

    # Get the name from the OS for the controller/joystick
    name = joystick.get_name()
    print("Joystick name: {}".format(name))

    # Usually axis run in pairs, up/down for one, and left/right for
    # the other.
    axes = joystick.get_numaxes()
    print("Number of axes: {}".format(axes))

    buttons = joystick.get_numbuttons()
    print("Number of buttons: {}".format(buttons))

    hats = joystick.get_numhats()
    print("Number of hats: {}".format(hats))

    import time
    while True:
        pygame.event.get()
        print("Joystick name: {}".format(name))
        for i in range(axes):
            axis = joystick.get_axis(i)
            print("Axis {} value: {:>6.3f}".format(i, axis))


        for i in range(10):
            
            button = joystick.get_button(i)
            print("Button {:>2} value: {}".format(i, button))

        # Hat switch. All or nothing for direction, not like joysticks.
        # Value comes back in an array.

        for i in range(hats):
            hat = joystick.get_hat(i)
            #print("Hat {} value: {}".format(i, str(hat)))

        time.sleep(0.1)