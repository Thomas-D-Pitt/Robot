import pygame

def main():
    pygame.init()

    # Initialize joystick
    if pygame.joystick.get_count() == 0:
        print("No joystick detected!")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Joystick Name: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")
    print(f"Number of Hats: {joystick.get_numhats()}")

    running = True
    while running:
        pygame.event.pump()  # Process joystick events

        # Read axis values
        for i in range(joystick.get_numaxes()):
            axis_value = joystick.get_axis(i)
            print(f"Axis {i}: {axis_value:.2f}")

        # Read button states
        for i in range(joystick.get_numbuttons()):
            if joystick.get_button(i):
                print(f"Button {i} pressed")

        # Read hat (D-pad) values
        for i in range(joystick.get_numhats()):
            hat_value = joystick.get_hat(i)
            print(f"Hat {i}: {hat_value}")

        pygame.time.wait(100)  # Wait to prevent spamming

if __name__ == "__main__":
    main()
