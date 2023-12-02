# Import the library that allows us to read our keyboard
from pynput import keyboard as kb

def key_pressed(key):
    global Auto
    print(Auto)
    Auto = True

def key_released(key):
    global Auto
    print(Auto)
    if key == kb.Key.up:  # Finish program
        Auto = False
    if key == kb.Key.ctrl:  # Finish program
        return False

def main():
    global Auto
    Auto = False

    listener = kb.Listener(
        on_press=lambda key: key_pressed(key),
        on_release=lambda key: key_released(key)
    )
    listener.start()
    listener.join()

if __name__ == "__main__":
    main()