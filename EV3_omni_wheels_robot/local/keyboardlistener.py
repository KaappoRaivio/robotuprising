from pynput import keyboard

def _onPress(key):
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys


    print(f"{key} pressed!")

    return True


def _onRelease(key):
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys

    print(f"{key} released!")

    return True


listener = keyboard.Listener(on_press=_onPress, on_release=_onRelease)
listener.start()

while True:
    pass
