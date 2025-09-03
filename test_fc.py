#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import argparse

def main(pin, pull):
    GPIO.setmode(GPIO.BCM)
    # Choix du pull-up/down selon l'argument
    pud = GPIO.PUD_UP if pull == "up" else GPIO.PUD_DOWN
    GPIO.setup(pin, GPIO.IN, pull_up_down=pud)

    last = GPIO.input(pin)
    print(f"[{time.strftime('%H:%M:%S')}] FC pin {pin} initial state: {last} (pull_{pull})")
    try:
        while True:
            cur = GPIO.input(pin)
            if cur != last:
                now = time.strftime('%H:%M:%S')
                print(f"[{now}] FC pin {pin} changed: {last} -> {cur}")
                last = cur
            # Debounce / réduire la charge CPU
            time.sleep(0.01)  # 10 ms
    except KeyboardInterrupt:
        print("\nArrêt demandé par l'utilisateur.")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up. Bye.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test pin fin de course (FC).")
    parser.add_argument("--pin", type=int, default=6, help="Numéro de pin BCM (défaut 5)")
    parser.add_argument("--pull", choices=["up","down"], default="up",
                        help="Pull-up ou pull-down selon le câblage (défaut: up)")
    args = parser.parse_args()
    main(args.pin, args.pull)
