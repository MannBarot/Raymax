# flake8: noqa
import math
import sys
import time
import traceback
from signal import SIGINT, signal

import pygame

from mobile_base_sdk import MobileBaseSDK
from reachy_sdk import ReachySDK
import os
os.environ["DISPLAY"] = ":0"  # for Linux/headless cases

msg = """
Keyboard controller for Reachy mobile base.

Arrow Keys : holonomic movement
  UP    = forward
  DOWN  = backward
  LEFT  = strafe left
  RIGHT = strafe right

A / D : rotate left / right

+/- : increase/decrease linear speed (+-0.05 m/s)
[/] : increase/decrease angular speed (+-0.2 rad/s)

CTRL-C or ESC to quit
"""


class KeyboardController:
    def __init__(self):
        print("Starting KeyboardController!")

        pygame.init()
        pygame.display.init()
        # A small window is needed to capture keyboard events
        self.screen = pygame.display.set_mode((400, 200))
        pygame.display.set_caption("Reachy Mobile Base Controller")

        self.lin_speed_ratio = 0.5
        self.rot_speed_ratio = 2.0

        print(msg)
        # Allow overriding the host via environment variable or first CLI argument
        ip_address = os.environ.get('REACHY_HOST') or (sys.argv[1] if len(sys.argv) > 1 else "localhost")
        print(f"Connecting to {ip_address}")
        # If connecting to a local Reachy instance, use ReachySDK with_mobile_base=True
        try:
            if ip_address in ("localhost", "127.0.0.1"):
                reachy = ReachySDK(host=ip_address, with_mobile_base=True)
                self.mobile_base = reachy.mobile_base
            else:
                # For remote/mobile-base-only connections, use MobileBaseSDK(ip)
                self.mobile_base = MobileBaseSDK(ip_address)
            print(f"Connected to mobile base at {ip_address}")
        except Exception as e:
            print(f"Failed to connect to mobile base at {ip_address}: {e}")
            raise

        def emergency_shutdown_(signal_received, frame):
            self.emergency_shutdown("SIGINT received")

        signal(SIGINT, emergency_shutdown_)

    def emergency_shutdown(self, msg=""):
        self.mobile_base.set_speed(x_vel=0.0, y_vel=0.0, rot_vel=0.0)
        print(f"Emergency shutdown: {msg}. Setting speeds to 0.")
        raise RuntimeError(msg)

    def tick_controller(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.emergency_shutdown("Window closed")
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.emergency_shutdown("ESC pressed")
                # Linear speed adjustment
                elif event.key == pygame.K_EQUALS or event.key == pygame.K_PLUS:
                    self.lin_speed_ratio = min(3.0, self.lin_speed_ratio + 0.05)
                    print(f"Max linear speed: {self.lin_speed_ratio:.2f} m/s")
                elif event.key == pygame.K_MINUS:
                    self.lin_speed_ratio = max(0.0, self.lin_speed_ratio - 0.05)
                    print(f"Max linear speed: {self.lin_speed_ratio:.2f} m/s")
                # Angular speed adjustment
                elif event.key == pygame.K_RIGHTBRACKET:
                    self.rot_speed_ratio = min(12.0, self.rot_speed_ratio + 0.2)
                    print(f"Max angular speed: {self.rot_speed_ratio:.2f} rad/s")
                elif event.key == pygame.K_LEFTBRACKET:
                    self.rot_speed_ratio = max(0.0, self.rot_speed_ratio - 0.2)
                    print(f"Max angular speed: {self.rot_speed_ratio:.2f} rad/s")

    def speeds_from_keyboard(self):
        keys = pygame.key.get_pressed()

        x_vel = 0.0
        y_vel = 0.0
        rot_vel = 0.0

        # Translations (arrow keys)
        if keys[pygame.K_UP]:
            x_vel += self.lin_speed_ratio
        if keys[pygame.K_DOWN]:
            x_vel -= self.lin_speed_ratio
        if keys[pygame.K_LEFT]:
            y_vel += self.lin_speed_ratio
        if keys[pygame.K_RIGHT]:
            y_vel -= self.lin_speed_ratio

        # Rotation (A/D)
        if keys[pygame.K_a]:
            rot_vel += self.rot_speed_ratio
        if keys[pygame.K_d]:
            rot_vel -= self.rot_speed_ratio

        return x_vel, y_vel, rot_vel

    def main_tick(self):
        self.tick_controller()
        x_vel, y_vel, rot_vel = self.speeds_from_keyboard()
        self.mobile_base.set_speed(
            x_vel=x_vel,
            y_vel=y_vel,
            rot_vel=rot_vel * 180.0 / math.pi
        )

        print(
            f"\rx_vel: {x_vel:.2f} m/s, y_vel: {y_vel:.2f} m/s, "
            f"rot_vel: {rot_vel:.2f} rad/s | "
            f"max_lin: {self.lin_speed_ratio:.2f}, max_rot: {self.rot_speed_ratio:.2f}",
            end=""
        )
        time.sleep(0.01)


def main():
    controller = KeyboardController()

    try:
        while True:
            controller.main_tick()
    except Exception as e:
        traceback.print_exc()
    finally:
        controller.mobile_base.set_speed(x_vel=0.0, y_vel=0.0, rot_vel=0.0)
        pygame.quit()


if __name__ == "__main__":
    main()