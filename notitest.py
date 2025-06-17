import time
from wpilib import Notifier

def say_hi():
    print(f"[{time.monotonic():.2f}] Hello from Notifier!")

notifier = Notifier(say_hi)
notifier.setName("TestNotifier")
notifier.startPeriodic(1.0)

# Keep the main thread alive
while True:
    time.sleep(0.5)
