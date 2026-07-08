import time
from rangerair import RangerAir, MotionMode

with RangerAir("can0") as bot:
    bot.wait_for_feedback()          # confirm the link is live
    bot.enable()                     # enter CAN command mode
    bot.clear_errors()               # release e-stop error / clear faults
    bot.set_mode(MotionMode.ACKERMANN)

    bot.drive(linear=0.15)           # 0.15 m/s forward, held at 50 Hz
    time.sleep(1.0)
    bot.drive(linear=0.15, steer=0.3)  # curve left
    time.sleep(1.0)
    bot.stop()
# leaving the `with` block stops the robot and drops it to standby