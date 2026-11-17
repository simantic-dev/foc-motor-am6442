# Rotor state shared by plant.py (which integrates it) and encoder.py (which reads the shaft),
# in a module of its own so both scripts import the same instance.
class Motor:
    def __init__(self):
        self.id = self.iq = 0.0
        self.wm = 0.0
        self.theta_m = 0.6 / 7  # rotor starts 0.6 electrical rad off the d axis


MOTOR = Motor()
