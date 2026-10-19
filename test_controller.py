
from control import PDController

controller = PDController()

# Test 1: First time step, previous error is zero
action1 = controller(reference=10, observation=8)
assert abs(action1 - 1.5) < 1e-9

# Test 2: Error decreases from 2 to 1
action2 = controller(reference=10, observation=9)
assert abs(action2 - (-0.45)) < 1e-9

# Test 3: Reset the controller and repeat the first test
controller.reset()
action3 = controller(reference=10, observation=8)
assert abs(action3 - 1.5) < 1e-9

print("All controller tests passed!")
