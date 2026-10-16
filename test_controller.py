
from uuv_mission.control import PDController

controller = PDController()

# Test 1: Initial error is 2
action1 = controller(10, 8)
assert abs(action1 - 1.5) < 1e-10
print("Test 1 passed")

# Test 2: Error decreases from 2 to 1
action2 = controller(10, 9)
assert abs(action2 - (-0.45)) < 1e-10
print("Test 2 passed")

# Test 3: Error remains at 1
action3 = controller(10, 9)
assert abs(action3 - 0.15) < 1e-10
print("Test 3 passed")

print("All controller tests passed!")
