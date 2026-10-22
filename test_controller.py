from uuv_mission.control import PDController

controller = PDController()

print(round(controller.compute(10, 7), 2))
print(round(controller.compute(10, 8), 2))
print(round(controller.compute(10, 10), 2))
print(round(controller.compute(10, 10), 2))
print(round(controller.compute(10, 12), 2))