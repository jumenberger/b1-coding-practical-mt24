from uuv_mission.control import PDController

c = PDController()
print(c.update(reference=10, observation=0))
print(c.update(reference=10, observation=4))