# Import relevant modules
from uuv_mission import Submarine, Mission, ClosedLoop, PDController


sub = Submarine()

# Instantiate your controller (depending on your implementation)
A, B, C, D = sub.get_dynamics()
pd_controller = PDController(A, B, C, D)

closed_loop = ClosedLoop(sub, pd_controller)
mission = Mission.from_csv(
    'C:/Users/cvest/Claudio/Oxford/3rd Year/B1/b1-coding-practical-mt24/data/mission.csv'
)

trajectory = closed_loop.simulate_with_random_disturbances(mission)
trajectory.plot_completed_mission(mission)